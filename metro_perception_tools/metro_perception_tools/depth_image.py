"""
Convert a PointCloud2 stream into a colorized panoramic depth image.

A cloud with a `label` field (the detector's labelled_points: 0 background, 1 corridor,
2 obstacle) is drawn in grey by depth, with corridor pixels green and obstacle pixels red;
any other cloud uses the RealSense Jet palette.
"""

import array
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField


# Elevations closer than this are one channel: the finest Pandar128 spacing is 0.086 deg.
CHANNEL_TOLERANCE_DEG = 0.02


def point_cloud_xyz_ring(
    message: PointCloud2, stride: int = 1
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Return compact XYZ coordinates and lidar ring numbers, or None without a ring field."""
    fields = {field.name: field for field in message.fields}
    missing = {'x', 'y', 'z'} - fields.keys()
    if missing:
        raise ValueError(f"PointCloud2 has no fields: {', '.join(sorted(missing))}")

    xyz_fields = [fields[name] for name in ('x', 'y', 'z')]
    if any(field.datatype != PointField.FLOAT32 for field in xyz_fields):
        raise ValueError('x, y and z fields must use FLOAT32')
    if 'ring' in fields and fields['ring'].datatype != PointField.UINT16:
        raise ValueError('ring field must use UINT16')

    byte_order = '>' if message.is_bigendian else '<'
    scalar_type = np.dtype(f'{byte_order}f4')
    ring_type = np.dtype(f'{byte_order}u2')
    count = message.width * message.height
    if count == 0:
        # A view with a field offset does not fit an empty buffer; an empty cloud is valid.
        return np.empty((0, 3), scalar_type), np.empty(0, ring_type) if 'ring' in fields else None
    step = max(1, stride)
    coordinates = [
        np.ndarray(
            shape=(count,),
            dtype=scalar_type,
            buffer=message.data,
            offset=field.offset,
            strides=(message.point_step,),
        )[::step]
        for field in xyz_fields
    ]
    if 'ring' not in fields:
        return np.column_stack(coordinates), None
    rings = np.ndarray(
        shape=(count,),
        dtype=ring_type,
        buffer=message.data,
        offset=fields['ring'].offset,
        strides=(message.point_step,),
    )[::step]
    return np.column_stack(coordinates), rings


def point_cloud_labels(message: PointCloud2, stride: int = 1) -> Optional[np.ndarray]:
    """Return the per-point UINT8 `label` field, or None if the cloud has none."""
    fields = {field.name: field for field in message.fields}
    if 'label' not in fields:
        return None
    if fields['label'].datatype != PointField.UINT8:
        raise ValueError('label field must use UINT8')
    count = message.width * message.height
    if count == 0:
        return np.empty(0, np.uint8)
    return np.ndarray(
        shape=(count,),
        dtype=np.uint8,
        buffer=message.data,
        offset=fields['label'].offset,
        strides=(message.point_step,),
    )[::max(1, stride)]


def rings_from_elevation(
    xyz: np.ndarray, min_range: float, channel_share: float = 0.1
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Recover lidar ring numbers from the elevation angle, top channel first.

    Every return of one lidar channel has the same elevation, so a frame shows one sharp
    elevation value per channel. Values holding at least `channel_share` of the largest one's
    returns are the channels. A return between two channels (e.g. a point added to the cloud
    on its own vertical grid) is given to both, so that an inserted surface leaves no empty
    channel behind it; such points are appended as copies. Returns closer than `min_range`,
    including empty (0, 0, 0) points, get ring 0. Returns (xyz, rings) of the same length.
    """
    xyz, rings, _ = rings_from_elevation_indexed(xyz, min_range, channel_share)
    return xyz, rings


def elevation_channels(
    xyz: np.ndarray, min_range: float, channel_share: float = 0.1
) -> Optional[np.ndarray]:
    """
    Return the elevation (deg) of every lidar channel seen in the cloud, ascending.

    Every return of one lidar channel has the same elevation, so a frame shows one sharp
    elevation value per channel. Values holding at least `channel_share` of the largest one's
    returns are the channels. None when fewer than two channels are seen.
    """
    x, y, z = xyz.T.astype(np.float64)
    ranges = np.sqrt(x * x + y * y + z * z)
    measured = np.isfinite(ranges) & (ranges >= max(min_range, 1e-3))
    if not np.any(measured):
        return None
    elevation = np.degrees(np.arctan2(z[measured], np.hypot(x[measured], y[measured])))
    values, counts = np.unique(np.round(elevation, 3), return_counts=True)
    group = np.cumsum(np.r_[True, np.diff(values) > CHANNEL_TOLERANCE_DEG]) - 1
    group_counts = np.bincount(group, weights=counts)
    centers = np.bincount(group, weights=values * counts) / group_counts
    channels = centers[group_counts >= channel_share * group_counts.max()]
    return channels if len(channels) > 1 else None


def rings_from_elevation_indexed(
    xyz: np.ndarray,
    min_range: float,
    channel_share: float = 0.1,
    channels: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Return rings_from_elevation's (xyz, rings) and the input index of every returned point.

    The index maps the copies appended for returns between two channels back to their source,
    so per-point values such as labels can follow them. `channels` from elevation_channels
    may be passed to skip finding them again in this cloud.
    """
    source = np.arange(len(xyz))
    rings = np.zeros(len(xyz), dtype=np.uint16)
    x, y, z = xyz.T.astype(np.float32)
    horizontal = np.sqrt(x * x + y * y)
    measured = np.isfinite(horizontal) & np.isfinite(z)
    measured &= np.sqrt(horizontal * horizontal + z * z) >= max(min_range, 1e-3)
    if not np.any(measured):
        return xyz, rings, source
    if channels is None:
        channels = elevation_channels(xyz, min_range, channel_share)
    if channels is None:
        return xyz, rings, source
    # Compare slopes z / horizontal instead of angles: the slope grows with the elevation, and
    # near a channel an angle step is the slope step divided by (1 + slope^2).
    slope = z[measured] / np.maximum(horizontal[measured], 1e-6)
    channel_slopes = np.tan(np.radians(channels)).astype(np.float32)
    upper = np.clip(np.searchsorted(channel_slopes, slope), 1, len(channels) - 1)
    lower = upper - 1
    to_degrees = np.float32(180.0 / np.pi) / (1.0 + slope * slope)
    above_lower = (slope - channel_slopes[lower]) * to_degrees
    below_upper = (channel_slopes[upper] - slope) * to_degrees
    nearest = np.where(above_lower <= below_upper, lower, upper)
    rings[measured] = len(channels) - 1 - nearest
    between = np.minimum(above_lower, below_upper) > CHANNEL_TOLERANCE_DEG
    if not np.any(between):
        return xyz, rings, source
    other = np.where(nearest == lower, upper, lower)[between]
    return (
        np.concatenate([xyz, xyz[measured][between]]),
        np.concatenate([rings, (len(channels) - 1 - other).astype(np.uint16)]),
        np.concatenate([source, source[measured][between]]),
    )


def channel_count(rings: np.ndarray) -> int:
    """Return the number of recovered channels, or 0 if the cloud shows fewer than two."""
    return int(rings.max()) + 1 if rings.any() else 0


def project_nearest(
    xyz: np.ndarray,
    rings: np.ndarray,
    labels: Optional[np.ndarray],
    width: int,
    height: int,
    min_depth: float,
    max_depth: float,
    min_azimuth_deg: float,
    max_azimuth_deg: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Project XYZ using lidar rings as rows and azimuth as columns, nearest point per pixel.

    Returns the depth image (inf where empty) and the label of the nearest point of every
    pixel (0 where empty or without labels).
    """
    depth = np.full(width * height, np.inf, dtype=np.float32)
    label_image = np.zeros(width * height, dtype=np.uint8)
    if xyz.size == 0:
        return depth.reshape(height, width), label_image.reshape(height, width)

    x, y, z = xyz.T
    ranges = np.sqrt(x * x + y * y + z * z)
    azimuth = np.degrees(np.arctan2(y, x))
    ring_min = float(np.min(rings))
    ring_max = float(np.max(rings))
    valid = (
        np.isfinite(ranges)
        & (ranges >= min_depth)
        & (ranges <= max_depth)
        & (azimuth >= min_azimuth_deg)
        & (azimuth <= max_azimuth_deg)
    )
    index = np.flatnonzero(valid)
    if index.size == 0:
        return depth.reshape(height, width), label_image.reshape(height, width)

    columns = (
        (azimuth[index] - min_azimuth_deg) * (width - 1) / (max_azimuth_deg - min_azimuth_deg)
    ).astype(np.int64)
    if ring_max == ring_min:
        rows = np.zeros(index.shape, dtype=np.int64)
    else:
        rows = (
            (rings[index].astype(np.float32) - ring_min) * (height - 1) / (ring_max - ring_min)
        ).astype(np.int64)
    pixels = np.clip(rows, 0, height - 1) * width + np.clip(columns, 0, width - 1)
    # One sort of pixel and range (in 0.1 mm) together; the first entry of a pixel is nearest.
    key = (pixels << 32) | np.minimum(ranges[index] * 1e4, 2**31).astype(np.int64)
    order = np.argsort(key)
    sorted_pixels = pixels[order]
    first = np.r_[True, sorted_pixels[1:] != sorted_pixels[:-1]]
    nearest = index[order[first]]
    depth[sorted_pixels[first]] = ranges[nearest]
    if labels is not None:
        label_image[sorted_pixels[first]] = labels[nearest]
    return depth.reshape(height, width), label_image.reshape(height, width)


def project_depth_panorama(
    xyz: np.ndarray,
    rings: np.ndarray,
    width: int,
    height: int,
    min_depth: float,
    max_depth: float,
    min_azimuth_deg: float,
    max_azimuth_deg: float,
) -> np.ndarray:
    """Project XYZ using lidar rings as rows and azimuth as columns."""
    return project_nearest(xyz, rings, None, width, height, min_depth, max_depth,
                           min_azimuth_deg, max_azimuth_deg)[0]


def project_labels(
    xyz: np.ndarray,
    rings: np.ndarray,
    labels: np.ndarray,
    width: int,
    height: int,
    min_depth: float,
    max_depth: float,
    min_azimuth_deg: float,
    max_azimuth_deg: float,
) -> np.ndarray:
    """Label of the nearest point per pixel of project_depth_panorama's image (0 when empty)."""
    return project_nearest(xyz, rings, labels, width, height, min_depth, max_depth,
                           min_azimuth_deg, max_azimuth_deg)[1]


def colorize_labelled_depth(
    depth: np.ndarray,
    labels: np.ndarray,
    min_depth: float,
    max_depth: float,
    histogram_equalization: bool = True,
) -> Tuple[np.ndarray, int]:
    """Grey depth (near is light) with corridor pixels green and obstacle pixels red."""
    valid = np.isfinite(depth)
    normalized = np.zeros(depth.shape, dtype=np.float32)
    valid_depth = depth[valid]
    if histogram_equalization and valid_depth.size:
        _, inverse, counts = np.unique(valid_depth, return_inverse=True, return_counts=True)
        normalized[valid] = (np.cumsum(counts, dtype=np.float64) / valid_depth.size)[inverse]
    else:
        normalized[valid] = np.clip((valid_depth - min_depth) / (max_depth - min_depth), 0.0, 1.0)
    level = (60 + 195 * (1.0 - normalized)).astype(np.uint8)
    result = np.repeat(level[..., None], 3, axis=2)
    corridor = valid & (labels == 1)
    result[corridor] = np.stack(
        [level[corridor] // 6, level[corridor], level[corridor] // 4], axis=1)
    result[valid & (labels == 2)] = (255, 40, 40)
    result[~valid] = 0
    return result, int(np.count_nonzero(valid))


def colorize_depth(
    depth: np.ndarray,
    min_depth: float,
    max_depth: float,
    histogram_equalization: bool = True,
) -> Tuple[np.ndarray, int]:
    """Apply the Intel RealSense Jet palette to a depth image."""
    valid = np.isfinite(depth)
    normalized = np.zeros(depth.shape, dtype=np.float32)
    valid_depth = depth[valid]
    if histogram_equalization and valid_depth.size:
        _, inverse, counts = np.unique(
            valid_depth, return_inverse=True, return_counts=True
        )
        cumulative = np.cumsum(counts, dtype=np.float64) / valid_depth.size
        normalized[valid] = cumulative[inverse]
    else:
        normalized[valid] = np.clip(
            (valid_depth - min_depth) / (max_depth - min_depth), 0.0, 1.0
        )

    # Jet anchors from librealsense/src/proc/colorizer.cpp.
    stops = np.array([0.0, 0.25, 0.5, 0.75, 1.0], dtype=np.float32)
    colors = np.array(
        [
            [0, 0, 255],
            [0, 255, 255],
            [255, 255, 0],
            [255, 0, 0],
            [50, 0, 0],
        ],
        dtype=np.float32,
    )
    result = np.zeros((*depth.shape, 3), dtype=np.uint8)
    for channel in range(3):
        result[..., channel] = np.interp(
            normalized, stops, colors[:, channel]
        ).astype(np.uint8)
    result[~valid] = 0
    return result, int(np.count_nonzero(valid))


CHANNEL_REFRESH_FRAMES = 50


def video_frame_index(stamp_ns: int, first_stamp_ns: int, fps: float) -> int:
    """Map a ROS timestamp to its constant-frame-rate video index."""
    elapsed_seconds = max(0, stamp_ns - first_stamp_ns) / 1_000_000_000
    return int(round(elapsed_seconds * fps))


class DepthImageNode(Node):
    """ROS 2 node producing a colorized panoramic depth image."""

    def __init__(self) -> None:
        super().__init__('lidar_depth_image')
        self.declare_parameter('input_topic', '/lidar_points')
        self.declare_parameter('output_topic', '/lidar/depth_image')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 128)
        self.declare_parameter('min_depth', 1.0)
        self.declare_parameter('max_depth', 300.0)
        self.declare_parameter('min_azimuth_deg', -140.0)
        self.declare_parameter('max_azimuth_deg', -40.0)
        self.declare_parameter('histogram_equalization', True)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('video_path', '')
        self.declare_parameter('video_fps', 10.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_azimuth = float(self.get_parameter('min_azimuth_deg').value)
        self.max_azimuth = float(self.get_parameter('max_azimuth_deg').value)
        self.histogram_equalization = bool(
            self.get_parameter('histogram_equalization').value
        )
        self.point_stride = int(self.get_parameter('point_stride').value)
        video_path = str(self.get_parameter('video_path').value)
        self.video_path = Path(video_path).expanduser() if video_path else None
        self.video_fps = float(self.get_parameter('video_fps').value)
        self.video_writer = None
        self.video_first_stamp_ns = None
        self.video_frames_written = 0
        self.last_video_frame = None
        self.elevation_rows_reported = False
        # Channel elevations of a cloud without rings; found again every CHANNEL_REFRESH_FRAMES.
        self.channels = None
        self.channels_age = 0

        self._validate_parameters()
        # Latest cloud only: a slow frame drops older clouds instead of delaying the video.
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, self.output_topic, image_qos)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._point_cloud_callback,
            input_qos,
        )
        self.frame_count = 0
        self.get_logger().info(
            f'PointCloud2 {self.input_topic} -> RGB depth image {self.output_topic} '
            f'({self.image_width}x{self.image_height}, '
            f'{self.min_depth:.1f}-{self.max_depth:.1f} m, '
            f'azimuth {self.min_azimuth:.1f}..{self.max_azimuth:.1f} deg)'
        )

    def _validate_parameters(self) -> None:
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError('image_width and image_height must be positive')
        if self.min_depth < 0.0 or self.max_depth <= self.min_depth:
            raise ValueError('max_depth must be greater than non-negative min_depth')
        if self.max_azimuth <= self.min_azimuth:
            raise ValueError('max_azimuth_deg must be greater than min_azimuth_deg')
        if self.point_stride <= 0:
            raise ValueError('point_stride must be positive')
        if self.video_fps <= 0.0:
            raise ValueError('video_fps must be positive')

    def _write_video_frame(self, rgb: np.ndarray, stamp) -> None:
        if self.video_path is None:
            return

        bgr = np.ascontiguousarray(rgb[..., ::-1])
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.video_writer is None:
            self.video_path.parent.mkdir(parents=True, exist_ok=True)
            self.video_writer = cv2.VideoWriter(
                str(self.video_path),
                cv2.VideoWriter_fourcc(*'mp4v'),
                self.video_fps,
                (self.image_width, self.image_height),
            )
            if not self.video_writer.isOpened():
                self.video_writer = None
                raise ValueError(f'Cannot create video: {self.video_path}')
            self.video_first_stamp_ns = stamp_ns
            self.get_logger().info(
                f'Recording timestamp-synchronized video to {self.video_path}'
            )

        target_index = video_frame_index(
            stamp_ns, self.video_first_stamp_ns, self.video_fps
        )
        if target_index < self.video_frames_written:
            return

        while (
            self.last_video_frame is not None
            and self.video_frames_written < target_index
        ):
            self.video_writer.write(self.last_video_frame)
            self.video_frames_written += 1

        self.video_writer.write(bgr)
        self.video_frames_written += 1
        self.last_video_frame = bgr

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        try:
            xyz, rings = point_cloud_xyz_ring(message, self.point_stride)
            labels = point_cloud_labels(message, self.point_stride)
            if rings is None:
                if self.channels is None or self.channels_age >= CHANNEL_REFRESH_FRAMES:
                    self.channels = elevation_channels(xyz, self.min_depth)
                    self.channels_age = 0
                self.channels_age += 1
                xyz, rings, source = rings_from_elevation_indexed(
                    xyz, self.min_depth, channels=self.channels)
                if labels is not None:
                    labels = labels[source]
                # Reported once, from the first cloud whose channels could be recovered.
                channels = channel_count(rings)
                if channels and not self.elevation_rows_reported:
                    self.elevation_rows_reported = True
                    self.get_logger().info(
                        'PointCloud2 has no ring field: rings recovered from elevation '
                        f'({channels} channels)'
                    )
            depth, label_image = project_nearest(
                xyz, rings, labels, self.image_width, self.image_height, self.min_depth,
                self.max_depth, self.min_azimuth, self.max_azimuth)
            if labels is not None:
                rgb, populated_pixels = colorize_labelled_depth(
                    depth, label_image, self.min_depth, self.max_depth,
                    self.histogram_equalization)
            else:
                rgb, populated_pixels = colorize_depth(
                    depth,
                    self.min_depth,
                    self.max_depth,
                    self.histogram_equalization,
                )
            self._write_video_frame(rgb, message.header.stamp)
        except (TypeError, ValueError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return

        image = Image()
        image.header = message.header
        image.height = self.image_height
        image.width = self.image_width
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = self.image_width * 3
        # An array.array skips rclpy's per-byte check of a bytes value (~0.1 s per image).
        image.data = array.array('B', rgb.tobytes())
        self.publisher.publish(image)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Published frame {self.frame_count}: {populated_pixels} depth pixels'
            )

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            duration = self.video_frames_written / self.video_fps
            if rclpy.ok():
                self.get_logger().info(
                    f'Video saved: {self.video_path} '
                    f'({self.video_frames_written} frames, {duration:.2f} s)'
                )
            self.video_writer = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
