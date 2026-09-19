"""Convert a PointCloud2 stream into a colorized panoramic depth image."""

from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField


def point_cloud_xyz_ring(
    message: PointCloud2, stride: int = 1
) -> Tuple[np.ndarray, np.ndarray]:
    """Return compact XYZ coordinates and lidar ring numbers."""
    fields = {field.name: field for field in message.fields}
    missing = {'x', 'y', 'z', 'ring'} - fields.keys()
    if missing:
        raise ValueError(f"PointCloud2 has no fields: {', '.join(sorted(missing))}")

    xyz_fields = [fields[name] for name in ('x', 'y', 'z')]
    if any(field.datatype != PointField.FLOAT32 for field in xyz_fields):
        raise ValueError('x, y and z fields must use FLOAT32')
    if fields['ring'].datatype != PointField.UINT16:
        raise ValueError('ring field must use UINT16')

    byte_order = '>' if message.is_bigendian else '<'
    scalar_type = np.dtype(f'{byte_order}f4')
    count = message.width * message.height
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
    rings = np.ndarray(
        shape=(count,),
        dtype=np.dtype(f'{byte_order}u2'),
        buffer=message.data,
        offset=fields['ring'].offset,
        strides=(message.point_step,),
    )[::step]
    return np.column_stack(coordinates), rings


def distance_cloud(message: PointCloud2) -> PointCloud2:
    """Build a full-resolution XYZ/distance cloud for RViz, in metres."""
    fields = {field.name: field for field in message.fields}
    points = np.empty((message.height, message.width, 4), dtype='<f4')
    for axis, name in enumerate(('x', 'y', 'z')):
        field = fields.get(name)
        if field is None or field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError('Distance visualization requires scalar FLOAT32 XYZ')
        if message.width * message.height:
            points[..., axis] = np.ndarray(
                shape=(message.height, message.width),
                dtype='>f4' if message.is_bigendian else '<f4',
                buffer=message.data,
                offset=field.offset,
                strides=(message.row_step, message.point_step),
            )
    points[..., 3] = np.linalg.norm(points[..., :3], axis=-1)
    return PointCloud2(
        header=message.header, height=message.height, width=message.width,
        fields=[PointField(name=name, offset=4 * index,
                           datatype=PointField.FLOAT32, count=1)
                for index, name in enumerate(('x', 'y', 'z', 'distance'))],
        is_bigendian=False, point_step=16, row_step=16 * message.width,
        is_dense=bool(np.isfinite(points).all()), data=points.tobytes(),
    )


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
    depth = np.full(width * height, np.inf, dtype=np.float32)
    if xyz.size == 0:
        return depth.reshape(height, width)

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
    if not np.any(valid):
        return depth.reshape(height, width)

    ranges = ranges[valid].astype(np.float32, copy=False)
    azimuth = azimuth[valid]
    valid_rings = rings[valid].astype(np.float32, copy=False)

    columns = (
        (azimuth - min_azimuth_deg)
        * (width - 1)
        / (max_azimuth_deg - min_azimuth_deg)
    ).astype(np.int32)
    if ring_max == ring_min:
        rows = np.zeros(valid_rings.shape, dtype=np.int32)
    else:
        rows = (
            (valid_rings - ring_min) * (height - 1) / (ring_max - ring_min)
        ).astype(np.int32)
    columns = np.clip(columns, 0, width - 1)
    rows = np.clip(rows, 0, height - 1)

    np.minimum.at(depth, rows * width + columns, ranges)
    return depth.reshape(height, width)


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
        self.declare_parameter('distance_topic', '/lidar/distance_points')
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

        self._validate_parameters()
        input_qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, self.output_topic, image_qos)
        self.distance_publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('distance_topic').value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
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
            # Avoid copying the full cloud when RViz is not subscribed.
            if self.distance_publisher.get_subscription_count():
                self.distance_publisher.publish(distance_cloud(message))
            xyz, rings = point_cloud_xyz_ring(message, self.point_stride)
            depth = project_depth_panorama(
                xyz,
                rings,
                self.image_width,
                self.image_height,
                self.min_depth,
                self.max_depth,
                self.min_azimuth,
                self.max_azimuth,
            )
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
        image.data = rgb.tobytes()
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
