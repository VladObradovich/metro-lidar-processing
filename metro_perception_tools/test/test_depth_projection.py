from metro_perception_tools.depth_image import (
    channel_count,
    colorize_depth,
    colorize_labelled_depth,
    point_cloud_labels,
    point_cloud_xyz_ring,
    project_depth_panorama,
    project_labels,
    rings_from_elevation,
    rings_from_elevation_indexed,
    video_frame_index,
)
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def xyz_intensity_cloud(points):
    """Build a PointCloud2 with x, y, z, intensity and no ring field."""
    data = np.asarray(points, dtype='<f4').reshape(-1, 3)
    data = np.column_stack([data, np.zeros(len(data), dtype='<f4')])
    message = PointCloud2()
    message.height = 1
    message.width = len(data)
    message.fields = [
        PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(('x', 'y', 'z', 'intensity'))
    ]
    message.point_step = 16
    message.row_step = 16 * len(data)
    message.data = data.tobytes()
    return message


def test_nearest_point_wins_same_pixel():
    xyz = np.array([[10.0, 0.0, 0.0], [5.0, 0.0, 0.0]], dtype=np.float32)
    rings = np.array([0, 0], dtype=np.uint16)
    depth = project_depth_panorama(
        xyz, rings, 360, 128, 1.0, 100.0, -50.0, 50.0
    )
    assert np.isclose(np.min(depth), 5.0)
    assert np.count_nonzero(np.isfinite(depth)) == 1


def test_realsense_jet_runs_from_blue_to_dark_red():
    depth = np.array([[1.0, 100.0, np.inf]], dtype=np.float32)
    rgb, populated = colorize_depth(
        depth, 1.0, 100.0, histogram_equalization=False
    )
    assert populated == 2
    assert np.array_equal(rgb[0, 0], [0, 0, 255])
    assert np.array_equal(rgb[0, 1], [50, 0, 0])
    assert np.array_equal(rgb[0, 2], [0, 0, 0])


def test_ring_numbers_define_image_rows():
    xyz = np.array([[10.0, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=np.float32)
    rings = np.array([0, 127], dtype=np.uint16)
    depth = project_depth_panorama(
        xyz, rings, 100, 128, 1.0, 100.0, -50.0, 50.0
    )
    assert np.isfinite(depth[0]).any()
    assert np.isfinite(depth[-1]).any()


def test_cloud_without_ring_returns_xyz_only():
    xyz, rings = point_cloud_xyz_ring(xyz_intensity_cloud([[1, 2, 3], [4, 5, 6]]))
    assert rings is None
    assert np.array_equal(xyz, [[1, 2, 3], [4, 5, 6]])


def rays(elevations_deg, repeat, distance=10.0):
    """Return points at the given elevations, each repeated along azimuth."""
    elevation = np.radians(np.repeat(elevations_deg, repeat))
    azimuth = np.radians(np.tile(np.linspace(-30, 30, repeat), len(elevations_deg)))
    horizontal = distance * np.cos(elevation)
    return np.column_stack(
        [horizontal * np.cos(azimuth), horizontal * np.sin(azimuth), distance * np.sin(elevation)]
    ).astype(np.float32)


def test_rings_from_elevation_number_channels_from_the_top():
    # Uneven spacing like the Pandar128: 0.9 deg at the edges, 0.125 deg in the middle.
    channels = [14.4, 13.5, 0.125, 0.0, -24.2, -25.1]
    xyz = rays(channels, 50)
    recovered, rings = rings_from_elevation(xyz, 1.0)
    assert np.array_equal(recovered, xyz)
    assert np.array_equal(np.unique(rings), np.arange(6))
    assert np.array_equal(rings[::50], [0, 1, 2, 3, 4, 5])


def test_uneven_channels_fill_every_image_row():
    xyz = rays([14.4, 13.5, 0.125, 0.0, -24.2, -25.1], 50)
    depth = project_depth_panorama(
        *rings_from_elevation(xyz, 1.0), 100, 6, 1.0, 100.0, -50.0, 50.0
    )
    assert np.isfinite(depth).any(axis=1).all()


def test_empty_points_and_points_outside_the_channels_keep_one_ring():
    xyz = np.vstack(
        [rays([1.0, 0.0], 50), rays([1.5, -0.5], 1), np.zeros((20, 3), dtype=np.float32)]
    )
    recovered, rings = rings_from_elevation(xyz, 1.0)
    assert len(recovered) == len(xyz)
    assert np.array_equal(rings[100:102], [0, 1])
    assert not rings[102:].any()


def test_point_between_channels_is_given_to_both():
    xyz = np.vstack([rays([1.0, 0.0], 50), rays([0.6], 3)])
    recovered, rings = rings_from_elevation(xyz, 1.0)
    assert rings.max() == 1
    assert np.array_equal(rings[100:103], [0, 0, 0])  # nearest: 1.0 deg
    assert np.array_equal(recovered[103:], xyz[100:103])
    assert np.array_equal(rings[103:], [1, 1, 1])


def test_inserted_surface_on_its_own_grid_leaves_no_empty_channel():
    # Channel 0.9 deg has no surface row nearest to it; the rows 0.96 and 0.835 bracket it.
    channels = [1.0, 0.9, 0.87, 0.75, 0.6]
    background = rays(channels, 50, distance=50.0)
    surface = rays([0.96, 0.835, 0.71], 3, distance=10.0)
    recovered, rings = rings_from_elevation(np.vstack([background, surface]), 1.0)
    depth = project_depth_panorama(
        recovered, rings, 100, len(channels), 1.0, 100.0, -50.0, 50.0
    )
    surface_rows = np.isclose(depth, 10.0, atol=0.1).any(axis=1)
    assert surface_rows[:4].all()


def test_video_frame_index_preserves_bag_timing():
    first = 1_000_000_000
    assert video_frame_index(first, first, 10.0) == 0
    assert video_frame_index(first + 100_000_000, first, 10.0) == 1
    assert video_frame_index(first + 3_400_000_000, first, 10.0) == 34


def test_empty_cloud_gives_an_empty_depth_image():
    xyz, rings = point_cloud_xyz_ring(xyz_intensity_cloud([]))
    assert xyz.shape == (0, 3) and rings is None
    xyz, rings = rings_from_elevation(xyz, 1.0)
    assert len(xyz) == 0 and len(rings) == 0
    assert channel_count(rings) == 0
    depth = project_depth_panorama(xyz, rings, 8, 4, 1.0, 100.0, -60.0, 60.0)
    assert depth.shape == (4, 8) and np.isinf(depth).all()
    ring_cloud = xyz_intensity_cloud([])
    ring_cloud.fields[3] = PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1)
    xyz, rings = point_cloud_xyz_ring(ring_cloud)
    assert xyz.shape == (0, 3) and rings.shape == (0,)


def test_channel_count_of_recovered_rings():
    _, rings = rings_from_elevation(rays([14.4, 13.5, 0.125, 0.0, -24.2, -25.1], 50), 1.0)
    assert channel_count(rings) == 6
    # Returns closer than the minimum range give no channels to report.
    _, rings = rings_from_elevation(rays([1.0, 0.0], 50, distance=0.5), 1.0)
    assert channel_count(rings) == 0


def labelled_cloud(points, labels):
    """Build the detector's labelled PointCloud2: x, y, z, rgb and a UINT8 label."""
    data = np.zeros(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                        ('rgb', '<f4'), ('label', 'u1'), ('pad', 'V3')])
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    data['x'], data['y'], data['z'] = points.T
    data['label'] = labels
    message = PointCloud2()
    message.height = 1
    message.width = len(data)
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='label', offset=16, datatype=PointField.UINT8, count=1),
    ]
    message.point_step = 20
    message.row_step = 20 * len(data)
    message.data = data.tobytes()
    return message


def test_labels_are_read_from_the_cloud():
    message = labelled_cloud([[1, 0, 0], [2, 0, 0], [3, 0, 0]], [0, 1, 2])
    assert point_cloud_labels(message).tolist() == [0, 1, 2]
    assert point_cloud_labels(xyz_intensity_cloud([[1, 0, 0]])) is None


def test_copies_between_channels_keep_their_source_index():
    xyz = rays([1.0, 0.0], 50)
    between = np.array([[10.0, 0.0, 10.0 * np.tan(np.radians(0.5))]])
    points, rings, source = rings_from_elevation_indexed(np.vstack([xyz, between]), 1.0)
    assert len(points) == len(source) == len(rings) == len(xyz) + 2
    assert source[-1] == len(xyz)  # The copy points back to the in-between return.


def test_nearest_point_gives_the_pixel_label():
    xyz = np.array([[10.0, 0.0, 0.0], [5.0, 0.0, 0.0]], dtype=np.float32)
    rings = np.array([0, 0], dtype=np.uint16)
    labels = np.array([1, 2], dtype=np.uint8)
    image = project_labels(xyz, rings, labels, 360, 128, 1.0, 100.0, -50.0, 50.0)
    assert np.count_nonzero(image) == 1
    assert image.max() == 2


def test_labelled_depth_is_grey_with_green_corridor_and_red_obstacle():
    depth = np.array([[5.0, 10.0, 20.0, np.inf]], dtype=np.float32)
    labels = np.array([[0, 1, 2, 0]], dtype=np.uint8)
    rgb, populated = colorize_labelled_depth(depth, labels, 1.0, 100.0, False)
    assert populated == 3
    grey, green, red, empty = rgb[0]
    assert grey[0] == grey[1] == grey[2]
    assert green[1] > green[0] and green[1] > green[2]
    assert tuple(red) == (255, 40, 40)
    assert tuple(empty) == (0, 0, 0)
