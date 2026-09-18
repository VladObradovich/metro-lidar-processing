from metro_perception_tools.depth_image import (
    colorize_depth,
    project_depth_panorama,
    video_frame_index,
)
import numpy as np


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


def test_video_frame_index_preserves_bag_timing():
    first = 1_000_000_000
    assert video_frame_index(first, first, 10.0) == 0
    assert video_frame_index(first + 100_000_000, first, 10.0) == 1
    assert video_frame_index(first + 3_400_000_000, first, 10.0) == 34
