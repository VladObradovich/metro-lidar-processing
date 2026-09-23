"""Geometry tests for inject_obstacle.py; no ROS or bag required."""

import importlib.util
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    'inject_obstacle', Path(__file__).resolve().parents[1] / 'inject_obstacle.py'
)
inject = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inject)


class InjectObstacleTests(unittest.TestCase):
    def test_frame_conversion_round_trips_and_matches_the_profile(self):
        sensor = np.array([[1.0, -10.0, 0.5], [3.0, 2.0, -1.0]])
        target = inject.to_target(sensor)
        np.testing.assert_allclose(target[0], [10.0, 1.0, 0.5])  # forward = -sensor y
        np.testing.assert_allclose(inject.to_sensor(target), sensor)

    def test_only_returns_behind_the_box_are_moved_onto_it(self):
        lo, hi = np.array([10.0, -0.5, -1.0]), np.array([11.0, 0.5, 1.0])
        points = np.array([
            [30.0, 0.0, 0.0],   # behind the box: moved to its near face
            [5.0, 0.0, 0.0],    # in front of the box: unchanged
            [30.0, 10.0, 0.0],  # ray misses the box: unchanged
            [0.0, 0.0, 0.0],    # no return: unchanged
        ])
        moved, hidden = inject.occlude(points, lo, hi)
        self.assertEqual(hidden, 1)
        np.testing.assert_allclose(moved[0], [10.0, 0.0, 0.0])
        np.testing.assert_allclose(moved[1:], points[1:])

    def test_axis_parallel_rays_are_handled(self):
        directions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        entry = inject.ray_box_entry(directions, [10, -1, -1], [12, 1, 1])
        self.assertAlmostEqual(entry[0], 10.0)
        self.assertTrue(np.isinf(entry[1]))

    def test_static_obstacle_approaches_by_the_travelled_distance(self):
        rows = [{'bag_stamp_ns': i * 100_000_000, 'ego_speed_mps': 10.0,
                 'ground_plane': [0.0, 0.0, 1.0, 1.9]} for i in range(30)]
        plan, travel = inject.plan_obstacle(rows, 'static', 2.0, 8.0)
        stamps = sorted(plan)
        self.assertAlmostEqual(travel, 20.0)
        near = [plan[stamp][0][0] for stamp in stamps]
        np.testing.assert_allclose(np.diff(near), -1.0)  # 10 m/s at 10 Hz
        self.assertAlmostEqual(near[-1], 8.0 - 0.25)
        self.assertAlmostEqual(plan[stamps[0]][0][2], -1.9)  # standing on the floor

    def test_crossing_stays_inside_the_route_at_walking_speed(self):
        lateral = [inject.box_at('crossing', 50.0, t / 10, -1.9)[0][1] + 0.25
                   for t in range(120)]
        self.assertLessEqual(max(np.abs(lateral)), inject.CROSSING_HALF_WIDTH_M + 1e-9)
        self.assertAlmostEqual(max(np.abs(np.diff(lateral))) * 10, 1.0)


if __name__ == '__main__':
    unittest.main()
