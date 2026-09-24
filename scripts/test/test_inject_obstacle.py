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
        centres = [plan[stamp][0] for stamp in stamps]
        np.testing.assert_allclose(np.diff(centres), -1.0)  # 10 m/s at 10 Hz
        self.assertAlmostEqual(centres[-1], 8.0)
        lo, hi, route = inject.place('static', *plan[stamps[0]], np.zeros((1, 3)))
        self.assertIsNone(route)  # No walls: straight route.
        self.assertAlmostEqual(lo[2], -1.9)  # Standing on the floor.

    def test_floor_is_the_lowest_surface_not_the_rails(self):
        floor = [[x, y, -1.5 + 0.01 * x] for x in np.arange(3, 60, 0.2)
                 for y in np.arange(-1.1, 1.1, 0.1)]
        rails = [[x, y, -1.15 + 0.01 * x] for x in np.arange(3, 60, 0.05) for y in (-0.76, 0.76)]
        plane = inject.estimate_floor(np.array(floor + rails))
        self.assertAlmostEqual(inject.floor_height(plane, 30.0, 0.0), -1.2, delta=0.05)

    def test_obstacle_is_placed_on_a_curved_route(self):
        radius = 300.0
        xs = np.arange(1.0, 90.0, 0.25)
        heights = np.arange(-0.9, 1.0, 0.3)  # 1.0-2.8 m above the floor at z = -1.9.
        walls = [[x, x * x / (2 * radius) + side * 2.3, z]
                 for x in xs for side in (-1.0, 1.0) for z in heights]
        plane = np.array([0.0, 0.0, 1.0, 1.9])
        c1, c2 = inject.estimate_route(np.array(walls), plane)
        self.assertAlmostEqual(c2, 1 / (2 * radius), delta=0.1 / (2 * radius))
        self.assertAlmostEqual(c1, 0.0, delta=0.01)
        lo, hi, _ = inject.place('static', 60.0, 0.0, plane, np.array(walls))
        self.assertAlmostEqual((lo[1] + hi[1]) / 2, 60.0 ** 2 / (2 * radius), delta=0.3)

    def test_crossing_stays_inside_the_route_at_walking_speed(self):
        lateral = [inject.box_at('crossing', 50.0, t / 10, -1.9)[0][1] + 0.25
                   for t in range(120)]
        self.assertLessEqual(max(np.abs(lateral)), inject.CROSSING_HALF_WIDTH_M + 1e-9)
        self.assertAlmostEqual(max(np.abs(np.diff(lateral))) * 10, 1.0)


if __name__ == '__main__':
    unittest.main()
