"""Ego speed plausibility and reference error on hand-made rows; no ROS or bag required."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'ego_motion_report', Path(__file__).resolve().parents[1] / 'ego_motion_report.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

FRAME_NS = 100_000_000


def row(i, speed, valid=True):
    return {'bag_stamp_ns': 5_000_000_000 + i * FRAME_NS, 'measurement_stamp_ns': i * FRAME_NS,
            'ego_motion_valid': valid, 'ego_speed_mps': speed if valid else 0.0}


class EgoMotionReportTest(unittest.TestCase):

    def test_implausible_changes_and_drops_are_listed(self):
        speeds = [16.0, 16.2, 16.0, 6.0, 6.0, 16.0, 0.0]
        rows = [row(i, v) for i, v in enumerate(speeds)] + [row(7, 0.0, valid=False)]
        result = tool.plausibility(rows)
        # 0.2 m/s in 0.1 s is within 3 m/s^2 plus one 0.5 m/s step; 10 m/s is not.
        self.assertEqual([jump[0] for jump in result['jumps']], [0.3, 0.5, 0.6])
        self.assertEqual(result['drops_to_standstill'], [0.6])
        self.assertEqual(result['invalid_frames'], 1)

    def test_speed_error_against_an_object_fixed_in_the_world(self):
        # The object closes in at 15 m/s; the estimate is right, then 1.5 m/s too low.
        rows = [row(i, 15.0 if i < 10 else 13.5) for i in range(20)]
        annotation = {'events': [{'reference_frames': [
            {'bag_stamp_ns': r['bag_stamp_ns'], 'distance_m': 60.0 - 1.5 * i}
            for i, r in enumerate(rows)]}]}
        reference = tool.reference_speed(rows, annotation)
        # A window needs 6 of its 7 frames: frames 2 to 17.
        self.assertEqual(sorted(reference), list(range(2, 18)))
        self.assertTrue(all(abs(speed - 15.0) < 1e-6 for speed in reference.values()))
        error = tool.speed_error(rows, reference)
        self.assertEqual(error['reference_frames'], 16)
        self.assertEqual(error['over_1_mps'], 8)
        self.assertAlmostEqual(error['max_mps'], 1.5)

    def test_frames_too_close_or_without_a_valid_speed(self):
        rows = [row(i, 10.0, valid=i != 5) for i in range(12)]
        annotation = {'events': [{'reference_frames': [
            {'bag_stamp_ns': r['bag_stamp_ns'], 'distance_m': 12.0 - i}
            for i, r in enumerate(rows)]}]}
        reference = tool.reference_speed(rows, annotation)
        # Distances of 3 m and less are not used, so windows past frame 6 lack frames.
        self.assertEqual(sorted(reference), [2, 3, 4, 5, 6])
        error = tool.speed_error(rows, reference)
        self.assertEqual(error['invalid_frames'], 1)


if __name__ == '__main__':
    unittest.main()
