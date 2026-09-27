"""Blind holdout totals on hand-made rows; no ROS or bag required."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import yaml

spec = importlib.util.spec_from_file_location(
    'holdout_alarms', Path(__file__).resolve().parents[1] / 'holdout_alarms.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

FRAME_NS = 100_000_000
START_NS = 1_000_000_000_000


def row(i, state):
    return {'bag_stamp_ns': START_NS + i * FRAME_NS, 'state': state}


class HoldoutAlarmsTest(unittest.TestCase):

    def test_only_frames_from_the_holdout_start_count(self):
        # Frames 0-9 are development, 10-609 (60 s) the holdout.
        states = ['OBSTACLE'] * 10 + ['UNKNOWN'] * 600
        states[20:23] = ['OBSTACLE'] * 3   # one event
        states[23] = 'UNKNOWN'
        states[25] = 'OBSTACLE'            # 0.2 s later: the same event
        states[100] = 'OBSTACLE'           # a second event
        rows = [row(i, state) for i, state in enumerate(states)]
        totals = tool.holdout_totals(rows, START_NS + 10 * FRAME_NS)
        self.assertEqual(totals['frames'], 600)
        self.assertEqual(totals['obstacle_frames'], 5)
        self.assertEqual(totals['events'], 2)
        self.assertEqual(totals['duration_s'], 59.9)
        self.assertAlmostEqual(totals['events_per_min'], 2.003, places=3)
        self.assertAlmostEqual(totals['unknown_share'], 595 / 600, places=4)

    def test_report_has_totals_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            splits = tmp / 'splits.yaml'
            splits.write_text(yaml.safe_dump({'blind_holdout': [
                {'bag': 'long', 'from_bag_stamp_ns': START_NS + 5 * FRAME_NS},
                {'bag': 'absent', 'from_bag_stamp_ns': START_NS}]}))
            (tmp / 'run' / 'long').mkdir(parents=True)
            rows = [row(i, 'OBSTACLE' if i == 7 else 'UNKNOWN') for i in range(20)]
            (tmp / 'run' / 'long' / 'frames.jsonl').write_text(
                ''.join(json.dumps(r) + '\n' for r in rows))
            result = tool.report(tmp / 'run', splits)
        self.assertIsNone(result['absent'])
        self.assertEqual(set(result['long']), {'frames', 'obstacle_frames', 'events', 'duration_s',
                                               'events_per_min', 'unknown_share'})
        self.assertEqual(result['long']['obstacle_frames'], 1)


if __name__ == '__main__':
    unittest.main()
