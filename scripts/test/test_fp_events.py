"""False-alarm event catalogue on hand-made rows; no ROS or bag required."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'fp_events', Path(__file__).resolve().parents[1] / 'fp_events.py'
)
fp_events = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fp_events)

SECOND = 1_000_000_000


def row(t, state='OBSTACLE', tracks=(), reason='OBSTACLE_CANDIDATE', route=(0.0, 0.0, True, 80)):
    return {'bag_stamp_ns': int(t * SECOND), 'state': state, 'reason': reason,
            'route': list(route), 'ego_motion_valid': True, 'ego_speed_mps': 15.0,
            'candidates': [], 'tracks': list(tracks)}


def track(x, y, length=0.5, channels=1, confirmed=True):
    return {'confirmed': confirmed, 'center': [x, y, 0.0], 'size': [length, 0.5, 1.5],
            'distance_m': x - length / 2, 'channels': channels}


ANNOTATION = {'reviewed_intervals': [
    {'start_ns': 0, 'end_ns': 10 * SECOND, 'label': 'negative'},
    {'start_ns': 10 * SECOND + 1, 'end_ns': 20 * SECOND, 'label': 'positive'}]}


class FpEventTests(unittest.TestCase):
    def test_classes_follow_offset_length_and_channels(self):
        route = (0.0, 0.0, True, 80)
        self.assertEqual(fp_events.object_class(track(20, 0.3, channels=2), route), 'gauge-G')
        self.assertEqual(fp_events.object_class(track(20, -1.2, channels=3), route), 'mid-M-G')
        self.assertEqual(fp_events.object_class(track(20, 2.0, length=6.0), route),
                         'edge-long-M')
        # Offsets are taken from the curved route, not from y = 0.
        curved = (0.0, 0.002, True, 80)
        self.assertEqual(fp_events.object_class(track(20, 0.8 + 0.1, channels=2), curved),
                         'gauge-G')

    def test_events_cover_only_negative_alarms_and_group_by_gap(self):
        rows = [row(0.0), row(0.1, tracks=[track(10, 2.0)]),
                row(0.2, tracks=[track(8.5, 2.0)], reason='OBSTACLE_COASTING'),
                row(0.3, state='UNKNOWN'), row(2.0, tracks=[track(30, 0.1, channels=2)]),
                row(12.0, tracks=[track(5, 0.0)])]
        events = fp_events.fp_events(rows, ANNOTATION)
        self.assertEqual([e['frames'] for e in events], [3, 1])
        self.assertEqual(events[0]['coasting_frames'], 1)
        self.assertEqual(events[0]['classes'], {'edge-M': 2})
        self.assertEqual(events[0]['distance_m'], [8.2, 9.8])
        self.assertEqual(events[1]['classes'], {'gauge-G': 1})
        self.assertEqual(events[1]['start_s'], 2.0)

    def test_unconfirmed_tracks_do_not_describe_an_event(self):
        rows = [row(1.0, tracks=[track(10, 0.0, confirmed=False), track(20, 2.0)])]
        self.assertEqual(fp_events.fp_events(rows, ANNOTATION)[0]['classes'], {'edge-M': 1})


if __name__ == '__main__':
    unittest.main()
