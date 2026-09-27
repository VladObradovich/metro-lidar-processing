"""Override handling and summaries of sweep_profile.py; no ROS or bag required."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'sweep_profile', Path(__file__).resolve().parents[1] / 'sweep_profile.py'
)
sweep = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sweep)


class SweepProfileTests(unittest.TestCase):
    def test_overrides_go_to_detector_or_named_section_without_touching_the_source(self):
        profile = {'detector': {'corridor_half_width_m': 2.0}, 'temporal': {'confirm_hits': 2}}
        result = sweep.apply_overrides(profile, [
            'static_max_length_m=4', 'temporal.confirm_hits=3', 'detector.static_channel=false'])
        self.assertEqual(result['detector'], {'corridor_half_width_m': 2.0,
                                              'static_max_length_m': 4,
                                              'static_channel': False})
        self.assertEqual(result['temporal'], {'confirm_hits': 3})
        self.assertEqual(profile['temporal'], {'confirm_hits': 2})
        with self.assertRaises(ValueError):
            sweep.apply_overrides(profile, ['static_max_length_m'])

    def test_jobs_skip_unannotated_bags_and_use_the_profile_of_the_sensor(self):
        dataset = {'bags': [
            {'id': 'a', 'path': 'a', 'input_topic': '/p', 'annotations': 'x.yaml',
             'sensor_profile': 'full_scan'},
            {'id': 'b', 'path': 'b', 'input_topic': '/p', 'sensor_profile': 'forward_sector'}]}
        jobs = sweep.jobs_for(dataset, '/data', Path('/out'), {'full_scan': 'full.yaml'})
        self.assertEqual(jobs, [(Path('/data/a'), '/p', Path('/out/a/frames.jsonl'),
                                 'full.yaml')])

    def test_synthetic_summary_adds_bins_over_bags_of_one_scenario(self):
        report = {
            'bag1-static': {'first_hit_distance_m': 61.2,
                            'recall_by_distance': {'0-20': [1.0, 4], '20-40': [0.5, 2],
                                                   '120-150': [None, 0]}},
            'bag2-static': {'first_hit_distance_m': None,
                            'recall_by_distance': {'0-20': [0.5, 2], '20-40': [None, 0]}}}
        self.assertEqual(sweep.synthetic_summary('dev', report),
                         ['  [dev] static   first=[61, None] 0-20:5/6 20-40:1/2'])


if __name__ == '__main__':
    unittest.main()
