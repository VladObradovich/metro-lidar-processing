"""The repository evaluation files parse and agree with each other; no ROS or bag required."""

from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
EVALUATION = ROOT / 'evaluation'
SPLITS = ('development', 'validation', 'holdout', 'regression')


def load(path):
    return yaml.safe_load(path.read_text())


class EvaluationDataTest(unittest.TestCase):

    def test_every_evaluation_yaml_parses(self):
        # The scorer reads these files; one bad scalar stops every evaluation run.
        files = sorted(EVALUATION.rglob('*.yaml'))
        self.assertIn(EVALUATION / 'splits.yaml', files)
        for path in files:
            with self.subTest(path=str(path.relative_to(ROOT))):
                load(path)

    def test_splits_scene_context_and_annotations_match_the_dataset(self):
        dataset = load(EVALUATION / 'dataset.yaml')
        registered = {bag['id'] for bag in dataset['bags']}
        splits = load(EVALUATION / 'splits.yaml')
        assigned = [bag for name in SPLITS for bag in splits.get(name) or []]
        self.assertEqual(len(assigned), len(set(assigned)), 'a bag is in two splits')
        self.assertLessEqual(set(assigned), registered)
        self.assertLessEqual(set(load(EVALUATION / 'scene_context.yaml')['bags']), registered)
        for bag in dataset['bags']:
            if not bag.get('annotations'):
                continue
            with self.subTest(bag=bag['id']):
                annotation = load(ROOT / bag['annotations'])
                self.assertEqual(annotation['bag_id'], bag['id'])
                self.assertTrue(annotation['reviewed'])
                self.assertEqual(annotation['time_basis'], 'bag_stamp_ns')
                self.assertIn(bag['id'], assigned, 'an annotated bag is in no split')

    def test_blind_holdout_carries_no_label(self):
        # Labels or events reaching into the blind holdout would make its frames scored.
        dataset = {bag['id']: bag for bag in load(EVALUATION / 'dataset.yaml')['bags']}
        holdout = load(EVALUATION / 'splits.yaml').get('blind_holdout') or []
        for entry in holdout:
            with self.subTest(bag=entry['bag']):
                self.assertIn(entry['bag'], dataset)
                start = entry['from_bag_stamp_ns']
                self.assertIsInstance(start, int)
                path = dataset[entry['bag']].get('annotations')
                if not path:
                    continue
                annotation = load(ROOT / path)
                for interval in annotation.get('reviewed_intervals') or []:
                    self.assertLessEqual(interval['end_ns'], start)
                for event in annotation.get('events') or []:
                    self.assertLessEqual(event['end_ns'], start)


if __name__ == '__main__':
    unittest.main()
