"""Bag selection tests for evaluate_all.py; no ROS required."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'evaluate_all', Path(__file__).resolve().parents[1] / 'evaluate_all.py'
)
evaluate_all = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluate_all)


class SelectedBagsTests(unittest.TestCase):
    def test_unannotated_bags_are_exported_and_unknown_ids_rejected(self):
        dataset = {'bags': [{'id': 'labelled', 'annotations': 'a.yaml'},
                            {'id': 'new_data', 'annotations': None}]}
        self.assertEqual(evaluate_all.selected_bags(dataset, None), ['labelled', 'new_data'])
        self.assertEqual(evaluate_all.selected_bags(dataset, ['new_data']), ['new_data'])
        with self.assertRaises(ValueError):
            evaluate_all.selected_bags(dataset, ['missing'])


if __name__ == '__main__':
    unittest.main()
