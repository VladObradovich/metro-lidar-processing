#!/usr/bin/env python3
"""
Report only the total number of alarms on the blind holdout of evaluation/splits.yaml.

The blind holdout is the part of a bag from from_bag_stamp_ns on. The data owner reports no
obstacle on the track there, so every OBSTACLE frame is a false alarm. To keep the holdout
blind the report has totals only: frames, OBSTACLE frames, alarm events (grouped as in
metrics.py), events per minute and the share of UNKNOWN frames; never times, distances or
positions of single alarms.

  holdout_alarms.py RUN_DIR [--splits evaluation/splits.yaml] [--output holdout.json]

RUN_DIR holds BAG/frames.jsonl (evaluate_all.py) or BAG.jsonl (evaluate_bag).
"""
import argparse
import importlib.util
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    'metrics', ROOT / 'metro_perception_tools/metro_perception_tools/metrics.py')
metrics = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(metrics)


def holdout_totals(rows, from_ns, gap_s=metrics.DEFAULT_EVENT_GAP_S):
    """Alarm totals over the rows at or after from_ns."""
    rows = sorted((row for row in rows if row['bag_stamp_ns'] >= from_ns),
                  key=lambda row: row['bag_stamp_ns'])
    events = metrics.alarm_events(rows, int(gap_s * 1e9))
    duration_s = (rows[-1]['bag_stamp_ns'] - rows[0]['bag_stamp_ns']) / 1e9 if rows else 0.0
    return {
        'frames': len(rows),
        'obstacle_frames': sum(row['state'] == 'OBSTACLE' for row in rows),
        'events': len(events),
        'duration_s': round(duration_s, 1),
        'events_per_min': round(60 * len(events) / duration_s, 3) if duration_s > 0 else None,
        'unknown_share': (round(sum(row['state'] == 'UNKNOWN' for row in rows) / len(rows), 4)
                          if rows else None),
    }


def report(run_dir, splits_path):
    """Totals for every blind holdout entry that the run has results for."""
    splits = yaml.safe_load(splits_path.read_text())
    result = {}
    for entry in splits.get('blind_holdout') or []:
        path = metrics.find_results(run_dir, entry['bag'])
        if path is None:
            result[entry['bag']] = None
            continue
        result[entry['bag']] = holdout_totals(metrics.read_rows(path),
                                              entry['from_bag_stamp_ns'])
    return result


def main():
    """Parse arguments and print the totals."""
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--splits', type=Path, default=ROOT / 'evaluation/splits.yaml')
    parser.add_argument('--output', type=Path, help='also write the totals as JSON (new file)')
    args = parser.parse_args()
    totals = report(args.run_dir, args.splits)
    for bag, value in totals.items():
        if value is None:
            print(f'{bag}: no results in {args.run_dir}')
            continue
        print(f"{bag}: {value['frames']} frames, {value['obstacle_frames']} OBSTACLE frames, "
              f"{value['events']} events in {value['duration_s']} s "
              f"({value['events_per_min']}/min), UNKNOWN share {value['unknown_share']}")
    if args.output:
        with args.output.open('x') as stream:
            json.dump(totals, stream, indent=2, allow_nan=False)
            stream.write('\n')


if __name__ == '__main__':
    main()
