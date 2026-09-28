#!/usr/bin/env python3
"""
Report how plausible the lidar-only ego speed of an evaluation run is, bag by bag.

Every bag: speed changes beyond what a train can do (MAX_ACCELERATION_MPS2 per second plus
one 5 cm profile step per frame), frames without a valid speed, and drops from moving to
standing still within one frame. Bags annotated from inserted objects (review_method
inserted_returns_trailing_block) also get the error against the approach speed of those
objects, which are fixed in the world: a Theil-Sen slope of their reference distances over
+-REFERENCE_HALF_WINDOW frames of header time.

  ego_motion_report.py RUN_DIR [--dataset evaluation/dataset.yaml] [--output report.json]

RUN_DIR holds BAG/frames.jsonl (evaluate_all.py) or BAG.jsonl (evaluate_bag).
"""
import argparse
import json
from pathlib import Path
import statistics

import yaml

ROOT = Path(__file__).resolve().parents[1]
MAX_ACCELERATION_MPS2 = 3.0
QUANTISATION_MPS = 0.5
MOVING_MPS, STANDING_MPS = 5.0, 0.5
REFERENCE_HALF_WINDOW = 3
REFERENCE_MIN_DISTANCE_M = 3.0
INSERTED = 'inserted_returns_trailing_block'


def plausibility(rows):
    """Implausible speed changes, frames without a valid speed and drops to standstill."""
    t0 = rows[0]['bag_stamp_ns'] if rows else 0
    jumps, drops, invalid = [], [], 0
    previous = None
    for row in rows:
        if not row.get('ego_motion_valid'):
            invalid += 1
            continue
        speed = row['ego_speed_mps']
        if previous is not None:
            dt = max(1e-3, (row['measurement_stamp_ns'] - previous[0]) * 1e-9)
            at = round((row['bag_stamp_ns'] - t0) / 1e9, 2)
            if abs(speed - previous[1]) > MAX_ACCELERATION_MPS2 * dt + QUANTISATION_MPS:
                jumps.append([at, round(previous[1], 2), round(speed, 2)])
            if previous[1] > MOVING_MPS and speed < STANDING_MPS:
                drops.append(at)
        previous = (row['measurement_stamp_ns'], speed)
    return {'frames': len(rows), 'invalid_frames': invalid, 'jumps': jumps,
            'drops_to_standstill': drops}


def reference_speed(rows, annotation):
    """Approach speed of world-fixed inserted objects per row index."""
    index = {row['bag_stamp_ns']: i for i, row in enumerate(rows)}
    times = [row['measurement_stamp_ns'] * 1e-9 for row in rows]
    per_row = {}
    for event in annotation.get('events') or []:
        distance = {index[ref['bag_stamp_ns']]: ref['distance_m']
                    for ref in event.get('reference_frames') or []
                    if ref['bag_stamp_ns'] in index and ref.get('distance_m') is not None and
                    ref['distance_m'] > REFERENCE_MIN_DISTANCE_M}
        for i in distance:
            points = [(times[j], distance[j])
                      for j in range(i - REFERENCE_HALF_WINDOW, i + REFERENCE_HALF_WINDOW + 1)
                      if j in distance]
            if len(points) < 2 * REFERENCE_HALF_WINDOW:
                continue
            slopes = [(x2 - x1) / (t2 - t1) for k, (t1, x1) in enumerate(points)
                      for t2, x2 in points[k + 1:] if t2 > t1]
            per_row.setdefault(i, []).append(-statistics.median(slopes))
    return {i: statistics.median(values) for i, values in per_row.items()}


def speed_error(rows, reference):
    """Absolute speed error against the reference; a frame without a valid speed counts apart."""
    errors = sorted(abs(rows[i]['ego_speed_mps'] - speed) for i, speed in reference.items()
                    if rows[i].get('ego_motion_valid'))
    invalid = len(reference) - len(errors)
    return {'reference_frames': len(reference), 'invalid_frames': invalid,
            'median_mps': statistics.median(errors) if errors else None,
            'p95_mps': errors[int(0.95 * (len(errors) - 1))] if errors else None,
            'max_mps': errors[-1] if errors else None,
            'over_1_mps': sum(error > 1.0 for error in errors)}


def results_of(run_dir, bag_id):
    for path in (run_dir / bag_id / 'frames.jsonl', run_dir / f'{bag_id}.jsonl'):
        if path.is_file():
            return path
    return None


def report(run_dir, dataset_path, root=ROOT):
    bags = {}
    for entry in yaml.safe_load(dataset_path.read_text())['bags']:
        path = results_of(run_dir, entry['id'])
        if path is None:
            continue
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        if not rows:
            continue
        rows.sort(key=lambda row: row['bag_stamp_ns'])
        result = plausibility(rows)
        annotation = (yaml.safe_load((root / entry['annotations']).read_text())
                      if entry.get('annotations') else {})
        if annotation.get('review_method') == INSERTED:
            result['reference'] = speed_error(rows, reference_speed(rows, annotation))
        bags[entry['id']] = result
    return bags


def format_report(bags):
    def fmt(value):
        return 'N/A' if value is None else f'{value:.2f}'
    lines = []
    for bag, result in bags.items():
        line = (f"{bag:36s} frames {result['frames']:6d} invalid {result['invalid_frames']:5d} "
                f"jumps {len(result['jumps']):3d} drops {len(result['drops_to_standstill']):2d}")
        reference = result.get('reference')
        if reference:
            line += (f" | reference {reference['reference_frames']} frames: median "
                     f"{fmt(reference['median_mps'])} p95 {fmt(reference['p95_mps'])} "
                     f"max {fmt(reference['max_mps'])} m/s, over 1 m/s {reference['over_1_mps']}")
        lines.append(line)
    return '\n'.join(lines)


def main():
    """Parse arguments and print the report."""
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'evaluation/dataset.yaml')
    parser.add_argument('--output', type=Path, help='also write the report as JSON (new file)')
    args = parser.parse_args()
    bags = report(args.run_dir, args.dataset)
    print(format_report(bags))
    if args.output:
        with args.output.open('x') as stream:
            json.dump(bags, stream, indent=2, allow_nan=False)
            stream.write('\n')


if __name__ == '__main__':
    main()
