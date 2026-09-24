#!/usr/bin/env python3
"""
List the false-alarm events of an evaluation run and what kind of object caused them.

An event is a series of OBSTACLE frames on the negative interval of a bag, grouped with the
same gap as metrics.py. Each object behind the decision (confirmed tracks, or candidates in
runs before G4) gets a descriptive class:

  zone     gauge (|offset from the route| <= 0.9 m), mid (<= 1.5 m) or edge (beyond)
  -long    longer than 3 m along the route
  -M / -G  evidence channels MOTION / GAUGE

The classes describe the data for review; they are not detector parameters.

  fp_events.py RUN_DIR [--dataset evaluation/dataset.yaml] [--output events.json]
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    'metrics', ROOT / 'metro_perception_tools/metro_perception_tools/metrics.py')
metrics = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(metrics)

GAUGE_HALF_WIDTH_M = 0.9
EDGE_OFFSET_M = 1.5
LONG_M = 3.0
MOTION, GAUGE = 1, 2


def route_offset(obj, route):
    """Lateral offset of an object centre from the route y = c1 x + c2 x^2 (or y = 0)."""
    x, y = obj['center'][0], obj['center'][1]
    if route and route[2]:
        return abs(y - (route[0] * x + route[1] * x * x))
    return abs(y)


def object_class(obj, route):
    """Descriptive class of one object, e.g. 'edge-long-M'."""
    offset = route_offset(obj, route)
    name = ('gauge' if offset <= GAUGE_HALF_WIDTH_M
            else 'mid' if offset <= EDGE_OFFSET_M else 'edge')
    if obj['size'][0] > LONG_M:
        name += '-long'
    channels = obj.get('channels', 0)
    if channels & MOTION:
        name += '-M'
    if channels & GAUGE:
        name += '-G'
    return name


def fp_events(rows, annotation, gap_s=metrics.DEFAULT_EVENT_GAP_S):
    """False-alarm events of one bag with frame counts, distances, ego speed and classes."""
    rows = sorted(rows, key=lambda row: row['bag_stamp_ns'])
    if not rows:
        return []
    intervals = annotation.get('reviewed_intervals') or []
    negative = [row for row in rows
                if metrics.frame_label(row['bag_stamp_ns'], intervals) == 'negative']
    t0 = rows[0]['bag_stamp_ns']
    groups = []
    for row in negative:
        if row['state'] != 'OBSTACLE':
            continue
        if groups and row['bag_stamp_ns'] - groups[-1][-1]['bag_stamp_ns'] <= gap_s * 1e9:
            groups[-1].append(row)
        else:
            groups.append([row])
    events = []
    for group in groups:
        classes = Counter()
        distances = []
        for row in group:
            for obj in metrics.decisive_objects(row):
                classes[object_class(obj, row.get('route'))] += 1
                if obj.get('distance_m') is not None:
                    distances.append(obj['distance_m'])
        speeds = [row['ego_speed_mps'] for row in group if row.get('ego_motion_valid')]
        events.append({
            'start_s': round((group[0]['bag_stamp_ns'] - t0) / 1e9, 2),
            'end_s': round((group[-1]['bag_stamp_ns'] - t0) / 1e9, 2),
            'frames': len(group),
            'coasting_frames': sum(row['reason'].startswith('OBSTACLE_COASTING')
                                   for row in group),
            'distance_m': ([round(min(distances), 1), round(max(distances), 1)]
                           if distances else None),
            'ego_speed_mps': round(sorted(speeds)[len(speeds) // 2], 1) if speeds else None,
            'classes': dict(classes.most_common()),
        })
    return events


def run_events(run_dir, dataset_path, splits_path, root=ROOT):
    """Events of every annotated bag of a run, keyed by bag id, with its split."""
    dataset = yaml.safe_load(dataset_path.read_text())
    splits = yaml.safe_load(splits_path.read_text()) if splits_path.is_file() else {}
    split_of = {bag: name for name in ('development', 'validation', 'holdout', 'regression')
                for bag in splits.get(name) or []}
    result = {}
    for entry in dataset['bags']:
        results = metrics.find_results(run_dir, entry['id'])
        if not entry.get('annotations') or results is None:
            continue
        annotation_path = Path(entry['annotations'])
        if not annotation_path.is_absolute():
            annotation_path = root / annotation_path
        result[entry['id']] = {
            'split': split_of.get(entry['id'], 'unassigned'),
            'events': fp_events(metrics.read_rows(results),
                                yaml.safe_load(annotation_path.read_text())),
        }
    return result


def format_events(result):
    """Human-readable listing, one line per event."""
    lines = []
    for bag, info in result.items():
        frames = sum(event['frames'] for event in info['events'])
        lines.append(f"{bag} [{info['split']}]: {len(info['events'])} events, {frames} frames")
        for event in info['events']:
            distance = ('..'.join(f'{d:.1f}' for d in event['distance_m'])
                        if event['distance_m'] else '-')
            lines.append(
                f"  {event['start_s']:6.1f}-{event['end_s']:6.1f} s  {event['frames']:3d} frames"
                f" ({event['coasting_frames']} coasting)  {distance} m"
                f"  ego {event['ego_speed_mps']}  {event['classes']}")
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'evaluation/dataset.yaml')
    parser.add_argument('--splits', type=Path, default=ROOT / 'evaluation/splits.yaml')
    parser.add_argument('--output', type=Path, help='also write the events as JSON')
    args = parser.parse_args()
    result = run_events(args.run_dir, args.dataset, args.splits)
    print(format_events(result))
    if args.output:
        with args.output.open('x') as output:
            json.dump(result, output, indent=2)
            output.write('\n')


if __name__ == '__main__':
    main()
