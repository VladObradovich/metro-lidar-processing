#!/usr/bin/env python3
"""
List the false-alarm events of an evaluation run and what kind of object caused them.

An event is a series of OBSTACLE frames on the negative interval of a bag, grouped with the
same gap as metrics.py. Each object behind the decision (confirmed tracks, or candidates in
runs before G4) gets a descriptive class:

  zone     gauge (|offset from the route| <= 0.9 m), mid (<= 1.5 m) or edge (beyond)
  -long    longer than 3 m along the route
  -M / -G  evidence channels MOTION / GAUGE

The classes describe the data for review; they are not detector parameters. Each event also
carries what points at its cause: frames without a valid ego speed or with a held
(unconfirmed) one, the time since the last frame without a valid speed, the number of
confirmed tracks behind it, the largest distance one of them moves along the track in the world
(x plus the lidar odometry integrated from the reported speeds; about zero for a fixed object),
their signed offset from the route and their height range above the reported ground plane
(sensor z when there is none).

  fp_events.py RUN_DIR [--dataset evaluation/dataset.yaml] [--output events.json]
"""
import argparse
from collections import Counter, defaultdict
import importlib.util
import json
import math
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


def route_offset(obj, route, signed=False):
    """Lateral offset of an object centre from the route y = c1 x + c2 x^2 (or y = 0)."""
    x, y = obj['center'][0], obj['center'][1]
    offset = y - (route[0] * x + route[1] * x * x) if route and route[2] else y
    return offset if signed else abs(offset)


def height_above_ground(x, y, z, plane):
    """Height above the plane a x + b y + c z + d = 0 (c > 0); z itself without a plane."""
    if not plane or len(plane) != 4 or not all(math.isfinite(v) for v in plane):
        return z
    a, b, c, d = plane
    return (a * x + b * y + c * z + d) / math.sqrt(a * a + b * b + c * c)


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


def odometry(rows):
    """Travelled distance per row from the reported speeds; an invalid speed holds the last one."""
    travelled, speed, previous, result = 0.0, 0.0, None, []
    for row in rows:
        stamp = row.get('measurement_stamp_ns', row['bag_stamp_ns'])
        if row.get('ego_motion_valid'):
            speed = row['ego_speed_mps']
        if previous is not None:
            travelled += speed * max(0.0, (stamp - previous) / 1e9)
        previous = stamp
        result.append(travelled)
    return result


def span(values, digits=1):
    return [round(min(values), digits), round(max(values), digits)] if values else None


def fp_events(rows, annotation, gap_s=metrics.DEFAULT_EVENT_GAP_S):
    """False-alarm events of one bag with frame counts, distances, ego speed and classes."""
    rows = sorted(rows, key=lambda row: row['bag_stamp_ns'])
    if not rows:
        return []
    travelled = dict(zip((row['bag_stamp_ns'] for row in rows), odometry(rows)))
    breaks = [row['bag_stamp_ns'] for row in rows if not row.get('ego_motion_valid')]
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
        distances, offsets, bottoms, tops = [], [], [], []
        world_x = defaultdict(list)
        for row in group:
            for obj in metrics.decisive_objects(row):
                classes[object_class(obj, row.get('route'))] += 1
                if obj.get('distance_m') is not None:
                    distances.append(obj['distance_m'])
                world_x[obj.get('id')].append(obj['center'][0] + travelled[row['bag_stamp_ns']])
                offsets.append(route_offset(obj, row.get('route'), signed=True))
                x, y, z = obj['center']
                plane = row.get('ground_plane')
                bottoms.append(height_above_ground(x, y, z - obj['size'][2] / 2, plane))
                tops.append(height_above_ground(x, y, z + obj['size'][2] / 2, plane))
        speeds = [row['ego_speed_mps'] for row in group if row.get('ego_motion_valid')]
        start = group[0]['bag_stamp_ns']
        last_break = max((stamp for stamp in breaks if stamp <= start), default=None)
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
            'ego_invalid_frames': sum(not row.get('ego_motion_valid') for row in group),
            'ego_held_frames': sum(bool((row.get('motion') or {}).get('unconfirmed_s'))
                                   for row in group),
            'since_ego_break_s': (round((start - last_break) / 1e9, 1)
                                  if last_break is not None else None),
            'tracks': len(world_x),
            'world_x_travel_m': (round(max(max(x) - min(x) for x in world_x.values()), 1)
                                 if world_x else None),
            'route_offset_m': span(offsets),
            'height_m': [round(min(bottoms), 1), round(max(tops), 1)] if tops else None,
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
                f"  ego {event['ego_speed_mps']} (invalid {event['ego_invalid_frames']},"
                f" held {event['ego_held_frames']}, break {event['since_ego_break_s']} s ago)"
                f"  tracks {event['tracks']}  world x travel {event['world_x_travel_m']} m"
                f"  offset {event['route_offset_m']}  height {event['height_m']}"
                f"  {event['classes']}")
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
