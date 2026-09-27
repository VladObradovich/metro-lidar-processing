#!/usr/bin/env python3
"""
Audit every PointCloud2 frame of the registered bags (D1 dataset passport).

Unlike inspect_bag.py, which looks at the first samples only, this script reads
all frames of the input topic and checks what the pipeline relies on: one
frame_id, one field layout, consistent buffer sizes, finite XYZ, monotonic
timestamps and the gap between bag and header time.

Runs in the host venv (pip install rosbags numpy pyyaml) or in the container:

  python3 scripts/audit_bags.py --dataset-root rosbags --output results/d1-audit.json
"""

import argparse
import json
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import yaml

POINT_FIELD_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def xyz_finite_count(message) -> int:
    """Count points whose x, y and z are all finite."""
    byte_order = '>' if message.is_bigendian else '<'
    fields = {field.name: field for field in message.fields}
    names = ['x', 'y', 'z']
    dtype = np.dtype(
        {
            'names': names,
            'formats': [f'{byte_order}{POINT_FIELD_DTYPES[fields[n].datatype]}' for n in names],
            'offsets': [fields[n].offset for n in names],
            'itemsize': message.point_step,
        }
    )
    finite = np.ones(message.width * message.height, dtype=bool)
    for row in range(message.height):
        start = row * message.row_step
        points = np.frombuffer(message.data, dtype=dtype, count=message.width, offset=start)
        mask = finite[row * message.width:(row + 1) * message.width]
        for name in names:
            mask &= np.isfinite(points[name])
    return int(finite.sum())


def summary(values) -> dict:
    """Return min, median and max of a sequence."""
    array = np.asarray(values, dtype=float)
    return {
        'min': float(array.min()),
        'median': float(np.median(array)),
        'max': float(array.max()),
    }


def audit_bag(path: Path, topic: str) -> dict:
    """Audit all frames of one topic in one bag."""
    frame_ids, layouts, problems = set(), set(), []
    bag_stamps, header_stamps, points, finite = [], [], [], []
    with AnyReader([path], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        other_topics = sorted(c.topic for c in reader.connections if c.topic != topic)
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise SystemExit(f'{path}: topic {topic} not found')
        for connection, bag_stamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            index = len(bag_stamps)
            frame_ids.add(message.header.frame_id)
            layouts.add(
                (
                    message.point_step,
                    message.is_bigendian,
                    tuple((f.name, f.offset, f.datatype, f.count) for f in message.fields),
                )
            )
            if message.row_step < message.width * message.point_step:
                problems.append(f'frame {index}: row_step < width * point_step')
            if len(message.data) < message.row_step * message.height:
                problems.append(f'frame {index}: data shorter than row_step * height')
            bag_stamps.append(bag_stamp)
            header_stamps.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
            points.append(message.width * message.height)
            finite.append(xyz_finite_count(message))

    bag_dt = np.diff(bag_stamps) / 1e6
    header_dt = np.diff(header_stamps) / 1e6
    offset = (np.asarray(bag_stamps) - np.asarray(header_stamps)) / 1e6
    return {
        'topic': topic,
        'frames': len(bag_stamps),
        'first_bag_stamp_ns': bag_stamps[0],
        'last_bag_stamp_ns': bag_stamps[-1],
        'duration_s': (bag_stamps[-1] - bag_stamps[0]) / 1e9,
        'frame_ids': sorted(frame_ids),
        'field_layouts': [
            {'point_step': step, 'is_bigendian': big, 'fields': [list(f) for f in fields]}
            for step, big, fields in sorted(layouts)
        ],
        'points_per_frame': summary(points),
        'finite_xyz_per_frame': summary(finite),
        'bag_period_ms': summary(bag_dt),
        'header_period_ms': summary(header_dt),
        'bag_minus_header_ms': summary(offset),
        'bag_stamps_monotonic': bool((bag_dt > 0).all()),
        'header_stamps_monotonic': bool((header_dt > 0).all()),
        'gaps_over_150ms': int((bag_dt > 150).sum()),
        'other_topics': other_topics,
        'problems': problems,
    }


def main():
    """Audit the selected bags and write one JSON report."""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, default=root / 'evaluation/dataset.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bags', nargs='*', help='Bag ids to audit (default: all but new_data)')
    args = parser.parse_args()

    dataset = yaml.safe_load(args.dataset.read_text())
    selected = args.bags or [e['id'] for e in dataset['bags'] if e['id'] != 'new_data']
    result = {'schema_version': 1, 'scope': 'all_frames', 'bags': {}}
    for entry in dataset['bags']:
        if entry['id'] not in selected:
            continue
        print(f"auditing {entry['id']} ...", flush=True)
        bag = args.dataset_root / entry['path']
        result['bags'][entry['id']] = audit_bag(bag, entry['input_topic'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
