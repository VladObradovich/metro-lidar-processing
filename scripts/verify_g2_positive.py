#!/usr/bin/env python3
"""
Check hand-marked person regions against raw lidar and G2 candidates.

The ranges are longitudinal coordinates in the assumed full-scan target frame,
not surveyed train-to-person distances. This script never derives a reference
range from a detector candidate.
"""

import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


EMPTY_STAMP = 1788354633105858961  # Visually reviewed empty frame, bag +10 s.


def points_at(db, stamp):
    row = db.execute('SELECT data FROM messages WHERE timestamp = ?',
                     (stamp,)).fetchone()
    if row is None:
        raise ValueError(f'No PointCloud2 at bag stamp {stamp}')
    msg = deserialize_message(row[0], PointCloud2)
    fields = {field.name: field.offset for field in msg.fields}
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': ['<f4'] * 3,
                      'offsets': [fields[name] for name in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    # Research profile rotation: sensor -Y -> assumed train-forward X.
    return np.stack((-raw['y'], raw['x'], raw['z']), axis=1)


def selected_points(points, roi):
    low = np.array(roi['min'])
    high = np.array(roi['max'])
    return points[np.isfinite(points).all(axis=1) &
                  (points >= low).all(axis=1) &
                  (points <= high).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_db', type=Path)
    parser.add_argument('annotation', type=Path)
    parser.add_argument('evaluation_jsonl', type=Path)
    args = parser.parse_args()
    labels = yaml.safe_load(args.annotation.read_text())
    references = [item for item in labels['events'][0]['reference_frames']
                  if 'person_roi_assumed_m' in item]
    rows = {row['bag_stamp_ns']: row for row in
            (json.loads(line) for line in args.evaluation_jsonl.open())}
    assert len(references) >= 2
    with sqlite3.connect(f'file:{args.bag_db}?mode=ro', uri=True) as db:
        empty = points_at(db, EMPTY_STAMP)
        for label in references:
            stamp = label['bag_stamp_ns']
            roi = label['person_roi_assumed_m']
            before = selected_points(empty, roi)
            person = selected_points(points_at(db, stamp), roi)
            # An empty reference ROI guards against annotating fixed tunnel fabric.
            assert len(before) == 0, (stamp, len(before))
            assert len(person) >= 100, (stamp, len(person))
            inside = person[np.abs(person[:, 1]) <= 2.0]
            assert len(inside) >= 50, (stamp, len(inside))
            reference = float(inside[:, 0].min())
            assert abs(reference - label['distance_m']) < 0.001
            candidates = [candidate for candidate in rows[stamp]['candidates']
                          if all(lo <= value <= hi for lo, value, hi in
                                 zip(roi['min'], candidate['center'], roi['max']))]
            assert len(candidates) == 1, (stamp, candidates)
            measured = candidates[0]['distance_m']
            error = abs(measured - reference)
            assert error <= label['uncertainty_m'], (stamp, error)
            print(f'{stamp}: person points={len(person)}, inside={len(inside)}, '
                  f'candidate={measured:.3f} m, raw reference={reference:.3f} m, '
                  f'error={error:.3f} m')
    print('PASS: two independently selected real-person regions matched')


if __name__ == '__main__':
    main()
