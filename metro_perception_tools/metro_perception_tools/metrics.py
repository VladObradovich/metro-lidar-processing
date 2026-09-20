"""Summarize exported frames; ground-truth quality metrics are not implemented."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def summarize(rows):
    if not rows:
        raise ValueError('No result frames')
    states = Counter(row['state'] for row in rows)
    if set(states) - {'UNKNOWN', 'OBSTACLE', 'NO_OBSTACLE_DETECTED'}:
        raise ValueError('Unrecognized assessment state')
    timings = np.array([row['processing_ms'] for row in rows], dtype=float)
    if not np.isfinite(timings).all() or (timings < 0).any():
        raise ValueError('Invalid processing_ms')
    return {
        'schema_version': 1, 'frames': len(rows), 'states': dict(states),
        'unknown_fraction': states['UNKNOWN'] / len(rows),
        'processing_ms': dict(zip(('p50', 'p95', 'p99'),
                                 np.percentile(timings, [50, 95, 99]).tolist())),
        'quality_metrics': None,
        'quality_note': 'TP/FP/FN and distance error require reviewed annotations and matching.',
        'scaffold': any(row.get('mode') == 'scaffold' for row in rows),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.results.read_text().splitlines() if line.strip()]
    result = summarize(rows)
    with args.output.open('x') as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')
