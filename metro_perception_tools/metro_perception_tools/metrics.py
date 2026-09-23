"""
Summarize exported frames and score them against reviewed annotations (Q1).

Matching rules (docs/evaluation-metrics.md):
- a frame gets the label of the reviewed interval that contains its bag_stamp_ns
  (on a shared boundary positive wins over uncertain over negative); frames
  outside every interval are 'unlabeled' and are not scored;
- 'uncertain' frames are reported but excluded from TP/FP/FN;
- on positive frames UNKNOWN and NO_OBSTACLE_DETECTED are misses (FN), so
  UNKNOWN cannot hide a missed obstacle;
- OBSTACLE frames closer than --event-gap-s are merged into one alarm event;
- frame TP/FN/precision and event detection are state-level: an OBSTACLE frame
  counts even if its candidate is another object: TP, recall and precision are
  upper bounds, while FN is a lower bound;
- object-level checks use reference frames with an annotated person ROI: a hit
  needs OBSTACLE and a candidate center inside the ROI (+ margin); OBSTACLE
  without such a candidate is a wrong-object alarm;
- distance error is computed only for object hits with a measured reference.
Empty samples give None (N/A), never a perfect score; precision is N/A
without positive frames.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

STATES = {'UNKNOWN', 'OBSTACLE', 'NO_OBSTACLE_DETECTED'}
LABELS = ('positive', 'negative', 'uncertain', 'unlabeled')
DEFAULT_EVENT_GAP_S = 0.5
DEFAULT_STAMP_TOLERANCE_S = 0.05
DEFAULT_ROI_MARGIN_M = 0.5


def summarize(rows):
    if not rows:
        raise ValueError('No result frames')
    states = Counter(row['state'] for row in rows)
    if set(states) - STATES:
        raise ValueError('Unrecognized assessment state')
    timings = np.array([row['processing_ms'] for row in rows], dtype=float)
    if not np.isfinite(timings).all() or (timings < 0).any():
        raise ValueError('Invalid processing_ms')
    return {
        'schema_version': 1, 'frames': len(rows), 'states': dict(states),
        'unknown_fraction': states['UNKNOWN'] / len(rows),
        'processing_ms': dict(zip(
            ('p50', 'p95', 'p99'),
            np.percentile(timings, [50, 95, 99]).tolist(),
        )),
        'quality_metrics': None,
        'quality_note': 'TP/FP/FN and distance error require reviewed annotations and matching.',
        'scaffold': any(row.get('mode') in {'scaffold', 'a02'} for row in rows),
    }


def ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def frame_label(stamp, intervals):
    """Return the frame label; on shared boundaries positive wins, then uncertain."""
    labels = {interval['label'] for interval in intervals
              if interval['start_ns'] <= stamp <= interval['end_ns']}
    for label in ('positive', 'uncertain', 'negative'):
        if label in labels:
            return label
    return 'unlabeled'


def alarm_events(rows, gap_ns):
    """Group OBSTACLE frames into events; rows must be sorted by bag_stamp_ns."""
    events = []
    for row in rows:
        if row['state'] != 'OBSTACLE':
            continue
        stamp = row['bag_stamp_ns']
        if events and stamp - events[-1]['end_ns'] <= gap_ns:
            events[-1]['end_ns'] = stamp
            events[-1]['frames'] += 1
        else:
            events.append({'start_ns': stamp, 'end_ns': stamp, 'frames': 1})
    return events


def inside_roi(center, roi, margin):
    return all(roi['min'][i] - margin <= center[i] <= roi['max'][i] + margin for i in range(3))


def object_checks(rows, events, tolerance_ns, margin):
    """Match candidates to annotated person ROIs on reference frames."""
    checks = []
    for event in events:
        if event.get('label') != 'positive':
            continue
        for reference in event.get('reference_frames') or []:
            roi = reference.get('person_roi_assumed_m')
            if not roi or not rows:
                continue
            stamp = reference['bag_stamp_ns']
            nearest = min(rows, key=lambda row: abs(row['bag_stamp_ns'] - stamp))
            check = {'event_id': event['id'], 'bag_stamp_ns': stamp,
                     'reference_m': reference.get('distance_m'),
                     'uncertainty_m': reference.get('uncertainty_m'), 'state': None,
                     'result': 'no_frame', 'reported_m': None, 'error_m': None}
            if abs(nearest['bag_stamp_ns'] - stamp) > tolerance_ns:
                checks.append(check)
                continue
            check['state'] = nearest['state']
            matched = [c for c in nearest.get('candidates') or []
                       if inside_roi(c['center'], roi, margin)]
            if nearest['state'] != 'OBSTACLE':
                check['result'] = 'miss'
            elif not matched:
                check['result'] = 'wrong_object'
            else:
                reported = min(c['distance_m'] for c in matched)
                check.update(result='hit', reported_m=reported)
                if check['reference_m'] is not None:
                    check['error_m'] = reported - check['reference_m']
            checks.append(check)
    return checks


def coverage(rows):
    """
    Share of frames whose corridor was evaluated and why UNKNOWN frames are UNKNOWN.

    evaluation_region_valid is not used: with background differencing the detector
    keeps it false on purpose, because differencing cannot certify a clear path.
    """
    if not rows:
        return {'frames': 0, 'observed': None, 'median_evaluated_range_m': None,
                'decision_coverage': None, 'unknown_reasons': {}}
    ranges = [row['evaluated_range_m'] for row in rows if (row.get('evaluated_range_m') or 0) > 0]
    return {
        'frames': len(rows),
        'observed': len(ranges) / len(rows),
        'decision_coverage': sum(row['state'] != 'UNKNOWN' for row in rows) / len(rows),
        'median_evaluated_range_m': float(np.median(ranges)) if ranges else None,
        'unknown_reasons': dict(Counter(
            row.get('reason') for row in rows if row['state'] == 'UNKNOWN')),
    }


def quality(rows, annotation, event_gap_s=DEFAULT_EVENT_GAP_S,
            stamp_tolerance_s=DEFAULT_STAMP_TOLERANCE_S, roi_margin_m=DEFAULT_ROI_MARGIN_M):
    """Score one bag's frames against its reviewed annotation."""
    if not annotation.get('reviewed'):
        raise ValueError('Annotation is not reviewed')
    if annotation.get('time_basis') != 'bag_stamp_ns':
        raise ValueError('Only bag_stamp_ns annotations are supported')
    rows = sorted(rows, key=lambda row: row['bag_stamp_ns'])
    intervals = annotation.get('reviewed_intervals') or []
    by_label = {label: [] for label in LABELS}
    for row in rows:
        by_label[frame_label(row['bag_stamp_ns'], intervals)].append(row)
    states = {label: dict(Counter(row['state'] for row in by_label[label])) for label in LABELS}

    positive = by_label['positive']
    negative = by_label['negative']
    tp_frames = sum(row['state'] == 'OBSTACLE' for row in positive)
    fn_frames = len(positive) - tp_frames
    fp_frames = sum(row['state'] == 'OBSTACLE' for row in negative)
    unknown_positive = sum(row['state'] == 'UNKNOWN' for row in positive)

    gap_ns = int(event_gap_s * 1e9)
    fp_events = alarm_events(negative, gap_ns)
    negative_s = sum(
        (max(r['bag_stamp_ns'] for r in group) - min(r['bag_stamp_ns'] for r in group)) / 1e9
        for group in split_runs(rows, negative)
    )
    t0 = rows[0]['bag_stamp_ns'] if rows else 0

    events = []
    for event in annotation.get('events') or []:
        if event.get('label') != 'positive':
            continue
        inside = [row for row in rows
                  if event['start_ns'] <= row['bag_stamp_ns'] <= event['end_ns']]
        alarms = [row for row in inside if row['state'] == 'OBSTACLE']
        first = alarms[0] if alarms else None
        events.append({
            'id': event['id'], 'frames': len(inside), 'obstacle_frames': len(alarms),
            'detected': bool(alarms),
            'first_alarm_delay_s': (
                (first['bag_stamp_ns'] - event['start_ns']) / 1e9 if first else None),
            'first_alarm_distance_m': first['distance_m'] if first else None,
        })
    detected = sum(event['detected'] for event in events)

    objects = object_checks(rows, annotation.get('events') or [],
                            int(stamp_tolerance_s * 1e9), roi_margin_m)
    results = Counter(check['result'] for check in objects)
    hits = [check for check in objects if check['result'] == 'hit']
    errors = [abs(check['error_m']) for check in hits if check['error_m'] is not None]
    for event in events:
        own = [check for check in hits if check['event_id'] == event['id']]
        first = min(own, key=lambda check: check['bag_stamp_ns']) if own else None
        event['object_confirmed'] = bool(own)
        event['first_matched_distance_m'] = first['reported_m'] if first else None
        event['first_matched_delay_s'] = (
            (first['bag_stamp_ns'] - next(source['start_ns'] for source in
             annotation['events'] if source['id'] == event['id'])) / 1e9
            if first else None)
    return {
        'frames_by_label': {label: len(by_label[label]) for label in LABELS},
        'states_by_label': states,
        'coverage': {label: coverage(by_label[label]) for label in LABELS},
        'coverage_scored': coverage([row for label in LABELS if label != 'unlabeled'
                                    for row in by_label[label]]),
        'frame': {
            'basis': 'state_level_upper_bound',
            'tp': tp_frames, 'fn': fn_frames, 'fp': fp_frames,
            'tn_or_unknown': len(negative) - fp_frames,
            'recall': ratio(tp_frames, len(positive)),
            'precision': ratio(tp_frames, tp_frames + fp_frames) if positive else None,
            'unknown_on_positive': unknown_positive,
            'false_alarm_rate': ratio(fp_frames, len(negative)),
        },
        'events': {
            'positive': len(events), 'detected': detected, 'missed': len(events) - detected,
            'recall': ratio(detected, len(events)), 'details': events,
        },
        'false_alarms': {
            'negative_duration_s': negative_s,
            'events': len(fp_events),
            'events_per_min': ratio(len(fp_events), negative_s / 60),
            'alarm_duration_s': alarm_frame_duration(negative, gap_ns),
            'event_span_s': sum((e['end_ns'] - e['start_ns']) / 1e9 for e in fp_events),
            'details': [{'start_s': (e['start_ns'] - t0) / 1e9,
                         'end_s': (e['end_ns'] - t0) / 1e9,
                         'frames': e['frames']} for e in fp_events],
        },
        'object': {
            'reference_frames': len(objects),
            'positive_frames': len(positive),
            'reference_coverage': ratio(len(objects), len(positive)),
            'hits': results['hit'], 'misses': results['miss'],
            'wrong_object': results['wrong_object'], 'no_frame': results['no_frame'],
            'recall': ratio(results['hit'], len(objects)),
            'checks': objects,
        },
        'distance': {
            'matched': len(errors),
            'mean_abs_error_m': float(np.mean(errors)) if errors else None,
            'max_abs_error_m': float(np.max(errors)) if errors else None,
        },
        'rules': {'event_gap_s': event_gap_s, 'stamp_tolerance_s': stamp_tolerance_s,
                  'roi_margin_m': roi_margin_m},
    }


def split_runs(rows, subset):
    """Split subset into runs of frames that are consecutive in rows."""
    members = {id(row) for row in subset}
    runs, current = [], []
    for row in rows:
        if id(row) in members:
            current.append(row)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return runs


def alarm_frame_duration(rows, gap_ns):
    """Estimate alarm occupancy from cadence, including isolated alarms when known."""
    if not any(row['state'] == 'OBSTACLE' for row in rows):
        return 0.0
    if len(rows) < 2:
        return None
    gaps = [right['bag_stamp_ns'] - left['bag_stamp_ns']
            for left, right in zip(rows, rows[1:])
            if 0 < right['bag_stamp_ns'] - left['bag_stamp_ns'] <= gap_ns]
    if not gaps:
        return None
    cadence = float(np.median(gaps))
    return sum(min((rows[i + 1]['bag_stamp_ns'] - row['bag_stamp_ns'])
                   if i + 1 < len(rows) else cadence, cadence)
               for i, row in enumerate(rows) if row['state'] == 'OBSTACLE') / 1e9


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def aggregate(bags):
    """Sum per-bag quality into totals; ratios are recomputed, not averaged."""
    total = Counter()
    for bag in bags.values():
        q = bag['quality']
        total['positive_frames'] += q['frames_by_label']['positive']
        total['negative_frames'] += q['frames_by_label']['negative']
        total['uncertain_frames'] += q['frames_by_label']['uncertain']
        for key in ('tp', 'fn', 'fp', 'unknown_on_positive'):
            total[key] += q['frame'][key]
        total['positive_events'] += q['events']['positive']
        total['detected_events'] += q['events']['detected']
        total['fp_events'] += q['false_alarms']['events']
        total['object_reference_frames'] += q['object']['reference_frames']
        total['object_hits'] += q['object']['hits']
        total['object_wrong'] += q['object']['wrong_object']
        total['negative_duration_s'] += q['false_alarms']['negative_duration_s']
    count_keys = ('positive_frames', 'negative_frames', 'uncertain_frames', 'tp', 'fn',
                  'fp', 'unknown_on_positive', 'positive_events', 'detected_events',
                  'fp_events', 'object_reference_frames', 'object_hits', 'object_wrong',
                  'negative_duration_s')
    result = {key: total[key] for key in count_keys}
    result.update(
        frame_recall=ratio(total['tp'], total['positive_frames']),
        frame_precision=(ratio(total['tp'], total['tp'] + total['fp'])
                         if total['positive_frames'] else None),
        false_alarm_rate=ratio(total['fp'], total['negative_frames']),
        event_recall=ratio(total['detected_events'], total['positive_events']),
        fp_events_per_min=ratio(total['fp_events'], total['negative_duration_s'] / 60),
        object_recall=ratio(total['object_hits'], total['object_reference_frames']),
        basis='frame and event values are state-level upper bounds; object_* are ROI-matched',
    )
    return result


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def find_results(run_dir, bag_id):
    for candidate in (run_dir / bag_id / 'frames.jsonl', run_dir / f'{bag_id}.jsonl'):
        if candidate.is_file():
            return candidate
    return None


def evaluate_run(run_dir, dataset_path, splits_path, root, **rules):
    """Score every annotated bag of a run directory and group totals by split."""
    dataset = yaml.safe_load(dataset_path.read_text())
    splits = yaml.safe_load(splits_path.read_text()) if splits_path.is_file() else {}
    split_of = {bag: name for name in ('development', 'validation', 'holdout', 'regression')
                for bag in splits.get(name) or []}
    manifest_path = run_dir / 'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    profiles = {bag['id']: bag.get('sensor_profile') for bag in manifest.get('bags', [])}
    manifest_bags = {bag['id']: bag for bag in manifest.get('bags', [])}
    run_issues, scoring_issues = [], []
    if not manifest:
        run_issues.append('manifest missing')
        scoring_issues.append('manifest missing')
    else:
        if manifest.get('dirty') is not False:
            run_issues.append('source checkout was dirty')
        if not manifest.get('image_id') or manifest['image_id'] == 'not_recorded':
            run_issues.append('image id missing')
        if not manifest.get('commit'):
            run_issues.append('source commit missing')
        if manifest.get('dataset_sha256') != sha256(dataset_path):
            run_issues.append('dataset hash mismatch')
        for relative, digest in (manifest.get('config_sha256') or {}).items():
            path = root / relative
            if not path.is_file() or sha256(path) != digest:
                run_issues.append(f'config hash mismatch: {relative}')
        if not manifest.get('config_sha256'):
            run_issues.append('config hashes missing')
        # The image and commit alone do not identify the binary that produced the frames.
        if not manifest.get('executable_sha256'):
            run_issues.append('executable hashes missing')
        recorded = manifest.get('scoring_sha256') or {}
        score_files = [
            splits_path,
            root / 'metro_perception_tools/metro_perception_tools/metrics.py',
            root / 'metro_perception_tools/metro_perception_tools/report.py',
        ]
        score_files += [root / entry['annotations'] for entry in dataset['bags']
                        if entry.get('annotations')]
        if not recorded:
            scoring_issues.append('scoring hashes missing')
        else:
            for path in score_files:
                relative = str(path.relative_to(root))
                if not path.is_file() or recorded.get(relative) != sha256(path):
                    scoring_issues.append(f'scoring hash missing or changed: {relative}')
        if manifest.get('scoring_executed_from') != 'source_tree':
            scoring_issues.append('scorer executable not tied to source hashes')
        selected = manifest.get('selected_bags')
        if selected is None:
            selected = [entry['id'] for entry in dataset['bags']]
        if set(selected) != {bag['id'] for bag in manifest.get('bags', [])}:
            run_issues.append('selected bags incomplete')
        if any(bag.get('status') != 'exported' for bag in manifest.get('bags', [])):
            run_issues.append('bag export incomplete')
        if any(not bag.get('files') for bag in manifest.get('bags', [])):
            run_issues.append('input file hashes missing')
    bags, skipped = {}, {}
    for entry in dataset['bags']:
        results = find_results(run_dir, entry['id'])
        if not entry.get('annotations'):
            skipped[entry['id']] = 'no annotations'
            if entry['id'] in manifest_bags:
                if results is None:
                    run_issues.append(f'selected bag has no results: {entry["id"]}')
                else:
                    exported = read_rows(results)
                    declared = entry.get('declared_message_count')
                    if declared is not None and len(exported) != declared:
                        run_issues.append(f'incomplete frame export: {entry["id"]}')
                    recorded_hash = manifest_bags[entry['id']].get('frames_sha256')
                    if manifest.get('scoring_sha256') and recorded_hash != sha256(results):
                        run_issues.append(f'result hash missing or changed: {entry["id"]}')
            continue
        if results is None:
            selected = manifest.get('selected_bags')
            not_selected = selected is not None and entry['id'] not in selected
            skipped[entry['id']] = 'not selected' if not_selected else 'no results'
            continue
        rows = read_rows(results)
        if (manifest.get('scoring_sha256') and
                manifest_bags.get(entry['id'], {}).get('frames_sha256') != sha256(results)):
            run_issues.append(f'result hash missing or changed: {entry["id"]}')
        summary = summarize(rows)
        declared = entry.get('declared_message_count')
        if manifest and declared is not None and len(rows) != declared:
            run_issues.append(f'incomplete frame export: {entry["id"]}')
        stamps = [row['bag_stamp_ns'] for row in rows]
        if manifest and (stamps != sorted(set(stamps))):
            run_issues.append(f'duplicate or unordered frames: {entry["id"]}')
        bags[entry['id']] = {
            'split': split_of.get(entry['id'], 'unassigned'),
            'results': str(results),
            'frames': {'declared': declared, 'processed': len(rows),
                       'lost': declared - len(rows) if declared is not None else None},
            'sensor_profile': profiles.get(entry['id']),
            'calibration_trust': dict(Counter(str(row.get('calibration_trust')) for row in rows)),
            'calibration_verified': any(row.get('calibration_verified') for row in rows),
            'states': summary['states'],
            'processing_ms': summary['processing_ms'],
            'quality': quality(rows, yaml.safe_load((root / entry['annotations']).read_text()),
                               **rules),
        }
    if manifest:
        for bag in manifest.get('bags', []):
            if bag['id'] not in bags and skipped.get(bag['id']) != 'no annotations':
                run_issues.append(f'selected bag has no results: {bag["id"]}')
    by_split = {}
    for name in sorted({bag['split'] for bag in bags.values()}):
        by_split[name] = aggregate({k: v for k, v in bags.items() if v['split'] == name})
    return {
        'schema_version': 2, 'run_dir': str(run_dir),
        'run_reproducible': not run_issues,
        'run_issues': run_issues,
        'scoring_current': not scoring_issues,
        'scoring_issues': scoring_issues,
        'provenance': {key: manifest.get(key) for key in (
            'commit', 'dirty', 'image_id', 'build_info', 'executable_sha256',
            'dataset_sha256', 'config_sha256', 'preview',
            'full_scan_research', 'max_points', 'max_cloud_bytes', 'tf_lookahead_s')}
        if manifest else None,
        'bags': bags, 'skipped': skipped,
        'by_split': by_split, 'total': aggregate(bags),
    }


def main():
    root = Path(__file__).resolve().parents[1]
    for parent in Path(__file__).resolve().parents:
        if (parent / 'evaluation/dataset.yaml').is_file():
            root = parent
            break
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('results', type=Path, help='frames.jsonl or a run directory')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--annotations', type=Path, help='annotation YAML for a single JSONL')
    parser.add_argument('--root', type=Path, default=root, help='repository root')
    parser.add_argument('--event-gap-s', type=float, default=DEFAULT_EVENT_GAP_S)
    parser.add_argument('--stamp-tolerance-s', type=float, default=DEFAULT_STAMP_TOLERANCE_S)
    parser.add_argument('--roi-margin-m', type=float, default=DEFAULT_ROI_MARGIN_M)
    args = parser.parse_args()
    rules = {'event_gap_s': args.event_gap_s, 'stamp_tolerance_s': args.stamp_tolerance_s,
             'roi_margin_m': args.roi_margin_m}
    if args.results.is_dir():
        result = evaluate_run(args.results, args.root / 'evaluation/dataset.yaml',
                              args.root / 'evaluation/splits.yaml', args.root, **rules)
    else:
        rows = read_rows(args.results)
        result = summarize(rows)
        if args.annotations:
            result['quality_metrics'] = quality(
                rows, yaml.safe_load(args.annotations.read_text()), **rules)
            result['quality_note'] = 'Scored against ' + str(args.annotations)
    with args.output.open('x') as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')


if __name__ == '__main__':
    main()
