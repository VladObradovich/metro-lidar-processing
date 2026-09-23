import json

import pytest
from metro_perception_tools.metrics import evaluate_run, quality, sha256, summarize
from metro_perception_tools.report import quality_lines


def test_unknown_does_not_become_a_true_negative():
    summary = summarize([
        {'state': 'UNKNOWN', 'processing_ms': 1, 'mode': 'scaffold'},
        {'state': 'NO_OBSTACLE_DETECTED', 'processing_ms': 2},
    ])
    assert summary['unknown_fraction'] == 0.5
    assert summary['quality_metrics'] is None
    assert summary['scaffold']


def test_invalid_or_empty_results_fail_instead_of_reporting_success():
    with pytest.raises(ValueError):
        summarize([])
    with pytest.raises(ValueError):
        summarize([{'state': 'UNKNOWN', 'processing_ms': float('nan')}])


def test_a02_is_not_a_completed_detector():
    result = summarize([{'state': 'UNKNOWN', 'processing_ms': 1, 'mode': 'a02'}])
    assert result['scaffold']
    assert result['quality_metrics'] is None


def frame(stamp, state, distance=None, candidates=()):
    return {'bag_stamp_ns': stamp, 'state': state, 'processing_ms': 1,
            'distance_m': distance, 'candidates': list(candidates)}


def annotation(intervals, events=()):
    return {'reviewed': True, 'time_basis': 'bag_stamp_ns',
            'reviewed_intervals': [
                {'start_ns': start, 'end_ns': end, 'label': label}
                for start, end, label in intervals],
            'events': list(events)}


S = 10**9


def test_unknown_on_positive_is_a_miss():
    rows = [frame(0, 'OBSTACLE'), frame(S // 10, 'UNKNOWN'),
            frame(2 * S // 10, 'NO_OBSTACLE_DETECTED')]
    result = quality(rows, annotation([(0, S, 'positive')]))
    assert result['frame'] == {**result['frame'], 'tp': 1, 'fn': 2, 'unknown_on_positive': 1}
    assert result['frame']['recall'] == 1 / 3


def test_shared_boundary_prefers_positive_and_uncertain_is_not_scored():
    rows = [frame(0, 'OBSTACLE'), frame(S, 'OBSTACLE'), frame(2 * S, 'UNKNOWN')]
    result = quality(rows, annotation([(0, S, 'uncertain'), (S, 2 * S, 'positive')]))
    assert result['frames_by_label']['uncertain'] == 1
    assert result['frames_by_label']['positive'] == 2
    assert result['frame']['fp'] == 0


def test_false_alarms_are_grouped_into_events_by_gap():
    stamps = [0, 1, 2, 10, 11, 30]
    rows = [frame(t * S // 10, 'OBSTACLE') for t in stamps]
    rows += [frame(t * S // 10, 'UNKNOWN') for t in range(40, 61)]
    result = quality(rows, annotation([(0, 6 * S, 'negative')]))
    alarms = result['false_alarms']
    assert result['frame']['fp'] == 6
    assert alarms['events'] == 3
    assert alarms['negative_duration_s'] == 6.0
    assert alarms['events_per_min'] == 30.0
    assert result['frame']['precision'] is None


def test_empty_samples_are_not_perfect_scores():
    result = quality([frame(0, 'UNKNOWN')], annotation([]))
    assert result['frame']['recall'] is None
    assert result['frame']['false_alarm_rate'] is None
    assert result['events']['recall'] is None
    assert result['distance']['mean_abs_error_m'] is None
    assert result['object']['recall'] is None


def test_distance_error_needs_a_candidate_inside_the_reference_roi():
    roi = {'min': [10.0, -1.0, -1.0], 'max': [11.0, 1.0, 1.0]}
    reference = {'bag_stamp_ns': S, 'distance_m': 10.0, 'uncertainty_m': 0.2,
                 'person_roi_assumed_m': roi}
    event = {'id': 'person', 'label': 'positive', 'start_ns': 0, 'end_ns': 2 * S,
             'reference_frames': [reference, {**reference, 'bag_stamp_ns': 2 * S}]}
    person = {'center': [10.5, 0.0, 0.0], 'distance_m': 10.3}
    wall = {'center': [5.0, 3.0, 0.0], 'distance_m': 5.0}
    rows = [frame(S, 'OBSTACLE', 5.0, [wall, person]), frame(2 * S, 'OBSTACLE', 5.0, [wall])]
    result = quality(rows, annotation([(0, 2 * S, 'positive')], [event]))
    first, second = result['object']['checks']
    assert first['result'] == 'hit' and abs(first['error_m'] - 0.3) < 1e-9
    assert second['result'] == 'wrong_object' and second['error_m'] is None
    assert result['distance']['matched'] == 1
    event = result['events']['details'][0]
    assert event['detected'] and event['object_confirmed']
    assert event['first_matched_distance_m'] == 10.3


def test_alarm_on_another_object_is_not_an_object_hit():
    roi = {'min': [10.0, -1.0, -1.0], 'max': [11.0, 1.0, 1.0]}
    event = {'id': 'person', 'label': 'positive', 'start_ns': 0, 'end_ns': S,
             'reference_frames': [{'bag_stamp_ns': 0, 'distance_m': None,
                                   'person_roi_assumed_m': roi}]}
    wall = {'center': [5.0, 3.0, 0.0], 'distance_m': 5.0}
    result = quality([frame(0, 'OBSTACLE', 5.0, [wall])],
                     annotation([(0, S, 'positive')], [event]))
    assert result['frame']['tp'] == 1
    assert result['frame']['basis'] == 'state_level_upper_bound'
    assert result['object'] == {**result['object'], 'hits': 0, 'wrong_object': 1, 'recall': 0}
    assert result['events']['details'][0]['detected']
    assert not result['events']['details'][0]['object_confirmed']


def test_coverage_reports_observed_corridor_and_unknown_reasons():
    rows = [{**frame(0, 'UNKNOWN'), 'evaluated_range_m': 0, 'reason': 'WARMUP'},
            {**frame(S, 'OBSTACLE'), 'evaluated_range_m': 40.0}]
    result = quality(rows, annotation([(0, S, 'negative')]))
    negative = result['coverage']['negative']
    assert negative['observed'] == 0.5
    assert negative['decision_coverage'] == 0.5
    assert negative['median_evaluated_range_m'] == 40.0
    assert negative['unknown_reasons'] == {'WARMUP': 1}
    assert result['coverage']['positive']['observed'] is None


def test_geometry_is_distinct_from_decision_and_median_uses_all_scored_frames():
    rows = [{**frame(0, 'UNKNOWN'), 'evaluated_range_m': 10.0},
            {**frame(S, 'OBSTACLE'), 'evaluated_range_m': 20.0},
            {**frame(2 * S, 'UNKNOWN'), 'evaluated_range_m': 30.0}]
    result = quality(rows, annotation([(0, 2 * S, 'positive')]))
    assert result['coverage_scored']['observed'] == 1
    assert result['coverage_scored']['decision_coverage'] == 1 / 3
    assert result['coverage_scored']['median_evaluated_range_m'] == 20.0


def test_single_frame_alarm_has_nonzero_occupancy():
    rows = [frame(0, 'UNKNOWN'), frame(S // 10, 'OBSTACLE'),
            frame(2 * S // 10, 'UNKNOWN')]
    result = quality(rows, annotation([(0, S, 'negative')]))
    assert result['false_alarms']['event_span_s'] == 0
    assert result['false_alarms']['alarm_duration_s'] == pytest.approx(0.1)


def test_one_frame_sample_cannot_estimate_alarm_duration():
    result = quality([frame(0, 'OBSTACLE')], annotation([(0, 0, 'negative')]))
    assert result['false_alarms']['alarm_duration_s'] is None


def test_unannotated_bag_is_skipped_by_quality(tmp_path):
    dataset = tmp_path / 'evaluation/dataset.yaml'
    dataset.parent.mkdir()
    dataset.write_text('bags:\n- id: new_data\n  annotations: null\n')
    splits = tmp_path / 'evaluation/splits.yaml'
    splits.write_text('development: []\n')
    run = tmp_path / 'run'
    (run / 'new_data').mkdir(parents=True)
    (run / 'new_data/frames.jsonl').write_text(json.dumps(frame(0, 'UNKNOWN')) + '\n')
    result = evaluate_run(run, dataset, splits, tmp_path)
    assert result['skipped'] == {'new_data': 'no annotations'}
    assert result['bags'] == {}
    assert result['total']['frame_recall'] is None
    assert 'new_data (no annotations)' in '\n'.join(quality_lines(result))
    (run / 'manifest.json').write_text(json.dumps({
        'selected_bags': ['new_data'],
        'bags': [{'id': 'new_data', 'status': 'exported', 'files': {'data.db3': 'abc'}}],
    }))
    (run / 'new_data/frames.jsonl').unlink()
    missing = evaluate_run(run, dataset, splits, tmp_path)
    assert any('selected bag has no results' in issue for issue in missing['run_issues'])


def test_skipped_distinguishes_not_selected_from_missing_results(tmp_path):
    dataset = tmp_path / 'evaluation/dataset.yaml'
    dataset.parent.mkdir()
    dataset.write_text('bags:\n- id: chosen\n  annotations: a.yaml\n'
                       '- id: other\n  annotations: a.yaml\n')
    splits = tmp_path / 'evaluation/splits.yaml'
    splits.write_text('development: []\n')
    run = tmp_path / 'run'
    run.mkdir()
    (run / 'manifest.json').write_text(json.dumps({
        'selected_bags': ['chosen'], 'bags': [{'id': 'chosen', 'status': 'exported'}]}))
    result = evaluate_run(run, dataset, splits, tmp_path)
    assert result['skipped'] == {'chosen': 'no results', 'other': 'not selected'}
    lines = quality_lines(result)
    header = lines[2:next(i for i, line in enumerate(lines) if line.startswith('Калибровка'))]
    assert '' not in header


def test_clean_run_remains_reproducible_without_scoring_hashes(tmp_path):
    dataset = tmp_path / 'evaluation/dataset.yaml'
    dataset.parent.mkdir()
    dataset.write_text('bags:\n- id: sample\n  annotations: evaluation/annotations/sample.yaml\n'
                       '  declared_message_count: 1\n')
    splits = tmp_path / 'evaluation/splits.yaml'
    splits.write_text('development: [sample]\n')
    labels = tmp_path / 'evaluation/annotations/sample.yaml'
    labels.parent.mkdir()
    labels.write_text('reviewed: true\ntime_basis: bag_stamp_ns\nreviewed_intervals:\n'
                      '- {start_ns: 0, end_ns: 0, label: negative}\n')
    run = tmp_path / 'run'
    (run / 'sample').mkdir(parents=True)
    (run / 'sample/frames.jsonl').write_text(json.dumps(frame(0, 'UNKNOWN')) + '\n')
    config = tmp_path / 'config.yaml'
    config.write_text('value: 1\n')
    (run / 'manifest.json').write_text(json.dumps({
        'dirty': False, 'commit': 'abc', 'image_id': 'sha256:abc',
        'dataset_sha256': sha256(dataset), 'selected_bags': ['sample'],
        'config_sha256': {'config.yaml': sha256(config)},
        'bags': [{'id': 'sample', 'status': 'exported', 'files': {'sample.db3': 'abc'}}],
    }))
    result = evaluate_run(run, dataset, splits, tmp_path)
    assert result['run_reproducible']
    assert not result['scoring_current']
    assert any('scoring hash' in issue for issue in result['scoring_issues'])

    package = tmp_path / 'metro_perception_tools/metro_perception_tools'
    package.mkdir(parents=True)
    metrics_source = package / 'metrics.py'
    report_source = package / 'report.py'
    metrics_source.write_text('version = 1\n')
    report_source.write_text('version = 1\n')
    paths = [splits, labels, metrics_source, report_source]
    manifest_path = run / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['scoring_executed_from'] = 'source_tree'
    manifest['scoring_sha256'] = {str(path.relative_to(tmp_path)): sha256(path)
                                  for path in paths}
    manifest['bags'][0]['frames_sha256'] = sha256(run / 'sample/frames.jsonl')
    manifest_path.write_text(json.dumps(manifest))
    assert evaluate_run(run, dataset, splits, tmp_path)['scoring_current']
    metrics_source.write_text('version = 2\n')
    changed = evaluate_run(run, dataset, splits, tmp_path)
    assert changed['run_reproducible']
    assert not changed['scoring_current']


def test_unreviewed_annotation_is_rejected():
    with pytest.raises(ValueError):
        quality([frame(0, 'OBSTACLE')], {'reviewed': False, 'time_basis': 'bag_stamp_ns'})
