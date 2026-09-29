import json
from pathlib import Path

import pytest
from metro_perception_tools.metrics import evaluate_run, profile_range, quality, sha256, summarize
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
    # Commit and image do not identify the binary that produced the frames.
    assert not result['run_reproducible']
    assert result['run_issues'] == ['executable hashes missing']
    manifest = json.loads((run / 'manifest.json').read_text())
    manifest['executable_sha256'] = {
        'metro_perception_ros/lib/metro_perception_ros/evaluate_bag': {
            'path': '/tmp/i/lib/metro_perception_ros/evaluate_bag', 'sha256': 'abc'}}
    (run / 'manifest.json').write_text(json.dumps(manifest))
    result = evaluate_run(run, dataset, splits, tmp_path)
    assert result['run_reproducible'], result['run_issues']
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


def test_confirmed_tracks_decide_object_hits_when_present():
    roi = {'min': [10.0, -1.0, -1.0], 'max': [11.0, 1.0, 1.0]}
    event = {'id': 'person', 'label': 'positive', 'start_ns': 0, 'end_ns': S,
             'reference_frames': [{'bag_stamp_ns': 0, 'distance_m': None,
                                   'person_roi_assumed_m': roi}]}
    person = {'center': [10.5, 0.0, 0.0], 'distance_m': 10.3}
    row = frame(0, 'OBSTACLE', 10.3, [person])
    row['tracks'] = [{**person, 'confirmed': False}]
    result = quality([row], annotation([(0, S, 'positive')], [event]))
    assert result['object']['wrong_object'] == 1  # Only an unconfirmed track in the ROI.
    row['tracks'] = [{**person, 'confirmed': True}]
    result = quality([row], annotation([(0, S, 'positive')], [event]))
    assert result['object']['hits'] == 1


def test_object_checks_are_grouped_by_object_distance():
    def reference(stamp, distance, near_face):
        return {'bag_stamp_ns': stamp, 'distance_m': distance,
                'person_roi_assumed_m': {'min': [near_face, -1.0, -1.0],
                                         'max': [near_face + 1.0, 1.0, 1.0]}}
    event = {'id': 'box', 'label': 'positive', 'start_ns': 0, 'end_ns': 3 * S,
             'reference_frames': [reference(0, 15.0, 15.0), reference(S, None, 50.0),
                                  reference(2 * S, 130.0, 130.0), reference(3 * S, 0.5, -0.5)]}
    near = {'center': [15.5, 0.0, 0.0], 'distance_m': 15.0}
    wall = {'center': [5.0, 3.0, 0.0], 'distance_m': 5.0}
    rows = [frame(0, 'OBSTACLE', 15.0, [near]), frame(S, 'OBSTACLE', 5.0, [wall]),
            frame(2 * S, 'UNKNOWN'), frame(3 * S, 'UNKNOWN')]
    result = quality(rows, annotation([(0, 3 * S, 'positive')], [event]))
    bins = {tuple(item['range_m']): item for item in result['object']['by_distance']}
    # Without a measured distance the near face of the ROI places the check.
    assert bins[(0, 20)] == {'range_m': [0, 20], 'frames': 2, 'obstacle': 1, 'hits': 1,
                             'recall': 0.5}
    assert bins[(40, 60)]['obstacle'] == 1 and bins[(40, 60)]['hits'] == 0
    assert bins[(120, 150)]['frames'] == 1 and bins[(120, 150)]['recall'] == 0
    assert len(bins) == 3


def test_alarm_on_an_annotated_negative_object_is_counted_on_every_frame():
    side = {'min': [10.0, 2.0, -1.0], 'max': [11.0, 3.0, 1.0]}
    distractor = {'id': 'side', 'label': 'negative', 'envelope_class': 'outside',
                  'start_ns': 0, 'end_ns': 3 * S,
                  'reference_frames': [{'bag_stamp_ns': t * S, 'distance_m': 10.0,
                                        'person_roi_assumed_m': side} for t in range(4)]}
    on_side = {'center': [10.5, 2.5, 0.0], 'distance_m': 10.0, 'confirmed': True}
    elsewhere = {'center': [30.0, 0.0, 0.0], 'distance_m': 30.0, 'confirmed': True}
    rows = [frame(0, 'OBSTACLE', 10.0), frame(S, 'OBSTACLE', 10.0),
            frame(2 * S, 'OBSTACLE', 30.0), frame(3 * S, 'UNKNOWN')]
    for row, tracks in zip(rows, ([on_side], [on_side, elsewhere], [elsewhere], [on_side])):
        row['tracks'] = tracks
    # The second frame is positive because of another object: the frame-level FP misses it.
    result = quality(rows, annotation([(0, 0, 'negative'), (S, S, 'positive'),
                                       (2 * S, 3 * S, 'negative')], [distractor]))
    negative = result['negative_objects']
    assert result['frame']['fp'] == 2
    assert negative['reference_frames'] == 4
    assert negative['alarm_frames'] == 2  # UNKNOWN with a track on it is no alarm
    assert negative['alarm_frames_on_negative'] == 1
    assert negative['events']['side'] == {'class': 'outside', 'frames': 4, 'alarm_frames': 2,
                                          'positive_frames': 1, 'alarm_frames_on_positive': 1}


def test_report_shows_distance_bins_and_alarms_on_negative_objects(tmp_path):
    dataset = tmp_path / 'evaluation/dataset.yaml'
    dataset.parent.mkdir()
    dataset.write_text('bags:\n- id: scene\n  annotations: evaluation/annotations/scene.yaml\n')
    splits = tmp_path / 'evaluation/splits.yaml'
    splits.write_text('regression: [scene]\n')
    roi = {'min': [10.0, -1.0, -1.0], 'max': [11.0, 1.0, 1.0]}
    side = {'min': [30.0, 2.0, -1.0], 'max': [31.0, 3.0, 1.0]}
    labels = annotation([(0, 0, 'positive'), (S, S, 'negative')], [
        {'id': 'box', 'label': 'positive', 'start_ns': 0, 'end_ns': 0,
         'reference_frames': [{'bag_stamp_ns': 0, 'distance_m': 10.0,
                               'person_roi_assumed_m': roi}]},
        {'id': 'side', 'label': 'negative', 'envelope_class': 'above', 'start_ns': S,
         'end_ns': S, 'reference_frames': [{'bag_stamp_ns': S, 'distance_m': 30.0,
                                            'person_roi_assumed_m': side}]}])
    path = tmp_path / 'evaluation/annotations/scene.yaml'
    path.parent.mkdir()
    path.write_text(json.dumps(labels))
    run = tmp_path / 'run'
    (run / 'scene').mkdir(parents=True)
    rows = [frame(0, 'OBSTACLE', 10.0, [{'center': [10.5, 0.0, 0.0], 'distance_m': 10.0}]),
            frame(S, 'OBSTACLE', 30.0, [{'center': [30.5, 2.5, 0.0], 'distance_m': 30.0}])]
    (run / 'scene/frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    result = evaluate_run(run, dataset, splits, tmp_path)
    assert result['total']['negative_object_alarm_frames_on_negative'] == 1
    assert result['total']['object_by_distance'] == [
        {'range_m': [0, 20], 'frames': 1, 'obstacle': 1, 'hits': 1, 'recall': 1.0}]
    text = '\n'.join(quality_lines(result))
    assert '| 0–20 | 1 | 1 | 1 | 100% |' in text
    assert '`side` (above): 1/1 кадров с тревогой на объекте.' in text
    assert 'из 1 FP-кадров на эти объекты приходится 1' in text


def test_working_range_counts_only_reachable_references():
    def reference(stamp, distance):
        return {'bag_stamp_ns': stamp, 'distance_m': distance,
                'person_roi_assumed_m': {'min': [distance, -1.0, -1.0],
                                         'max': [distance + 1.0, 1.0, 1.0]}}
    event = {'id': 'box', 'label': 'positive', 'start_ns': 0, 'end_ns': 2 * S,
             'reference_frames': [reference(0, 15.0), reference(S, 50.0),
                                  reference(2 * S, 150.0)]}
    rows = [frame(0, 'OBSTACLE', 15.0, [{'center': [15.5, 0.0, 0.0], 'distance_m': 15.0}]),
            frame(S, 'OBSTACLE', 5.0, [{'center': [5.0, 3.0, 0.0], 'distance_m': 5.0}]),
            frame(2 * S, 'UNKNOWN')]
    labels = annotation([(0, 2 * S, 'positive')], [event])
    assert quality(rows, labels)['working_range'] is None
    result = quality(rows, labels, working_range_m=120.0)
    assert result['working_range'] == {
        'range_m': 120.0, 'reference_frames': 2, 'beyond_frames': 1, 'obstacle': 2, 'hits': 1,
        'state_recall': 1.0, 'object_recall': 0.5}
    assert result['object']['recall'] == 1 / 3  # the totals still count the far frame


def test_working_range_comes_from_the_run_profile(tmp_path):
    dataset = tmp_path / 'evaluation/dataset.yaml'
    dataset.parent.mkdir()
    dataset.write_text(''.join(f'- id: {name}\n  annotations: evaluation/annotations/a.yaml\n'
                               for name in ('near', 'wide', 'closed')).join(['bags:\n', '']))
    splits = tmp_path / 'evaluation/splits.yaml'
    splits.write_text('regression: [near, wide, closed]\n')
    roi = {'min': [150.0, -1.0, -1.0], 'max': [151.0, 1.0, 1.0]}
    labels = annotation([(0, 0, 'positive')], [
        {'id': 'far', 'label': 'positive', 'start_ns': 0, 'end_ns': 0,
         'reference_frames': [{'bag_stamp_ns': 0, 'distance_m': 150.0,
                               'person_roi_assumed_m': roi}]}])
    (tmp_path / 'evaluation/annotations').mkdir()
    (tmp_path / 'evaluation/annotations/a.yaml').write_text(json.dumps(labels))
    (tmp_path / 'near.yaml').write_text('allow_unverified_calibration: true\n'
                                        'detection_roi:\n  max: [120.0, 5.0, 5.0]\n')
    (tmp_path / 'wide.yaml').write_text('allow_unverified_calibration: true\n')  # no ROI
    # Like full_scan_unresolved.yaml: the pipeline stops for unverified calibration.
    (tmp_path / 'closed.yaml').write_text('calibration_verified: false\n'
                                          'detection_roi:\n  max: [150.0, 150.0, 20.0]\n')
    run = tmp_path / 'run'
    for name in ('near', 'wide', 'closed'):
        (run / name).mkdir(parents=True)
        (run / name / 'frames.jsonl').write_text(json.dumps(frame(0, 'UNKNOWN')) + '\n')
    names = ('near', 'wide', 'closed')
    manifest = {'config_sha256': {f'{n}.yaml': sha256(tmp_path / f'{n}.yaml') for n in names},
                'bags': [{'id': n, 'sensor_profile': f'{n}.yaml'} for n in names]}
    (run / 'manifest.json').write_text(json.dumps(manifest))
    # An older manifest without recorded ranges: the profiles still have the run hashes.
    result = evaluate_run(run, dataset, splits, tmp_path)
    assert result['bags']['near']['quality']['working_range']['beyond_frames'] == 1
    assert result['bags']['wide']['quality']['working_range'] is None
    assert result['bags']['closed']['quality']['working_range'] is None
    total = result['total']['working_range']
    assert total['range_m'] == [120.0] and total['bags_without_range'] == 2
    assert total['reference_frames'] == 0 and total['state_recall'] is None
    assert '| **всего** | 120 | 0 (1) | 0 | 0 | N/A | N/A |' in '\n'.join(quality_lines(result))
    wider = evaluate_run(run, dataset, splits, tmp_path, working_range_m=200.0)
    assert wider['total']['working_range']['reference_frames'] == 3
    # The profile changes after the run: its range no longer describes the run.
    (tmp_path / 'near.yaml').write_text('allow_unverified_calibration: true\n'
                                        'detection_roi:\n  max: [200.0, 5.0, 5.0]\n')
    changed = evaluate_run(run, dataset, splits, tmp_path)
    assert changed['bags']['near']['quality']['working_range'] is None
    assert 'изменился после прогона' in '\n'.join(quality_lines(changed))
    # A range recorded at run time stands whatever the profile says now.
    manifest['bags'][0]['working_range_m'] = 120.0
    (run / 'manifest.json').write_text(json.dumps(manifest))
    recorded = evaluate_run(run, dataset, splits, tmp_path)
    assert recorded['bags']['near']['quality']['working_range']['range_m'] == 120.0


def test_repository_profiles_have_a_working_range_only_when_the_detector_runs():
    root = next((parent for parent in Path(__file__).resolve().parents
                 if (parent / 'metro_perception_ros/config').is_dir()), None)
    if root is None:
        pytest.skip('repository sensor profiles are not available')
    ranges = {name: profile_range(root, path) for name, path in {
        'assumed': 'metro_perception_ros/config/forward_sector_assumed.yaml',
        'research': 'metro_perception_ros/config/full_scan_research_assumed.yaml',
        'unresolved': 'metro_perception_ros/config/full_scan_unresolved.yaml',
        'preview': 'metro_perception_bringup/config/sensors/forward_sector_preview.yaml',
    }.items()}
    assert ranges == {'assumed': 120.0, 'research': 120.0, 'unresolved': None, 'preview': None}
