#!/usr/bin/env python3
"""
Run evaluate_bag with sensor-profile overrides and report development and validation apart.

Overrides are KEY=VALUE with KEY either a detector parameter (`static_max_length_m=4`) or
`section.key` (`temporal.confirm_hits=3`); they are applied to both research profiles
(forward_sector_assumed and full_scan_research_assumed). Every annotated real bag of
evaluation/dataset.yaml and the development synthetic set are run; --val adds the validation
synthetic set. Results go to OUT_ROOT/NAME, which must not exist. For a quick look only: clean
runs for reporting go through scripts/evaluate_in_container.sh.

  sweep_profile.py NAME [KEY=VALUE ...] [--val] [--install /tmp/i] [--jobs 8]
"""
import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


metrics = load('metrics', ROOT / 'metro_perception_tools/metro_perception_tools/metrics.py')
inject = load('inject_obstacle', ROOT / 'scripts/inject_obstacle.py')
PROFILES = {'forward_sector': 'forward_sector_assumed.yaml',
            'full_scan': 'full_scan_research_assumed.yaml'}
SHOWN_BINS = ('0-20', '20-40', '40-60', '60-80', '80-100')


def apply_overrides(profile, overrides):
    """Return a copy of a profile with KEY=VALUE overrides (detector section by default)."""
    result = json.loads(json.dumps(profile))
    for item in overrides:
        key, sep, value = item.partition('=')
        if not sep or not key:
            raise ValueError(f'Override is not KEY=VALUE: {item}')
        section, _, name = key.rpartition('.')
        result.setdefault(section or 'detector', {})[name] = yaml.safe_load(value)
    return result


def jobs_for(dataset, data_root, out_dir, profiles):
    """(bag, topic, frames.jsonl, profile) for every annotated bag of a dataset."""
    jobs = []
    for entry in dataset['bags']:
        if not entry.get('annotations'):
            continue
        frames = out_dir / entry['id'] / 'frames.jsonl'
        jobs.append((Path(data_root) / entry['path'], entry['input_topic'], frames,
                     profiles[entry['sensor_profile']]))
    return jobs


def run_job(job, install):
    bag, topic, frames, profile = job
    frames.parent.mkdir(parents=True)
    command = ['ros2', 'run', 'metro_perception_ros', 'evaluate_bag',
               str(bag), topic, str(frames), str(profile)]
    if install:
        command = ['bash', '-c', 'source "$0/setup.bash" && exec "$@"', str(install), *command]
    with frames.with_suffix('.log').open('w') as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)


def half_summary(run):
    """Per split: FP frames/events, object hits and first-confirmation delay."""
    lines = []
    for split, total in run['by_split'].items():
        delays = [event['first_matched_delay_s'] for bag in run['bags'].values()
                  if bag['split'] == split for event in bag['quality']['events']['details']]
        text = (f"{split:12s} FP {total['fp']}/{total['negative_frames']} frames, "
                f"{total['fp_events']} events ({total['fp_events_per_min']:.2f}/min)")
        if total['object_reference_frames']:
            text += f", object {total['object_hits']}/{total['object_reference_frames']}"
        if delays:
            shown = ', '.join('none' if d is None else f'{d:.2f}' for d in delays)
            text += f', first confirmation {shown} s'
        lines.append(text)
    for bag_id, bag in run['bags'].items():
        q = bag['quality']
        lines.append(f"  {bag_id:36s} {bag['split'][:3]} fp={q['frame']['fp']:4d} "
                     f"events={q['false_alarms']['events']:3d} "
                     f"p95={bag['processing_ms']['p95']:.0f} ms")
    return lines


def synthetic_summary(name, report):
    """Recall per distance bin and distance of the first confirmation, per scenario."""
    bins = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    first = defaultdict(list)
    for bag, value in report.items():
        scenario = bag.rsplit('-', 1)[-1]
        distance = value['first_hit_distance_m']
        first[scenario].append(None if distance is None else round(distance))
        for key, (recall, count) in value['recall_by_distance'].items():
            if count:
                bins[scenario][key][0] += round((recall or 0) * count)
                bins[scenario][key][1] += count
    lines = []
    for scenario in sorted(bins):
        cells = ' '.join(f'{key}:{hit}/{count}' for key, (hit, count) in bins[scenario].items()
                         if key in SHOWN_BINS)
        lines.append(f'  [{name}] {scenario:8s} first={first[scenario]} {cells}')
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('name')
    parser.add_argument('overrides', nargs='*', help='KEY=VALUE or section.KEY=VALUE')
    parser.add_argument('--val', action='store_true', help='add the validation synthetic set')
    parser.add_argument('--install', type=Path, help='colcon install base to source')
    parser.add_argument('--jobs', type=int, default=8)
    parser.add_argument('--out-root', type=Path, default=Path('/results/sweep'))
    parser.add_argument('--dataset-root', type=Path, default=Path('/data'))
    parser.add_argument('--synthetic-dev', type=Path, default=Path('/results/synthetic-dev'))
    parser.add_argument('--synthetic', type=Path, default=Path('/results/synthetic'))
    args = parser.parse_args()
    out = args.out_root / args.name
    (out / 'profiles').mkdir(parents=True)
    profiles = {}
    for sensor, file_name in PROFILES.items():
        source = yaml.safe_load((ROOT / 'metro_perception_ros/config' / file_name).read_text())
        profiles[sensor] = out / 'profiles' / file_name
        profiles[sensor].write_text(yaml.safe_dump(apply_overrides(source, args.overrides)))
    dataset_path = ROOT / 'evaluation/dataset.yaml'
    jobs = jobs_for(yaml.safe_load(dataset_path.read_text()), args.dataset_root,
                    out / 'real', profiles)
    synthetic = [args.synthetic_dev] + ([args.synthetic] if args.val else [])
    for root in synthetic:
        jobs += jobs_for(yaml.safe_load((root / 'dataset.yaml').read_text()), root,
                         out / root.name, profiles)
    with ThreadPoolExecutor(args.jobs) as pool:
        list(pool.map(lambda job: run_job(job, args.install), jobs))

    run = metrics.evaluate_run(out / 'real', dataset_path, ROOT / 'evaluation/splits.yaml', ROOT)
    reports = {root.name: inject.recall_report(out / root.name, root) for root in synthetic}
    (out / 'summary.json').write_text(json.dumps(
        {'overrides': args.overrides, 'by_split': run['by_split'],
         'bags': {k: {'split': v['split'], 'quality': v['quality'],
                      'processing_ms': v['processing_ms']} for k, v in run['bags'].items()},
         'synthetic': reports}, indent=2) + '\n')
    lines = [f"== {args.name} {' '.join(args.overrides)}"] + half_summary(run)
    for name, report in reports.items():
        lines += synthetic_summary(name, report)
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
