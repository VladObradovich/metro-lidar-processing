#!/usr/bin/env python3
"""Export registered bags into a new output directory and record run provenance."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def executable_hashes():
    """Hash the installed evaluate_bag and the project libraries it loads."""
    files = {}
    for package, relative in (
            ('metro_perception_ros', 'lib/metro_perception_ros/evaluate_bag'),
            ('metro_perception_ros', 'lib/libpointcloud_adapter.so'),
            ('metro_perception_core', 'lib/libmetro_perception_core.so')):
        prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', package], text=True).strip()
        path = (Path(prefix) / relative).resolve()
        files[f'{package}/{relative}'] = {'path': str(path), 'sha256': sha256(path)}
    return files


def selected_bags(dataset, requested):
    """Select registered bag ids, regardless of annotation availability."""
    registered = [entry['id'] for entry in dataset['bags']]
    unknown = set(requested or []) - set(registered)
    if unknown:
        raise ValueError('Unknown bag ids: ' + ', '.join(sorted(unknown)))
    return [bag for bag in registered if not requested or bag in requested]


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, default=root / 'evaluation/dataset.yaml')
    parser.add_argument('--image-id', default='not_recorded')
    parser.add_argument('--build-info', default='not_recorded',
                        help='How the installed packages were built (e.g. colcon command)')
    parser.add_argument('--tf-lookahead-s', type=float, default=0.05)
    parser.add_argument('--max-points', type=int, default=2000000)
    parser.add_argument('--max-cloud-bytes', type=int, default=256 * 1024 * 1024)
    parser.add_argument(
        '--preview',
        action='store_true',
        help='Use unverified lidar-centered preview profiles (clear path remains UNKNOWN)',
    )
    parser.add_argument('--bags', nargs='+', help='Bag ids to run (default: all registered)')
    parser.add_argument(
        '--full-scan-research',
        action='store_true',
        help='Score full_scan bags with the unverified research forward axis (-sensor Y)',
    )
    args = parser.parse_args()
    if args.preview and args.full_scan_research:
        parser.error('--preview and --full-scan-research are mutually exclusive')
    if not 1 <= args.max_points <= 10000000:
        parser.error('--max-points must be in [1, 10000000]')
    if not 1 <= args.max_cloud_bytes <= 1024 * 1024 * 1024:
        parser.error('--max-cloud-bytes must be in [1, 1073741824]')
    if not math.isfinite(args.tf_lookahead_s) or not 0 <= args.tf_lookahead_s <= 1:
        parser.error('--tf-lookahead-s must be in [0, 1]')
    dataset = yaml.safe_load(args.dataset.read_text())
    try:
        selected = selected_bags(dataset, args.bags)
    except ValueError as exc:
        parser.error(str(exc))
    args.output_dir.mkdir(parents=True, exist_ok=False)

    def git(*cmd):
        return subprocess.check_output(
            ['git', '-C', str(root), *cmd],
            text=True,
        ).strip()
    manifest = {'schema_version': 1, 'mode': 'geometric_rolling', 'image_id': args.image_id,
                'commit': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain')),
                'dataset_sha256': sha256(args.dataset), 'config_sha256': {},
                'scoring_sha256': {}, 'scoring_executed_from': 'source_tree', 'bags': [],
                'preview': args.preview, 'full_scan_research': args.full_scan_research,
                'max_points': args.max_points,
                'max_cloud_bytes': args.max_cloud_bytes,
                'tf_lookahead_s': args.tf_lookahead_s, 'selected_bags': args.bags,
                'build_info': args.build_info, 'executable_sha256': executable_hashes(),
                'note': (
                    'Profiles are selected by sensor_profile metadata; '
                    'full-scan orientation remains unresolved; experimental detector, no deskew.'
                )}
    for config in sorted(list((root / 'metro_perception_bringup/config').rglob('*.yaml')) +
                         list((root / 'metro_perception_ros/config').rglob('*.yaml'))):
        manifest['config_sha256'][str(config.relative_to(root))] = sha256(config)
    score_files = [root / 'evaluation/splits.yaml',
                   root / 'metro_perception_tools/metro_perception_tools/metrics.py',
                   root / 'metro_perception_tools/metro_perception_tools/report.py']
    score_files += [root / entry['annotations'] for entry in dataset['bags']
                    if entry.get('annotations')]

    def display(path):
        try:
            return str(path.relative_to(root))
        except ValueError:  # Annotations of synthetic data live outside the repository.
            return str(path)
    manifest['scoring_sha256'] = {display(path): sha256(path) for path in score_files}
    manifest_path = args.output_dir / 'manifest.json'
    manifest['selected_bags'] = selected
    metrics_script = root / 'metro_perception_tools/metro_perception_tools/metrics.py'
    report_script = root / 'metro_perception_tools/metro_perception_tools/report.py'
    try:
        for entry in dataset['bags']:
            if entry['id'] not in manifest['selected_bags']:
                continue
            bag = args.dataset_root / entry['path']
            if args.preview:
                profile = (root / 'metro_perception_bringup/config/sensors' /
                           (entry['sensor_profile'] + '_preview.yaml'))
            else:
                profiles = {
                    'forward_sector': (
                        root / 'metro_perception_ros/config/forward_sector_assumed.yaml'
                    ),
                    'full_scan': root / 'metro_perception_ros/config' / (
                        'full_scan_research_assumed.yaml' if args.full_scan_research
                        else 'full_scan_unresolved.yaml'),
                }
                try:
                    profile = profiles[entry['sensor_profile']]
                except KeyError as exc:
                    raise ValueError(f"Unknown sensor_profile: {entry['sensor_profile']}") from exc
            result_dir = args.output_dir / entry['id']
            result_dir.mkdir()
            info = {'id': entry['id'], 'topic': entry['input_topic'], 'status': 'running',
                    'sensor_profile': str(profile.relative_to(root)),
                    'files': {item.name: sha256(item) for item in sorted(bag.iterdir())
                              if item.is_file()}}
            manifest['bags'].append(info)
            try:
                subprocess.run(
                    [
                        'ros2', 'run', 'metro_perception_ros', 'evaluate_bag', str(bag),
                        entry['input_topic'], str(result_dir / 'frames.jsonl'), str(profile),
                        str(args.max_points), str(args.max_cloud_bytes), str(args.tf_lookahead_s),
                    ],
                    check=True,
                )
                subprocess.run([sys.executable, str(metrics_script),
                                str(result_dir / 'frames.jsonl'), '--output',
                                str(result_dir / 'summary.json')], check=True)
                info['frames_sha256'] = sha256(result_dir / 'frames.jsonl')
                info['status'] = 'exported'
            except Exception:
                info['status'] = 'failed'
                raise
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        subprocess.run([sys.executable, str(metrics_script),
                        str(args.output_dir), '--root', str(root),
                        '--dataset', str(args.dataset), '--output',
                        str(args.output_dir / 'quality.json')], check=True)
        subprocess.run([sys.executable, str(report_script),
                        str(args.output_dir / 'quality.json'), '--output',
                        str(args.output_dir / 'quality.md')], check=True)
    finally:
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
