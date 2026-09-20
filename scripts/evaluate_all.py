#!/usr/bin/env python3
"""Export registered bags into a new output directory and record run provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, default=root / 'evaluation/dataset.yaml')
    parser.add_argument('--image-id', default='not_recorded')
    parser.add_argument('--preview', action='store_true',
                        help='Use unverified lidar-centered visualization profiles (always UNKNOWN)')
    args = parser.parse_args()
    dataset = yaml.safe_load(args.dataset.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=False)
    git = lambda *cmd: subprocess.check_output(['git', '-C', str(root), *cmd], text=True).strip()
    manifest = {'schema_version': 1, 'mode': 'a02', 'image_id': args.image_id,
                'commit': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain')),
                'dataset_sha256': sha256(args.dataset), 'config_sha256': {}, 'bags': [],
                'preview': args.preview,
                'note': 'Static profiles only; mounting remains unverified, no detector/deskew.'}
    for config in sorted(list((root / 'metro_perception_bringup/config').rglob('*.yaml')) +
                         list((root / 'metro_perception_ros/config').rglob('*.yaml'))):
        manifest['config_sha256'][str(config.relative_to(root))] = sha256(config)
    manifest_path = args.output_dir / 'manifest.json'
    try:
        for entry in dataset['bags']:
            bag = args.dataset_root / entry['path']
            profile = (root / 'metro_perception_bringup/config/sensors' /
                       (entry['sensor_profile'] + '_preview.yaml') if args.preview else
                       root / 'metro_perception_ros/config/lidar_only.yaml')
            result_dir = args.output_dir / entry['id']
            result_dir.mkdir()
            info = {'id': entry['id'], 'topic': entry['input_topic'], 'status': 'running',
                    'sensor_profile': str(profile.relative_to(root)),
                    'files': {item.name: sha256(item) for item in sorted(bag.iterdir())
                              if item.is_file()}}
            manifest['bags'].append(info)
            try:
                subprocess.run(['ros2', 'run', 'metro_perception_ros', 'evaluate_bag', str(bag),
                                entry['input_topic'], str(result_dir / 'frames.jsonl'), str(profile)], check=True)
                subprocess.run(['ros2', 'run', 'metro_perception_tools', 'metrics',
                                str(result_dir / 'frames.jsonl'), '--output',
                                str(result_dir / 'summary.json')], check=True)
                info['status'] = 'exported_a02'
            except Exception:
                info['status'] = 'failed'
                raise
    finally:
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
