#!/usr/bin/env python3
"""Build the site/API/PDF or run the local live documentation server."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

from generate_reference import generate, write_changed
from metadata import load_metadata

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / 'docs'


def run(command, env=None):
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def prepare(language):
    if not (DOCS / language / 'index.md').is_file():
        raise ValueError('No documentation sources for language: ' + language)
    if not shutil.which('doxygen'):
        raise ValueError('Install the system package doxygen, then repeat the command')
    counts = generate(language)
    output = ROOT / 'results/docs' / language
    output.mkdir(parents=True, exist_ok=True)
    config = (
        '\n'.join(
            [
                'PROJECT_NAME = "Metro Perception"',
                'OUTPUT_DIRECTORY = "' + str(output / 'doxygen') + '"',
                'INPUT = "'
                + str(ROOT / 'metro_perception_core/include')
                + '" "'
                + str(ROOT / 'metro_perception_ros/include')
                + '"',
                'FILE_PATTERNS = *.hpp',
                'RECURSIVE = YES',
                'EXTRACT_ALL = YES',
                'EXTRACT_PRIVATE = NO',
                'EXTRACT_STATIC = NO',
                'GENERATE_HTML = NO',
                'GENERATE_LATEX = NO',
                'GENERATE_XML = YES',
                'QUIET = YES',
                'WARN_IF_UNDOCUMENTED = NO',
                'ENABLE_PREPROCESSING = YES',
                'MACRO_EXPANSION = NO',
                'FULL_PATH_NAMES = NO',
            ]
        )
        + '\n'
    )
    write_changed(output / 'Doxyfile', config)
    run(['doxygen', str(output / 'Doxyfile')])
    return output, counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['build', 'serve', 'prepare'])
    parser.add_argument('--language', default='ru', choices=['ru', 'en'])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default='8000', type=int)
    args = parser.parse_args()
    metadata = load_metadata()
    output, counts = prepare(args.language)
    env = dict(os.environ, DOCS_LANG=args.language, DOCS_PRINT='0', DOCS_DATE=metadata['date'])
    sphinx_args = ['-c', str(DOCS), '-W', '--keep-going', '-a', '-E', str(DOCS / args.language)]
    if args.action == 'prepare':
        return
    if args.action == 'serve':
        import shlex

        prebuild = shlex.join(
            [sys.executable, str(Path(__file__).resolve()), 'prepare', '--language', args.language]
        )
        command = [
            sys.executable,
            '-m',
            'sphinx_autobuild',
            '--host',
            args.host,
            '--port',
            str(args.port),
            '--watch',
            str(DOCS),
            '--watch',
            str(ROOT / 'metro_perception_interfaces/msg'),
            '--watch',
            str(ROOT / 'metro_perception_core/include'),
            '--watch',
            str(ROOT / metadata['version_source']),
            '--watch',
            str(ROOT / 'metro_perception_ros/include'),
            '--watch',
            str(ROOT / 'metro_perception_ros/config'),
            '--ignore',
            '*/_generated/*',
            '--ignore',
            '*/__pycache__/*',
            '--pre-build',
            prebuild,
            *sphinx_args,
            str(output / 'html'),
        ]
        try:
            run(command, env)
        except KeyboardInterrupt:
            pass
        return
    run([sys.executable, '-m', 'sphinx', '-b', 'html', *sphinx_args, str(output / 'html')], env)
    env['DOCS_PRINT'] = '1'
    run(
        [sys.executable, '-m', 'sphinx', '-b', 'singlehtml', *sphinx_args, str(output / 'print')],
        env,
    )
    from render_pdf import render

    report = render(output, args.language, metadata)
    report.update(counts)
    report['source_commit'] = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True
    ).strip()
    report['prose_source_commit'] = metadata['prose_source_commit']
    report['document_version'] = metadata['version']
    report['document_date'] = metadata['date']
    inputs = [
        path
        for path in DOCS.rglob('*')
        if path.is_file() and '_generated' not in path.parts and '__pycache__' not in path.parts
    ]
    for directory in [
        'metro_perception_core/include',
        'metro_perception_ros/include',
        'metro_perception_interfaces/msg',
        'metro_perception_ros/config',
    ]:
        inputs.extend(path for path in (ROOT / directory).rglob('*') if path.is_file())
    inputs.append(ROOT / metadata['version_source'])
    report['source_file_sha256'] = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(inputs)
    }
    write_changed(
        output / 'build-report.json', json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    )
    print(
        json.dumps(
            {key: value for key, value in report.items() if key != 'source_file_sha256'},
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == '__main__':
    try:
        main()
    except (ValueError, subprocess.CalledProcessError) as error:
        print('Documentation build failed: ' + str(error), file=sys.stderr)
        sys.exit(1)
