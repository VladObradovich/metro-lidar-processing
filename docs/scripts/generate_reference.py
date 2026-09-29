#!/usr/bin/env python3
"""Generate interface/config tables without importing ROS or the detector."""

import html
import json
import re
from pathlib import Path

import yaml
from bs4 import BeautifulSoup

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / 'docs'


def code(value):
    return '<code>' + html.escape(str(value)).replace('_', '_<wbr>') + '</code>'


def defaults(struct):
    text = (ROOT / 'metro_perception_core/include/metro_perception_core/config.hpp').read_text()
    block = text.split('struct ' + struct, 1)[1].split('void validate()', 1)[0]
    values = {}
    for declaration in re.findall(r'(?:std::size_t|std::uint32_t|double|bool)\s+([^;]+);', block):
        values.update(re.findall(r'(\w+)\{([^}]+)\}', declaration))
    return values


def write_changed(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text() != text:
        path.write_text(text)


def generate(language='ru'):
    descriptions = yaml.safe_load((DOCS / 'data/schema-descriptions.yaml').read_text())
    layouts = json.loads((DOCS / 'data/reference-layout.json').read_text())
    profile = yaml.safe_load(
        (ROOT / 'metro_perception_ros/config/forward_sector_assumed.yaml').read_text()
    )
    algorithm, temporal = defaults('AlgorithmConfig'), defaults('TemporalConfig')
    if set(algorithm) | set(temporal) != set(descriptions['parameters']):
        raise ValueError(
            'Config fields changed: update schema-descriptions.yaml and parameter groups'
        )
    if {key for _, _, keys in descriptions['groups'] for key in keys} != set(algorithm):
        raise ValueError('Parameter groups do not cover AlgorithmConfig')
    if not set(profile['detector']) <= set(algorithm) or not set(profile['temporal']) <= set(
        temporal
    ):
        raise ValueError('Unknown config fields in the sensor profile')
    summary = (
        f'<p class="lead">{len(algorithm)} полей AlgorithmConfig и '
        f'{len(temporal)} полей TemporalConfig, значения C++ и штатного профиля.</p>\n'
    )
    write_changed(DOCS / '_generated' / language / 'parameter-summary.html', summary)
    for filename, layout in layouts.items():
        soup = BeautifulSoup(layout, 'html.parser')
        rows = []
        if filename.startswith('parameters-'):
            group = int(filename.removesuffix('.html').split('-')[-1])
            keys = descriptions['groups'][group - 1][2] if group <= 8 else list(temporal)
            values = algorithm if group <= 8 else temporal
            overrides = (
                dict(profile['detector'], blind_radius_m=profile['blind_radius_m'])
                if group <= 8
                else profile['temporal']
            )
            for key in keys:
                override = overrides.get(key, values[key])
                if isinstance(override, bool):
                    override = str(override).lower()
                rows.append(
                    [code(key), code(values[key]), code(override), descriptions['parameters'][key]]
                )
        else:
            # The layout records the exact message type in its source path.
            name = re.search(r'msg/(\w+)\.msg', soup.get_text()).group(1)
            constants = []
            for line in (
                (ROOT / 'metro_perception_interfaces/msg' / (name + '.msg'))
                .read_text()
                .splitlines()
            ):
                line = line.split('#', 1)[0].strip()
                if not line:
                    continue
                typ, field = line.split(None, 1)
                if '=' in field:
                    key, value = field.split('=', 1)
                    constants.append(code(key) + '=' + html.escape(value))
                else:
                    if field not in descriptions['messages']:
                        raise ValueError(
                            'Missing semantic description for message field: ' + field
                        )
                    rows.append([code(field), code(typ), descriptions['messages'][field]])
            existing = soup.select_one('.message-constants')
            if existing:
                existing.decompose()
            if constants:
                node = BeautifulSoup(
                    '<p class="message-constants">Константы: ' + '; '.join(constants) + '.</p>',
                    'html.parser',
                ).p
                soup.select_one('.table-caption').insert_before(node)
        soup.tbody.clear()
        for row in rows:
            soup.tbody.append(
                BeautifulSoup(
                    '<tr>' + ''.join('<td>' + value + '</td>' for value in row) + '</tr>',
                    'html.parser',
                ).tr
            )
        write_changed(DOCS / '_generated' / language / filename, str(soup) + '\n')
    return {
        'algorithm_fields': len(algorithm),
        'temporal_fields': len(temporal),
        'generated_tables': len(layouts),
    }


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--language', default='ru')
    print(json.dumps(generate(parser.parse_args().language), ensure_ascii=False))
