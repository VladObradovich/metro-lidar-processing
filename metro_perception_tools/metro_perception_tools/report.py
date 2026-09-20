"""Render a runtime summary as Markdown without inventing detection metrics."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.summary.read_text())
    lines = ['# Результат прогона', '',
             'Режим: ' + ('каркас, детектор отсутствует' if data['scaffold'] else 'обработка'),
             '', f"Кадров: {data['frames']}",
             f"UNKNOWN: {data['unknown_fraction']:.1%}", '',
             '| Задержка обработки | мс |', '|---|---:|']
    lines += [f'| {key} | {value:.3f} |' for key, value in data['processing_ms'].items()]
    lines += ['', 'Качество обнаружения (TP/FP/FN, ошибка расстояния) не оценено.',
              'Время каркаса не является производительностью будущего детектора.', '']
    with args.output.open('x') as output:
        output.write('\n'.join(lines))
