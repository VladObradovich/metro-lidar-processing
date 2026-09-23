"""Render a runtime or quality summary as Markdown without inventing detection metrics."""
import argparse
import json
from pathlib import Path


def fmt(value, spec='.3f', suffix=''):
    return 'N/A' if value is None else f'{value:{spec}}{suffix}'


def runtime_lines(data):
    lines = ['# Результат прогона', '',
             'Режим: ' + ('каркас, детектор отсутствует' if data['scaffold'] else 'обработка'),
             '', f"Кадров: {data['frames']}",
             f"UNKNOWN: {data['unknown_fraction']:.1%}", '',
             '| Задержка обработки | мс |', '|---|---:|']
    lines += [f'| {key} | {value:.3f} |' for key, value in data['processing_ms'].items()]
    lines += ['', 'Качество обнаружения (TP/FP/FN, ошибка расстояния) не оценено.',
              'Время каркаса не является производительностью будущего детектора.', '']
    return lines


def quality_lines(data):
    provenance = data.get('provenance') or {}
    commit = provenance.get('commit') or 'не записан'
    if provenance.get('dirty'):
        commit += ' (dirty)'
    lines = ['# Качество обнаружения', '',
             f'Коммит: `{commit}`. Прогон: `{data["run_dir"]}`.',
             'Прогон детектора воспроизводим: '
             + ('да' if data.get('run_reproducible') else 'не подтверждено') + '.',
             'Scorer в manifest совпадает с текущими исходниками: '
             + ('да' if data.get('scoring_current') else 'не подтверждено') + '.',
             *(['Проблемы прогона: ' + '; '.join(data['run_issues']) + '.']
               if data.get('run_issues') else []),
             *(['Проблемы оценки: ' + '; '.join(data['scoring_issues']) + '.']
               if data.get('scoring_issues') else []),
             'Калибровка ASSUMED: дальности в предполагаемой системе лидара.'
             + (' Full-scan bag оценены исследовательским профилем (forward = −Y датчика).'
                if provenance.get('full_scan_research') else ''), '',
             'Кадровые TP/recall/precision и «событие обнаружено» по состоянию кадра '
             'являются **верхней оценкой**, FN — нижней: тревога могла относиться '
             'к другому объекту. '
             'Объектный уровень проверяется только на кадрах с размеченной областью человека.',
             '', '## По split', '',
             '| Split | Кадры +/−/? | TP | FN (UNKNOWN) | FP | Recall кадров ≤ | '
             'Precision кадров ≤ | FP-кадров | Событий ≤ | FP-событий/мин | Объект: попаданий |',
             '|---|---|---:|---:|---:|---:|---:|---:|---|---:|---|']
    if not data['bags']:
        lines += ['', 'В прогоне нет записей с разметкой: метрики качества N/A.', '']
    for name, s in list(data['by_split'].items()) + [('**всего**', data['total'])]:
        lines.append(
            f"| {name} | {s['positive_frames']}/{s['negative_frames']}/{s['uncertain_frames']} "
            f"| {s['tp']} | {s['fn']} ({s['unknown_on_positive']}) | {s['fp']} "
            f"| {fmt(s['frame_recall'], '.1%')} | {fmt(s['frame_precision'], '.1%')} "
            f"| {fmt(s['false_alarm_rate'], '.1%')} "
            f"| {s['detected_events']}/{s['positive_events']} "
            f"| {fmt(s['fp_events_per_min'], '.2f')} "
            f"| {s['object_hits']}/{s['object_reference_frames']} |")
    lines += ['', '## По bag', '',
              '| Bag | Split | Кадров (потеряно) | TP/FN | FP-кадров | FP-событий | '
              'FP-событий/мин | Геометрия | Медиана дальности оценки, м | p95, мс |',
              '|---|---|---|---|---|---:|---:|---:|---:|---:|']
    for bag, b in data['bags'].items():
        q = b['quality']
        scored = q['coverage_scored']
        lines.append(
            f"| {bag} | {b['split']} | {b['frames']['processed']} ({b['frames']['lost']}) "
            f"| {q['frame']['tp']}/{q['frame']['fn']} "
            f"| {q['frame']['fp']}/{q['frames_by_label']['negative']} "
            f"| {q['false_alarms']['events']} | {fmt(q['false_alarms']['events_per_min'], '.2f')} "
            f"| {fmt(scored['observed'], '.0%')} "
            f"| {fmt(scored['median_evaluated_range_m'], '.1f')} "
            f"| {b['processing_ms']['p95']:.1f} |")
    for bag, b in data['bags'].items():
        q = b['quality']
        if not q['events']['details'] and not q['object']['checks']:
            continue
        lines += ['', f'## {bag}: событие, объект и дальность', '']
        for event in q['events']['details']:
            lines.append(
                f"- `{event['id']}`: по состоянию "
                f"{'обнаружено' if event['detected'] else 'пропущено'}, "
                f"{event['obstacle_frames']}/{event['frames']} кадров с тревогой, первая тревога "
                f"через {fmt(event['first_alarm_delay_s'], '.2f', ' с')} на "
                f"{fmt(event['first_alarm_distance_m'], '.2f', ' м')}; объект "
                f"{'подтверждён' if event['object_confirmed'] else 'не подтверждён'}, первый "
                'сопоставленный кандидат среди размеченных кадров через '
                f"{fmt(event['first_matched_delay_s'], '.2f', ' с')} на "
                f"{fmt(event['first_matched_distance_m'], '.2f', ' м')}.")
        lines.append(
            f"- Область объекта размечена на {q['object']['reference_frames']} "
            f"из {q['object']['positive_frames']} положительных кадров."
        )
        names = {'hit': 'попадание', 'miss': 'пропуск',
                 'wrong_object': 'тревога на другом объекте', 'no_frame': 'нет кадра'}
        for check in q['object']['checks']:
            line = f"- кадр {check['bag_stamp_ns']}: {names[check['result']]}"
            if check['reference_m'] is not None:
                line += (f"; эталон {check['reference_m']:.2f} м, выдано "
                         f"{fmt(check['reported_m'], '.2f', ' м')}, ошибка "
                         f"{fmt(check['error_m'], '+.2f', ' м')}")
            lines.append(line)
    if data.get('skipped'):
        lines += ['', 'Не оценены: ' + ', '.join(
            f'{bag} ({reason})' for bag, reason in data['skipped'].items()) + '.']
    lines += ['', 'FP считаются только на отрицательных кадрах; геометрия и дальность — '
              'по всем размеченным кадрам bag.',
              'Геометрия: дальность оценки > 0.',
              'Объектная доля относится только к кадрам с размеченной областью; '
              'доля для всего положительного интервала не определена.',
              'Правила сопоставления — docs/evaluation-metrics.md.', '']
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.summary.read_text())
    lines = quality_lines(data) if 'bags' in data else runtime_lines(data)
    with args.output.open('x') as output:
        output.write('\n'.join(lines))


if __name__ == '__main__':
    main()
