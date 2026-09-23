# Очередь работ

**Актуальный режим по умолчанию:** [lidar-only-default.md](lidar-only-default.md).
Для проверки приватных bag измеренная калибровка не требуется.

| Пункт | Следующий результат | Файлы / место | Проверка |
|---|---|---|---|
| D02, T1 | Полный аудит всех кадров, а не первых samples | tools/inspect_bag.py | timestamps, смена схемы, числа точек |
| D03, A/T1 | Опциональное уточнение осей/монтажа; lidar-only default уже доступен | bringup/config/sensors, vehicle.yaml | несколько сцен обоих входов |
| I01, A/B/C | Утвердить черновой API | core/include, interfaces/msg | сборка всех потребителей |
| A02 | Реализовано: очистка/TF/два ROI/raw indices; реальные extrinsics не подтверждены | core/pipeline, ros/preprocessing | core/TF tests, smoke_a02, все 7 bag; см. calibration.md |
| R01, B | Реализовано: latest-only worker 1+1, QoS, overwrite/max-age/session guards; lifecycle оставлен P1 | ros/perception_node, ros/test | overwrite, reset, overload |
| A03–A04, A | G1 закрыт: RANSAC + МНК, дальность по непрерывной опоре, устойчивость к кратковременной потере пола; криволинейный коридор — P2 | core/src/detector.cpp | real positive/negative, coverage, ложные тревоги |
| A05–A07, A | B0 с угловой кластеризацией, проверкой исходных точек, bbox и расстоянием; пороги требуют измерений | core/src/detector.cpp, pipeline.cpp | real positive, низкий блок, границы габарита |
| R02, B | Базовые переходы состояний реализованы; temporal confirmation и retirement source ещё нет | core/temporal_monitor, ros/monitor | ассоциация, late result, stop/restart |
| R04, C | Статус и bbox есть; коридор/nearest point ещё не отображаются | ros/visualizer_node | stale UI, RViz и namespace |
| E01, B | Общий B0-контракт: exact-stamp TF, replay `/tf` + `/tf_static`, кандидаты в JSONL; realtime queue/wait/watchdog остаются online-only | ros/preprocessing, ros/evaluate_bag | synthetic online/offline parity, real replay |
| D04/E02, T1 | Разметка, matching, TP/FP/FN и distance error | evaluation, tools/metrics.py | UNKNOWN не скрывает FN |
| P01/P02, C/T2 | Runtime и CI с новыми зависимостями | docker, .github | чистая сборка + smoke |
| X01–X03 | Только измеренные улучшения | core, evaluation/experiments.yaml | сравнение с сохранённым B0 |

## Шаблон дефекта

- SHA/config/image:
- Bag и интервал либо synthetic fixture:
- Команда воспроизведения:
- Ожидание:
- Фактический результат и лог:
- Ответственный и регрессионная проверка:

Актуальный календарь и критерии приёмки находятся в [PLAN.md](../PLAN.md).
