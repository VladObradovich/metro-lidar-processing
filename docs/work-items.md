# Очередь работ

**Актуальный режим по умолчанию:** [lidar-only-default.md](lidar-only-default.md).
Для проверки приватных bag измеренная калибровка не требуется.

| Пункт | Следующий результат | Файлы / место | Проверка |
|---|---|---|---|
| D02, T1 | Закрыт: аудит всех кадров шести bag, единый корень `/data`, SHA-256 | scripts/audit_bags.py, docs/dataset-passport.md | повторный audit_bags.py, sha256sum -c |
| D03, A/T1 | Закрыт как ASSUMED: калибровки и габарита от организаторов нет; уточнение — только при их получении | bringup/config/sensors, vehicle.yaml | несколько сцен обоих входов |
| I01, A/B/C | Утвердить черновой API | core/include, interfaces/msg | сборка всех потребителей |
| A02 | Реализовано: очистка/TF/два ROI/raw indices; реальные extrinsics не подтверждены | core/pipeline, ros/preprocessing | core/TF tests, smoke_a02, все 7 bag; см. calibration.md |
| R01, B | Реализовано: latest-only worker 1+1, QoS, overwrite/max-age/session guards; lifecycle оставлен P1 | ros/perception_node, ros/test | overwrite, reset, overload |
| A03–A04, A | G1 закрыт: RANSAC + МНК, дальность по непрерывной опоре, устойчивость к кратковременной потере пола; криволинейный коридор — P2 | core/src/detector.cpp | real positive/negative, coverage, ложные тревоги |
| A05–A07, A | G2 закрыт по алгоритмическому критерию: lidar-одометрия, bbox, две реальные 3D-разметки человека с проверенной дальностью в предполагаемой системе; физическая привязка и полнота обнаружения остаются открыты | core/src/detector.cpp, evaluation/annotations/doubleT_obstacle.yaml | core-тесты, smoke_ego_motion.py, verify_g2_positive.py |
| R02, B | Базовые переходы состояний реализованы; temporal confirmation и retirement source ещё нет | core/temporal_monitor, ros/monitor | ассоциация, late result, stop/restart |
| R04, C | Статус и bbox есть; коридор/nearest point ещё не отображаются | ros/visualizer_node | stale UI, RViz и namespace |
| E01, B | Общий B0-контракт: exact-stamp TF, replay `/tf` + `/tf_static`, кандидаты в JSONL; realtime queue/wait/watchdog остаются online-only | ros/preprocessing, ros/evaluate_bag | synthetic online/offline parity, real replay |
| D04/E02, T1 | D2 закрыт: positive/uncertain/negative в doubleT_obstacle, пять отрицательных bag целиком, splits без независимого holdout; остаются matching, TP/FP/FN и distance error | evaluation, tools/metrics.py, docs/doubleT-platform-false-alarms.md | UNKNOWN не скрывает FN |
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
