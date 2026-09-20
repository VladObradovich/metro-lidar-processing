# Очередь работ

| Пункт | Следующий результат | Файлы / место | Проверка |
|---|---|---|---|
| D02, T1 | Полный аудит всех кадров, а не первых samples | tools/inspect_bag.py | timestamps, смена схемы, числа точек |
| D03, A/T1 | Проверенные оси, sensor origin и габарит | bringup/config/sensors, vehicle.yaml | несколько сцен обоих входов |
| I01, A/B/C | Утвердить черновой API | core/include, interfaces/msg | сборка всех потребителей |
| A02/R01, B | TF на время измерения, очистка, очередь 1+1 | ros/pointcloud_adapter, perception_node | нули до transform, reset, overload |
| A03, A | Ground с ограничением нормали/высоты | core/ground_estimator.hpp/.cpp | стена, платформа, низкий блок |
| A04, A | Прямой corridor и границы пригодной области | core/corridor_estimator.hpp/.cpp | край объекта пересекает габарит |
| A05–A07, A | Кандидаты, clustering, raw validation, distance | core/clusterer, object_validator, pipeline | реальный positive, sensor origin |
| R02, B | Переходы состояний, retirement source, reset | core/temporal_monitor, ros/monitor | late result, stop/restart, history |
| R04, C | Bbox/corridor/nearest point и удаление markers | ros/visualizer_node | stale UI и namespace |
| E01, B | Общий TF/config/time path online/offline | ros/evaluate_bag | одинаковая входная последовательность |
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

День 2: B0 → день 3: адресные улучшения → день 4: измерения/freeze → день 5: сдача.
