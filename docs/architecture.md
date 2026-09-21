# Архитектура

**Lidar-only policy:** [lidar-only-default.md](lidar-only-default.md).
Forward-sector имеет отдельный ASSUMED-профиль; full-scan без ориентации остаётся UNKNOWN.

```text
PointCloud2 → perception_node → FrameAnalysis → obstacle_monitor → PathAssessment
                   │                                      │
                   └─ общий адаптер + core                └─ visualizer → RViz
                              ↑
                    evaluate_bag → JSONL → metrics/report
```

`core` содержит собственные C++17-типы, `interfaces` — ROS-сообщения,
`ros` — адаптеры/ноды, `bringup` — launch/config, `tools` — Python-инструменты.
Depth image — отдельный существующий путь визуализации.

Доступны decode, A02 (очистка/TF/ROI/raw indices), UNKNOWN и heartbeat.
Нода меняет frame_id только после применённого transform, сохраняя stamp.
Статический профиль публикуется в /tf_static и используется общим TF resolver
онлайн/offline. Внешний TF онлайн читается строго на header.stamp без ожидания
и без подмены последним transform. Счётчики A02 публикуются в FrameAnalysis.
`calibration_trust` формируется в core после успешного TF/transform и проходит
через FrameAnalysis, monitor и PathAssessment. `ASSUMED` не эквивалентен
`VERIFIED`: пустой результат при assumed-калибровке не может подтверждать
свободный путь.
Тяжёлый worker, reset service, ground/corridor, Detection3DArray и подтверждение
ещё не реализованы. Межкадрового накопления облаков нет.

FrameAnalysis хранит единый результат кадра. Header — время наблюдения;
heartbeat PathAssessment сохраняет исходный stamp/sequence. result_age_ms —
processing age плюс локальное ожидание monitor, а не возраст измерения сенсора.
Steady clock служит watchdog; header stamps — для будущей temporal logic.
Скачок header назад увеличивает session в текущем каркасе; полноценная защита
от запоздалого старого source/reset ещё входит в R02.

Расширять API нужно одновременно с online/offline преобразованиями и тестами.
