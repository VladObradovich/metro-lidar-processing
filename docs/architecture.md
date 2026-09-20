# Архитектура

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

В каркасе доступны только decode, UNKNOWN и heartbeat. Нода не меняет frame_id
на base_link без transform. Тяжёлый worker, TF, diagnostics, reset service,
геометрия, стандартный Detection3DArray и подтверждение ещё не реализованы.

FrameAnalysis хранит единый результат кадра. Header — время наблюдения;
heartbeat PathAssessment сохраняет исходный stamp/sequence. result_age_ms —
processing age плюс локальное ожидание monitor, а не возраст измерения сенсора.
Steady clock служит watchdog; header stamps — для будущей temporal logic.
Скачок header назад увеличивает session в текущем каркасе; полноценная защита
от запоздалого старого source/reset ещё входит в R02.

Расширять API нужно одновременно с online/offline преобразованиями и тестами.
