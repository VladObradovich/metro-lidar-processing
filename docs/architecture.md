# Архитектура

**Lidar-only policy:** [lidar-only-default.md](lidar-only-default.md).
Forward-sector имеет отдельный ASSUMED-профиль; его source frame bind-ится по первому
валидному PointCloud2 каждой session, не по модели лидара. Full-scan без ориентации
остаётся UNKNOWN.

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
Статический transform из sensor profile используется perception/evaluator локально.
Глобальная публикация в /tf_static принадлежит bringup и выключена в обычном
perception.launch.py; demo.launch.py включает её для RViz через штатный
tf2_ros/static_transform_publisher. Внешний TF онлайн разрешается строго на
header.stamp с bounded ожиданием tf_wait_timeout_s на steady clock; latest TF не
используется как fallback. Это позволяет пережить небольшой порядок доставки
cloud/TF и не зависнуть при паузе /clock. Счётчики A02 публикуются в FrameAnalysis.
`calibration_trust` формируется в core после успешного TF/transform и проходит
через FrameAnalysis, monitor и PathAssessment. `ASSUMED` не эквивалентен
`VERIFIED`: пустой результат при assumed-калибровке не может подтверждать
свободный путь.
PointCloud callback больше не выполняет тяжёлый A02: он кладёт ConstSharedPtr в
latest-only слот. Один worker обрабатывает максимум один кадр, ещё один может
ожидать; новый кадр вытесняет старый pending и увеличивает overwritten_frames.
Перед публикацией проверяются session и max_processing_age_s, поэтому результат
старой сессии или слишком старый результат не становится свежим наблюдением.
queue_age_ms и tf_wait_ms разделяют локальную очередь и ожидание transform.
Reset service, ground/corridor, Detection3DArray и подтверждение ещё не реализованы.
Межкадрового накопления облаков нет.

FrameAnalysis хранит единый результат кадра. Header — время наблюдения;
heartbeat PathAssessment сохраняет исходный stamp/sequence. result_age_ms —
processing age плюс локальное ожидание monitor, а не возраст измерения сенсора.
Steady clock служит watchdog; header stamps — для будущей temporal logic.
Скачок header назад увеличивает session в текущем каркасе; полноценная защита
от запоздалого старого source/reset ещё входит в R02.

Расширять API нужно одновременно с online/offline преобразованиями и тестами.
