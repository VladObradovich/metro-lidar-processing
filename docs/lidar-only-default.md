# Режим по умолчанию: только лидар

Для приватного rosbag не требуется измеренная монтажная калибровка, TF, odometry
или IMU. Нода и evaluator автоматически загружают один установленный профиль
`metro_perception_ros/config/lidar_only.yaml`, если sensor_profile не указан.
`evaluate_all.py` также использует этот профиль (без `--preview`).

Допущения фиксированы для всех bag, без выбора по названию записи:

- X вперёд = исходная −Y, Y влево = исходная +X, Z вверх = исходная +Z;
- rotation RPY = [0, 0, π/2], translation = [0, 0, 0];
- `base_link` — условная база с началом в оптическом центре лидара;
- расстояние считается от лидара, не от переднего габарита поезда;
- source frame берётся из первого непустого допустимого header.frame_id.
  Изменение frame в последующих сообщениях даёт TF_UNAVAILABLE; для нового
  источника с другим frame нужно перезапустить обработку или задать другой профиль.

Это практический baseline по доступным данным. Он не определяет ориентацию
неизвестного лидара автоматически. Если в приватном bag другие оси или монтаж,
нужно заменить профиль, а не перенастраивать пороги по имени bag.
Поворот/перенос не требуют odometry. Накопления облаков и deskew нет.

`calibration_verified: false` сохраняется честно. Новый параметр
`allow_unverified_calibration: true` разрешает продолжить pipeline с этими
допущениями; в FrameAnalysis и JSONL будет `calibration_assumed: true`.
Отсутствие измеренной калибровки больше не порождает CALIBRATION_UNVERIFIED
в дефолтном режиме. Ошибки входа, TF и пустая geometry ROI всё ещё отклоняются.
Сейчас после A02 возвращается NOT_IMPLEMENTED/UNKNOWN, поскольку A03–A07
(сам детектор) ещё не реализованы; калибровка больше не блокирует их подключение.

## Запуск

После пересборки пакетов/образа и source ROS workspace:

```bash
ros2 launch metro_perception_bringup perception.launch.py \
  input_topic:=/lidar_points use_sim_time:=true
# Во втором терминале:
ros2 bag play /data/private_bag --clock
```

Для RViz вместо perception.launch.py использовать demo.launch.py. Fixed Frame
по умолчанию — base_link. Static TF появляется после первого допустимого облака.
Для другого topic (включая /sensing/lidar/hesai128/pointcloud) достаточно изменить
input_topic; имя frame не нужно угадывать. Topic автоматически не выбирается.

Offline без профиля — те же допущения и та же обработка:

```bash
ros2 run metro_perception_ros evaluate_bag \
  /data/private_bag /lidar_points /results/private.jsonl
python3 scripts/evaluate_all.py --dataset-root /data --output-dir /results/default-run
python3 scripts/smoke_a02.py --default
```

## Точная калибровка и внешние TF остаются опциональными

Можно передать свой YAML через `sensor_profile:=/path/sensor.yaml` или пятым
аргументом evaluate_bag. Существующие preview/strict-профили не изменены.
Если allow_unverified_calibration отсутствует, используется строгий режим:
неподтверждённая калибровка блокирует последующие стадии как раньше.

Если известен монтаж, задать реальные translation_m/rotation_rpy_rad и
calibration_verified=true с источником измерений. Для внешнего TF онлайн:
задать конкретные source_frame/target_frame, оставить translation_m и
rotation_rpy_rad null, указать политику принятия калибровки. Тогда static TF
не публикуется, resolver использует внешний TF на header.stamp.
Default не переключается между своим и внешним TF от кадра к кадру: один
источник преобразования на запуск. Динамический TF из bag в offline evaluator
пока не воспроизводится. IMU/odom пока не используются алгоритмом.

Проверка: default и strict online/offline smoke, поздний подписчик /tf_static,
сброс сессии, отказ при смене source frame; тесты core и ROS resolver.
Исторический [отчёт A02](a02-validation.md) относится к строгому preview,
до введения этого рабочего режима по умолчанию.

## Результат проверки 21.09.2026

Сборка Humble и 14 C++/ROS-тестов прошли; scripts/test — 13 passed.
В установленном runtime без исходников прошли default и strict online/offline smoke,
проверки позднего подписчика /tf_static, смены frame, reset и watchdog.
Без аргумента профиля повторно обработаны все 345 кадров doubleT_platform
и 201 кадр doubleT_obstacle: transform_applied=true, calibration_assumed=true,
calibration_verified=false, reason=NOT_IMPLEMENTED. Блокировки калибровкой нет.
Это не проверка готовности детектора; остальные bag повторно в этом режиме не прогонялись.
Логи и JSONL: results/lidar-only-default-20260921/ (не в Git).
