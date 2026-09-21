# A02 и записи без TF, odom и IMU

**Актуальный режим по умолчанию:** [lidar-only-default.md](lidar-only-default.md).
Для проверки приватных bag измеренная калибровка не требуется.

## Что установлено по данным

В metadata всех семи записей есть только `sensor_msgs/msg/PointCloud2`.
`/tf`, `/tf_static`, odometry и IMU не записаны. Нельзя восстановить монтажное
смещение относительно поезда из названия frame или заводского correction CSV.
`PLAN.md`, §§ 5.2 и 6.2, требует не считать такой corridor метрически правильным.

| Bag | Кадров по metadata | Topic | Frame первого облака |
|---|---:|---|---|
| doubleT_obstacle | 201 | /sensing/lidar/hesai128/pointcloud | lidar_livox |
| doubleT_platform | 345 | /lidar_points | hesai_lidar |
| roundT_doubleT | 252 | /lidar_points | hesai_lidar |
| roundT_pressureGate_roundT | 268 | /lidar_points | hesai_lidar |
| roundT_squareT_pressureGate_squareT | 545 | /lidar_points | hesai_lidar |
| squareT_platform_squareT_switch | 877 | /lidar_points | hesai_lidar |
| new_data | 11271 | /lidar_points | hesai_lidar |

Первые облака секторных записей содержат 307200 точек, включая пустые возвраты;
1-й и 99-й процентили азимута валидных XYZ — примерно −139° и −41°
(у `new_data`: −134.5° и −44.2°). Это основание для **гипотезы** направления
вперёд вдоль −Y. Это не проверка roll/pitch, высоты над рельсом или переднего
габарита. Полный скан `doubleT_obstacle` содержит 921600 точек; его направление
движения из полного круга не определяется. Поэтому full_scan_preview и
full_scan_unresolved больше не задают фиктивный поворот: облако остаётся в
native frame до появления отдельной оценки направления.

### Что дают приложенные файлы

`Pandar128E3X_v4p5_User_Manual_128-en-251110.pdf`, печатные страницы 11–12
(PDF 15–16), § 1.3: Z — ось вращения, Y — нулевой азимут, начало — оптическое
начало измерений лидара. § 3.1.4, страницы 55–57, описывает построение XYZ
из расстояния, угла канала и поправки момента излучения.

- Angle CSV: 128 каналов, elevation/azimuth. В названии канала 42 есть китайская
  пометка «水平» (горизонтальный); файл читается как GB18030, не UTF-8.
- Firetime CSV: поправки времени излучения для каналов, режимов и состояний угла.
- Эти CSV относятся к преобразованию сырых пакетов в облако. Повторное применение
  их к готовым XYZ не добавляет TF и может исказить уже откалиброванные координаты.
  Нет доказательства, какие поправки использовал записывавший драйвер.

Исходники не копируются в образ. SHA-256 для идентификации полученных файлов:

```text
manual:   dbbc1fc023cb1ef84bf3e462b979de3f101c65693a159bd474bcf94485d263de
angle:    779d6593f5d1e3cadb1014ee55b23c8036a0e05f95a93ddcb68af9ada7e8a6dc
firetime: 3179c91824f48723afa50e0d63dcfc79cd704b9cce93dd327dd7d55c737746f1
```

## Реализованная обработка

Отдельной preprocessor-ноды нет. `core/pipeline.cpp` делает A02 для одного кадра:

1. Проверка лимита/пустого входа и пригодности переданного rigid transform.
2. Удаление NaN/Inf и `(0, 0, 0)` **в исходных координатах**.
3. Ближняя сферическая маска `norm(XYZ) < blind_radius_m` до transform.
4. Поворот/перенос в target frame; `sensor_origin` равен перенесённому началу лидара.
5. Широкая geometry ROI и вложенная detection ROI, границы включены.
6. Каждая geometry-точка хранит `raw_index`: номер точки в исходном row-major
   PointCloud2 (включая отфильтрованные точки). `detection_indices` индексирует
   geometry_points; обратная ссылка на raw сохраняется без voxelization.

Ближняя маска 0.5 м и ROI — начальные параметры, не измеренные габариты поезда
и не обещание дальности детектора. Точки вне detection ROI остаются в geometry ROI
для будущей оценки поверхности/коридора. Геометрия последующих стадий ещё не реализована.

ROS-адаптер запрашивает `target <- source` строго на `header.stamp`.
Нет подстановки latest/identity при ошибке. Нулевой/отрицательный stamp,
неожиданный source frame, отсутствующий, устаревший или будущий TF дают
`TF_UNAVAILABLE`. Lookup неблокирующий; онлайн TF listener работает в своём потоке.
Статический профиль валиден для любого положительного stamp, не требует `/clock`
и восстанавливается в приватном TF buffer perception после reset часов.
Публикация в глобальный `/tf_static` вынесена в bringup: обычный headless launch
её не включает, а demo включает отдельный `tf2_ros/static_transform_publisher`
для RViz. Нельзя одновременно публиковать другой TF для той же пары.

Если transform применён, FrameAnalysis получает target frame и исходный stamp.
Если нет — сохраняется исходный frame. Счётчики показывают число geometry,
detection, invalid, blind и outside-ROI точек. Они не означают валидный corridor.

| Условие | processing_status / reason | PathAssessment |
|---|---|---|
| TF недоступен | TF_UNAVAILABLE / TF_UNAVAILABLE | UNKNOWN |
| TF есть, монтаж не подтверждён | INVALID_GEOMETRY / CALIBRATION_UNVERIFIED | UNKNOWN |
| Матрица/геометрия невалидна | INVALID_GEOMETRY / INVALID_TRANSFORM, EMPTY_GEOMETRY_ROI и др. | UNKNOWN |
| A02 пройден с подтверждённой калибровкой | NOT_IMPLEMENTED | UNKNOWN: A03–A07 ещё нет |
| Вход остановлен | INPUT_PAUSED_OR_STOPPED в monitor | UNKNOWN, stale=true |

Конфигурационные ошибки (нечисловые/null компоненты частичного transform,
некорректный ROI и т. п.) завершают запуск с понятной ошибкой, а не включают fallback.

## Запуск для просмотра имеющихся bag

После сборки пакетов и `source install/setup.bash` (либо в пересобранном образе):

```bash
profile="$(ros2 pkg prefix metro_perception_bringup)/share/metro_perception_bringup/config/sensors/forward_sector_preview.yaml"
ros2 launch metro_perception_bringup demo.launch.py \
  sensor_profile:="$profile" fixed_frame:=lidar_preview \
  input_topic:=/lidar_points use_sim_time:=true
# Во втором терминале с тем же ROS_DOMAIN_ID:
ros2 bag play /data/new_data --clock
# В третьем:
ros2 topic echo /metro/analysis
```

Для headless заменить `demo.launch.py` на `perception.launch.py` и убрать
`fixed_frame` либо передать `rviz:=false` в demo. На хосте заменить `/data/new_data`
на путь к записи, например `rosbags/new_data`.

Для `doubleT_obstacle` выбрать `full_scan_preview.yaml` и
`input_topic:=/sensing/lidar/hesai128/pointcloud`.

Forward-sector preview использует frame **lidar_preview**, начало в оптическом
центре и `Rz(+π/2)` (−Y → +X, +X → +Y, +Z → +Z). Он не выдаётся за
`base_link`. Full-scan preview остаётся в `lidar_livox` без придуманной оси
движения. Без `sensor_profile` runtime использует только explicit
forward-sector default; см. ссылку выше.

## Когда появится монтажная калибровка

Шаблоны `forward_sector.yaml` и `full_scan.yaml` используют `target_frame: base_link`.
Для настоящего base_link зафиксировать X вперёд, Y влево, Z вверх, положение начала
и измерить transform из исходного frame. Заполнить `translation_m` и
`rotation_rpy_rad` (roll, pitch, yaw), указать метод/измерения в `calibration_source`.
Только после независимой проверки можно указать `calibration_verified: true`.
Альтернатива: оставить обе transform-записи null и передавать проверенный TF извне;
онлайн всё равно проверяет source frame и calibration_verified.

Нельзя просто изменить false на true в preview. По имеющимся данным остаются
неизвестными монтажные translation/roll/pitch, связь с передним габаритом и
положение базы поезда. D03 остаётся открытым в этой части.

## Offline и проверка

```bash
ros2 run metro_perception_ros evaluate_bag \
  /data/new_data /lidar_points /results/new-data-a02.jsonl "$profile"
# Все семь записей, новые результаты в ещё не существующий каталог:
python3 scripts/evaluate_all.py --dataset-root /data \
  --output-dir /results/a02-preview --preview
python3 scripts/smoke_perception.py
python3 scripts/smoke_a02.py
```

Evaluator и нода используют один loader статического профиля, resolver и core.
Без профиля evaluator использует только forward-sector assumed default.
`evaluate_all.py` выбирает forward_sector/full_scan профиль по dataset metadata.
Динамические `/tf` из будущих bag пока не воспроизводятся.
Offline лимит точек — 2 000 000; кастомный ROS override `max_points` автоматически
не переносится (оставшаяся часть E01). Прогон проверяет A02, а не качество детекции.

`smoke_a02.py` создаёт свой маленький bag и сравнивает онлайн/offline причины,
статусы и все счётчики. Отдельно проверяет missing TF, UNKNOWN и watchdog
при use_sim_time=true без /clock, доставку /tf_static позднему подписчику и reset
сессии при скачке stamp назад. Core-тесты проверяют порядок очистки,
rotation/translation, raw indices, ROI, неправильную калибровку и отсутствие
накопления. TF-тест проверяет интерполяцию на времени измерения и отказ при
экстраполяции/нулевом stamp. Полный прогон реальных записей — в
[a02-validation.md](a02-validation.md).

## Ограничения движения

Static TF описывает только жёсткую установку сенсора. Он не заменяет odometry,
IMU, deskew внутри скана или компенсацию движения между сканами. Облака между
кадрами не накапливаются; фиктивные odom/map/IMU не публикуются.
Будущая temporal logic может подтверждать кандидатов по measurement stamps,
но не объединять их XYZ как неподвижные точки мира. Реализация подтверждения
остаётся R02, после появления кандидатов A03–A07. UNKNOWN сейчас сохраняется.
