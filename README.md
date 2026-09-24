# Metro Perception

Система на ROS 2 Humble обнаруживает препятствия впереди поезда по облакам LiDAR из rosbag. Общий C++ pipeline работает онлайн и в последовательной офлайн-оценке. Результат `/metro/assessment` содержит состояние пути, причину, кандидатов, подтверждённые треки и продольное расстояние до ближайшего подтверждённого препятствия. `OBSTACLE` требует конечной положительной дальности; отсутствие измерения обозначается `distance_valid=false` и NaN. `NO_OBSTACLE_DETECTED` допускается только при проверенной калибровке, пригодной области наблюдения и отсутствии кандидатов. На предоставленных bag калибровка лишь предполагается, поэтому отсутствие препятствия даёт `UNKNOWN`, а не «путь свободен».

На текущем коде `28e8d03` (`g4b`) объект человека совпал с разметкой в 53 из 56 положительных кадров; на пяти отрицательных записях осталось 153 ложных кадра в 12 событиях. Это исследовательский результат на известных записях, не оценка скрытого контроля. Статус задач — в [PLAN.md](PLAN.md).

## Содержание

- [Структура проекта](#структура-проекта)
- [Быстрый старт: образ → bag → результат](#быстрый-старт-образ--bag--результат)
- [Как читать результат](#как-читать-результат)
- [RViz](#rviz)
- [Параметры и профили](#параметры-и-профили)
- [Архитектура](#архитектура)
- [Алгоритм](#алгоритм)
- [Эксперименты и воспроизведение](#эксперименты-и-воспроизведение)
- [Ограничения](#ограничения)
- [Разработка](#разработка)
- [Изображение глубины и рабочие записи](#изображение-глубины-и-рабочие-записи)

## Структура проекта

```text
metro_perception_core/                    C++ ядро без зависимости от ROS
├── include/metro_perception_core/        конфигурация и публичный API
├── src/                                 pipeline, detector, temporal_monitor
└── test/                                проверки геометрии и треков
metro_perception_interfaces/              ROS-типы и их пакет сборки
└── msg/                                 FrameAnalysis, PathAssessment,
                                         ObstacleCandidate/Track, CorridorSegment
metro_perception_ros/                     адаптация ядра к ROS 2
├── src/                                 perception_node, obstacle_monitor_node,
│                                        visualizer_node, evaluate_bag и адаптеры
├── config/                              профили секторного и полного скана
└── test/                                тесты ROS-контрактов, TF и сессий
metro_perception_bringup/                 запуск и отображение
├── launch/                              perception, demo, depth_image
├── config/                              общие algorithm/runtime YAML
└── rviz/                                конфигурации RViz для детектора и глубины
metro_perception_tools/                   Python-инструменты
├── metro_perception_tools/              depth_image, inspect_bag, metrics, report
└── test/                                тесты проекции и метрик
evaluation/                               данные оценки, без самих облаков
├── annotations/                         интервалы и области размеченных объектов
├── dataset.yaml, splits.yaml            список bag, профили и разделение выборок
└── scene_context.yaml                   описание сцен и допущений
scripts/                                  build/test, smoke, анализ bag,
                                         оценка, инъекция синтетики, desktop
docker/                                   Dockerfile для разработки и runtime,
                                         entrypoint и расширения Compose
compose.local.yaml, compose.yaml         headless и desktop сервисы
.devcontainer/                            конфигурации Dev Container и X11-скрипты
.github/workflows/                        CI: lint, packages, runtime, desktop
docs/                                    исторические рабочие заметки
rosbags/                                 приватные входные bag, вне Git
results/                                 генерируемые JSONL, метрики и логи, вне Git
README.md, PLAN.md                       текущее описание и план
```

<a id="быстрый-старт-образ--bag--результат"></a>

## Быстрый старт: образ → bag → результат

Нужны Docker и распакованные bag в `rosbags/` (папка `rosbags/<bag>/` с `metadata.yaml`). Команды выполняются из корня репозитория. `results/` доступен контейнеру для результатов офлайн-оценки; онлайн-сценарий публикует ROS-топик.

```bash
docker build -f docker/Dockerfile.runtime --target runtime -t metro-lidar:local .
mkdir -p results
docker run --rm --init --name metro-demo \
  --user "$(id -u):$(id -g)" \
  -v "$PWD/rosbags:/data:ro" -v "$PWD/results:/results" \
  metro-lidar:local \
  ros2 launch metro_perception_bringup perception.launch.py input_topic:=/lidar_points
```

В другом терминале, когда узлы запустились:

```bash
docker exec metro-demo metro-entrypoint ros2 bag play /data/doubleT_platform
```

В третьем терминале:

```bash
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/assessment --field state
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/assessment --field reason
```

Для положительной записи `doubleT_obstacle` используется полный скан с **исследовательским предположением** о направлении движения. Человек появляется после примерно 13-й секунды bag. Запустите следующий launch вместо варианта выше, затем `ros2 bag play /data/doubleT_obstacle`:

```bash
docker run --rm --init --name metro-demo \
  --user "$(id -u):$(id -g)" \
  -v "$PWD/rosbags:/data:ro" -v "$PWD/results:/results" \
  metro-lidar:local ros2 launch metro_perception_bringup perception.launch.py \
  input_topic:=/sensing/lidar/hesai128/pointcloud \
  sensor_profile:=/opt/metro/install/share/metro_perception_ros/config/full_scan_research_assumed.yaml
```

У установленного пакета путь профиля можно получить командой `ros2 pkg prefix metro_perception_ros`: добавьте `/share/metro_perception_ros/config/<имя>.yaml`. Команды `docker exec` явно запускают entrypoint, чтобы загрузить ROS и установленный workspace. Контейнер ограничивает ROS-связь localhost; все терминалы обращаются к одному контейнеру.

**Проверка 24.09.2026:** сборка и запуск описанных команд прошли. При `doubleT_obstacle` на этом стенде `ros2 bag play` публиковал облако, но `FrameAnalysis` за время прогона не был получен, а assessment оставался `UNKNOWN`; повтор с `--start-offset 13 --rate 0.2` также не дал анализа. Поэтому онлайн-подтверждение человека с дальностью этой инструкцией пока не доказано. Сохранённые результаты G4b ниже получены последовательным `evaluate_bag`; задача online 1× остаётся Q2 в PLAN.md.

Вариант с Compose запускает такой же headless runtime: `docker compose -f compose.local.yaml up -d --build`, затем `docker compose -f compose.local.yaml exec local metro-entrypoint ros2 launch ...`, `... ros2 bag play ...` и `... ros2 topic echo ...`. После работы: `docker compose -f compose.local.yaml down`.

## Как читать результат

| Топик | Тип | Содержание |
|---|---|---|
| `/lidar_points` или `/sensing/lidar/hesai128/pointcloud` | `sensor_msgs/msg/PointCloud2` | Вход; выбирается `input_topic` |
| `/metro/analysis` | `metro_perception_interfaces/msg/FrameAnalysis` | Геометрия одного кадра, кандидаты и диагностика |
| `/metro/assessment` | `metro_perception_interfaces/msg/PathAssessment` | Решение монитора, треки, дальность, возраст результата |
| `/metro/markers` | `visualization_msgs/msg/MarkerArray` | Маркеры при запуске `demo.launch.py` |

В ROS 2 Humble удобно читать отдельные поля:

```bash
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/assessment --field state
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/assessment --field distance_m
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/assessment --field reason
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/assessment --once --no-arr
docker exec metro-demo metro-entrypoint ros2 topic echo /metro/analysis --field overwritten_frames
docker exec metro-demo metro-entrypoint ros2 topic hz /metro/assessment
```

`state`: `0 UNKNOWN`, `1 OBSTACLE`, `2 NO_OBSTACLE_DETECTED`. Интерпретируйте `distance_m` только вместе с `distance_valid=true`; это продольная дальность до ближайшего подтверждённого трека. `OBSTACLE_WITH_ASSUMED_CALIBRATION` означает препятствие при непроверенной ориентации; `OBSTACLE_COASTING_WITH_ASSUMED_CALIBRATION` — прогноз трека в кадре без повторного измерения. `CANDIDATE_UNCONFIRMED` означает, что кандидат ещё не выполнил правило подтверждения. `ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR`, `BACKGROUND_CANNOT_CONFIRM_CLEAR`, `BASELINE_WARMUP`, `GROUND_UNSUPPORTED` и `INPUT_PAUSED_OR_STOPPED` объясняют типичные `UNKNOWN`. Поле `stale` сообщает о потере актуального входа; при этом объекты и треки очищаются, а ключ последнего наблюдения сохраняется.

`reported_objects` — кандидаты текущего кадра, включая неподтверждённые, с `bbox`, `nearest_point`, `distance_m`, `distance_valid`, `support_points` и `reasons` (`MOTION`/`GAUGE`). `tracks` содержат `track_id`, `confirmed`, `coasting`, `hits`, `age_frames`, `bbox`, `distance_m`. У `FrameAnalysis` смотрите `processing_age_ms`, `queue_age_ms`, `tf_wait_ms`, `received_frames`, `processed_frames`, `rejected_frames`, `overwritten_frames`, `evaluated_range_m` и `reason`. Поле `result_age_ms` в assessment показывает возраст результата по локальным часам.

## RViz

На машине с рабочим графическим дисплеем запустите desktop-контейнер, затем в одном терминале демо, в другом — bag:

```bash
bash scripts/desktop.sh up
bash scripts/desktop.sh exec ros2 launch metro_perception_bringup demo.launch.py \
  input_topic:=/lidar_points publish_sensor_tf:=true sensor_frame_override:=hesai_lidar
bash scripts/desktop.sh exec ros2 bag play /data/doubleT_platform
```

Для полного скана замените `input_topic` на `/sensing/lidar/hesai128/pointcloud`, передайте `sensor_profile:=/opt/metro/install/share/metro_perception_ros/config/full_scan_research_assumed.yaml` и оставьте `publish_sensor_tf:=true`. У профиля `exact` имя `sensor_frame_override` не нужно. `demo.launch.py` включает визуализатор и RViz (`rviz:=true`, `fixed_frame:=lidar_assumed`). При `publish_sensor_tf:=true` для профиля `bind_first` обязательно явно указать фактический frame облака через `sensor_frame_override`; для `exact` он уже задан в YAML. После работы: `bash scripts/desktop.sh down`.

Легенда `/metro/markers`: `status` — текст `STATE d m: reason`; `corridor` — зелёный при пригодной геометрии и покрытии, жёлтый иначе; `candidate_bbox` — красный для `MOTION`, синий для только `GAUGE` (ближайший непрозрачнее); `nearest_point` — точка ближайшего кандидата; `track_label` — `#id d m`, с пометкой `(predicted)` при прогнозе; `track_coasting` — полупрозрачный прогнозируемый бокс. Устаревшие маркеры удаляются.

В пустой конфигурации RViz установите **Fixed Frame = `lidar_assumed`**, затем **Add → By topic → `/metro/markers` → MarkerArray**. Добавьте входной `PointCloud2` и `TF`; для облака должен существовать transform из исходного frame в `lidar_assumed`. Для отдельного инструмента изображения глубины добавьте **Image → `/lidar/depth_image`** после запуска `depth_image.launch.py`.

<details>
<summary>Ручной desktop Docker и HiDPI</summary>

`docker build -f docker/Dockerfile.runtime --target runtime-desktop -t metro-lidar:desktop .` собирает образ с RViz. `scripts/desktop.sh` настраивает X11 proxy, монтирования, UID/GID и, если доступно, render device. Его `shell` открывает подготовленное окружение; `exec` запускает команду. Для ручного `docker run` нужно самостоятельно передать X11 socket, `DISPLAY`, `/data` и `/results`; подробности и настройки масштаба экрана находятся в [документации демо](docs/demo.md).

</details>

## Параметры и профили

| Аргумент `perception.launch.py` | По умолчанию | Назначение |
|---|---|---|
| `input_topic` | `/lidar_points` | Входное облако |
| `namespace` | `metro` | Пространство выходных топиков |
| `sensor_profile` | пусто, встроенный `forward_sector_assumed.yaml` | YAML геометрии и алгоритма |
| `publish_sensor_tf` | `false` | Публикация предполагаемого статического TF для RViz |
| `sensor_frame_override` | пусто | Явное имя source frame для `bind_first` при публикации TF |
| `algorithm_config`, `runtime_config` | YAML из bringup | Параметры узлов |
| `use_sim_time` | `false` | ROS clock; с `true` нужен bag `--clock` |

`demo.launch.py` добавляет `rviz` (`true`) и `fixed_frame` (`lidar_assumed`). Профиль выбирайте по **типу сенсора**, а не по имени bag. `forward_sector_assumed.yaml` связывается с первым `frame_id` потока (`bind_first`); ось вперёд принята как −Y сенсора. `full_scan_research_assumed.yaml` ожидает точно `lidar_livox`, использует ту же непроверенную ориентацию и нужен для исследования положительного bag. `full_scan_unresolved.yaml` не задаёт направление вперёд и сохраняет `UNKNOWN`. Исходные frame в известных данных: секторный `hesai_lidar`, полный скан `lidar_livox`. Передавайте путь к YAML через `sensor_profile:=...`.

Основные параметры в секциях `detector:` и `temporal:` профилей:

| Параметры | Значение | Смысл |
|---|---:|---|
| `corridor_half_width_m`, `corridor_height_m` | 2,0 м; 3,5 м | Полуширина и высота коридора |
| `background_history_frames`, `background_lag_frames` | 10; 5 | История для `MOTION` и лаг сравнения |
| `static_half_width_m`, `static_min_height_m`, `static_max_height_m` | 0,9 м; 0,3–2,5 м | Центральная полоса и высота для `GAUGE` |
| `static_max_length_m` | 3,0 м | Отсев длинных конструкций в `GAUGE` |
| `low_object_height_m`, `low_object_half_width_m` | 1,0 м; 0,5 м | Низкий объект учитывается только между рельсами |
| `envelope_half_width_m`, `envelope_min_speed_mps` | 1,5 м; 1 м/с | На ходу боковой `MOTION` без `GAUGE` не создаёт кандидата |
| `confirm_hits`, `confirm_window`; `gauge_confirm_hits`, `gauge_confirm_window` | 2/3; 2/3 | Подтверждение трека по кадрам |
| `release_misses`, `still_speed_mps`, `still_min_points` | 2; 0,5 м/с; 50 | Удержание и мгновенное подтверждение уверенной цели на стоянке |

Полный набор и допустимые значения — в [config.hpp](metro_perception_core/include/metro_perception_core/config.hpp); загрузка YAML — в [preprocessing.cpp](metro_perception_ros/src/preprocessing.cpp). Параметры среды в [runtime.yaml](metro_perception_bringup/config/runtime.yaml): `input_reliability=best_effort`, `max_processing_age_s=0.30`, `tf_wait_timeout_s=0.05`, `timeout_s=0.5`. Исследовательский профиль разрешает обработку при ASSUMED калибровке, но не подтверждает свободный путь.

## Архитектура

```text
PointCloud2 → perception_node → FrameAnalysis → obstacle_monitor_node → PathAssessment
                                                               │
                                                               └→ visualizer_node → MarkerArray
rosbag → evaluate_bag → тот же C++ pipeline → frames.jsonl → metrics.py → quality.json
```

| Пакет | Роль |
|---|---|
| `metro_perception_core` | ROS-независимый C++ детектор и временной монитор |
| `metro_perception_interfaces` | Сообщения анализа, кандидатов, треков и состояния |
| `metro_perception_ros` | Узлы, адаптер PointCloud2, TF и офлайн-оценщик |
| `metro_perception_bringup` | Launch, runtime config и RViz |
| `metro_perception_tools` | Изображение глубины, метрики, отчёты и осмотр bag |

Онлайн-обработка использует latest-only слот: один кадр обрабатывается, один ожидает; более ранний ожидающий может быть перезаписан. Монитор привязывает результат к `source_instance_id`, `session_id`, `frame_sequence`; чужие и старые анализы отвергаются. Watchdog после остановки входа переводит результат в `UNKNOWN`, очищает кандидатов и треки, сохраняя ключ наблюдения. Время bag служит для разметки и оценки, `header.stamp` — для точного TF, локальные монотонные часы — для watchdog и возраста результата.

Контракт решения: `OBSTACLE` возникает только от подтверждённого трека с конечной положительной дальностью; `NO_OBSTACLE_DETECTED` требует `VERIFIED` калибровки, пригодной области на положительной дальности и отсутствия кандидатов; иначе `UNKNOWN`. Кандидаты `reported_objects` относятся к отдельному кадру, а не к подтверждённым трекам. При невалидной дальности сохраняются NaN и `distance_valid=false`.

## Алгоритм

1. **Подготовка A02.** Адаптер проверяет структуру PointCloud2, отбрасывает NaN/Inf и ближнюю слепую область, применяет TF к целевой системе координат и две ROI. Сохраняет связь с исходными точками и диагностику отброса.
2. **Опора пола.** По нижним точкам пространственных ячеек выбирается ограниченная плоскость RANSAC с проверкой наклона и поддержки. Непрерывность пола по пятиметровым диапазонам ограничивает пригодную дальность; пропадание опоры ведёт к `UNKNOWN`.
3. **Ось маршрута.** Стены тоннеля дают криволинейную ось `y = c1·x + c2·x²`; коридор строится вокруг неё. Это позволяет отличать путь от стен и платформы на повороте.
4. **Два канала свидетельств.** `MOTION` сравнивает текущий дальностный профиль со скользящей историей после компенсации собственной скорости по LiDAR. `GAUGE` допускает статический объект в узкой центральной полосе без истории, чтобы обнаружить его при приближении.
5. **Объекты.** Угловая кластеризация объединяет соседние возвраты; bounding box и ближайшая точка проверяются на исходных точках внутри коридора. Длинные конструкции отсекаются в `GAUGE`; низкие возвраты принимаются только между рельсами. При движении боковой `MOTION` вне габарита вагона не создаёт препятствие, если нет `GAUGE`.
6. **Дальность и время.** Дальность — продольная координата ближайшей принятой точки, а не Евклидово расстояние до центра бокса. Треки связываются между кадрами: два попадания из трёх подтверждают, один пропуск удерживается прогнозом; достаточно плотный `MOTION` объект при остановке подтверждается сразу. Решение использует только подтверждённые треки и контракт калибровки.

Нужны только облака LiDAR и явно заданный профиль. IMU, одометрия поезда и внешний TF не требуются для assumed-профиля; при наличии TF используется точная метка облака. Предположение об ориентации не является измеренной калибровкой.

## Эксперименты и воспроизведение

Набор состоит из шести коротких bag: одна запись с человеком (`doubleT_obstacle`) и пять отрицательных по описанию владельца. Разметка находится в [evaluation/annotations](evaluation/annotations), выборка — в [splits.yaml](evaluation/splits.yaml). Development содержит positive; validation содержит только отрицательные bag. После G4 ложные тревоги разбирались на обеих половинах («две половины»), поэтому validation не независима. Синтетические объекты добавляются к отрицательным bag и оцениваются отдельно. `new_data` в текущую разметку и метрики не входит.

| Прогон | SHA текущей истории | FP development, кадры/события | FP validation, кадры/события | Совпадения с человеком |
|---|---|---:|---:|---:|
| Q1 / G3 | `c1e791e` / `9d5d9a1` | 35 / 12 | 162 / 17 | 11 / 35 |
| G3f / G3r | `ae6177f` / `0a4b47f` | 53 / 10 | 144 / 13 | 35 / 35 |
| S2 | `89f02f5` | 157 / 20 | 229 / 17 | 53 / 56 |
| G4 | `848549d` | 134 / 8 | 203 / 14 | 52 / 56 |
| G4b | `28e8d03` | 97 / 6 | 56 / 6 (2,13/мин) | 53 / 56 |

Начиная с S2, область человека размечена на 56 кадрах вместо 35, поэтому доли между ранними и поздними прогонами напрямую не сравниваются. `FP` — кадры со состоянием `OBSTACLE` на отрицательном интервале; события объединяют близкие срабатывания. Прогоны и хеши исходного кода/конфигурации сохранены в `results/<run>/manifest.json`, оценка — в `quality.json`. В старых манифестах до перевода тел коммитов записаны прежние SHA; соответствия находятся в `results/ru-bodies/sha-map-2026-09-24.txt`.

| Bag в G4b | Split | FP кадры/события | Offline p50/p95, мс |
|---|---|---:|---:|
| `doubleT_obstacle` | development | 67 / 1 | 70,5 / 79,9 |
| `doubleT_platform` | development | 10 / 3 | 79,5 / 86,2 |
| `roundT_pressureGate_roundT` | development | 20 / 2 | 72,0 / 103,8 |
| `roundT_doubleT` | validation | 56 / 6 | 50,4 / 73,7 |
| `roundT_squareT_pressureGate_squareT` | validation | 0 / 0 | 110,0 / 126,8 |
| `squareT_platform_squareT_switch` | validation | 0 / 0 | 94,3 / 111,8 |

Это время **последовательной офлайн-обработки кадра**, не задержка онлайн-системы и не доказательство 10 Гц. В G4b синтетический неподвижный человек на validation обнаружен в 32/32 кадрах до 20 м, 40/46 на 20–40 м, 26/42 на 40–60 м, 2/22 на 60–80 м и 0/11 на 80–100 м; первое подтверждение по сценам — около 50–66 м. Пересекающий полосу человек: примерно 70% на 0–40 м. Низкий бокс высотой 0,5 м: 0–20% вблизи. На development синтетический неподвижный человек обнаруживался до 79 м, на 80+ м recall равен нулю. На двух реальных эталонных кадрах ошибка продольной дальности составляет +0,10…+0,13 м; это сравнение с размеченным облаком в предположенной системе координат, а не метрологическая проверка дальности поезда. Задержка первого подтверждения G4 на положительном событии составляла около 0,38 с; у текущего G4b по `quality.json` — 0,286 с.

Основные ложные тревоги приходили от стен, платформ и оборудования при въезде в двойной тоннель, на поворотах и от боковых длинных конструкций. Снижение дали оценка криволинейной оси, раздельный `GAUGE`, ограничение длины/высоты, низких объектов и боковой полосы на ходу, а также подтверждение 2/3. Проверялись и отклонялись склейка близких срезов, чрезмерно широкий криволинейный `GAUGE` и общее правило 3/5: они ухудшали ложные тревоги, дальность или задержку. Сложными остаются двухпутный тоннель и человек около 56 м: последний требует пересмотра разметки/свидетельств без подгонки по имени записи.

Для повторения количественной оценки используйте mounted checkout и новый каталог `/results/NAME`:

```bash
docker build -f docker/Dockerfile --target universal -t metro-lidar:dev docker
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/repo:ro" -v "$PWD/rosbags:/data:ro" -v "$PWD/results:/results" \
  metro-lidar:dev bash /repo/scripts/evaluate_in_container.sh my-run --full-scan-research
python3 scripts/fp_events.py results/my-run
```

Скрипт строит именно смонтированный checkout, хеширует бинарники и пишет JSONL/метрики; детали аргументов — в [evaluate_in_container.sh](scripts/evaluate_in_container.sh), [evaluate_all.py](scripts/evaluate_all.py), [fp_events.py](scripts/fp_events.py). Синтетические сценарии `static`, `crossing`, `box` создаёт [inject_obstacle.py](scripts/inject_obstacle.py); его команды `inject`, `dataset`, `report` описаны в начале файла. Результаты прогонов в `results/` не входят в Git.

## Ограничения

- На реальных bag нет утверждения «путь свободен»: калибровка ASSUMED. Официальный профиль неориентированного полного скана сохраняет `UNKNOWN`.
- Надёжная дальность обнаружения синтетического человека сейчас примерно до 60–80 м, хотя detection ROI простирается до 120 м; после ~90 м часто не хватает опоры пола. Требование 100+ м не доказано.
- Низкие объекты обнаруживаются плохо; правило между рельсами намеренно отсекает низкие боковые конструкции.
- Двухпутный тоннель, ошибки LiDAR-одометрии и предполагаемая ориентация полного скана остаются источниками ошибок.
- Есть только один реальный positive; validation использована при разработке, независимый positive holdout отсутствует. Скрытые контрольные bag могут дать другие метрики.
- Онлайн 1×, потери, RSS и визуальная проверка RViz на целевой машине ещё не измерены. Пока нет основания обещать частоту или демонстрационную готовность.
- При движении человек в боковой полосе 1,5–2,0 м без `GAUGE` не сообщается как препятствие: это правило габарита вагона, требующее проверки на контрольных сценах.

## Разработка

Для разработки есть Dev Container и универсальный [Dockerfile](docker/Dockerfile) (`universal`, `desktop`). В среде ROS 2 Humble: `bash scripts/build.sh`, `source install/local_setup.bash`, `bash scripts/test.sh`; интеграция — `python3 scripts/smoke_monitor.py` и `python3 scripts/smoke_detector.py`. CI проверяет `clang-format --dry-run --Werror`, `ament_flake8`, `ament_pep257`, сборку, тесты и smoke в четырёх jobs. Изменения детектора требуют повторить оценку по размеченным bag, синтетике, `UNKNOWN` и времени с зафиксированными commit/profile/manifest.

## Изображение глубины и рабочие записи

`depth_image.launch.py` из пакета bringup публикует `/lidar/depth_image` и может записать видео через `video_path`. Это вспомогательный вид облака; состояние пути и расстояние публикует детектор. Параметры изображения доступны через `ros2 launch metro_perception_bringup depth_image.launch.py --show-args`.

[docs/](docs/) и [evaluation/README.md](evaluation/README.md) содержат исторические рабочие записи на дату создания. Текущие статус, команды и ограничения описаны здесь и в [PLAN.md](PLAN.md).
