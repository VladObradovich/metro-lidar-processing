# Проверка A02 на текущем наборе

Ниже сохранён **исторический** прогон строгого preview от 21.09.2026. Текущий default
уже использует отдельную допущенную геометрию для известного forward-sector входа:
[lidar-only-default.md](lidar-only-default.md). Числа исторического прогона ниже не
пересчитывались после последующих изменений pipeline.

Дата прогона: 21.09.2026. ROS 2 Humble, отдельный контейнер, исходные bag смонтированы read-only.

Прочитаны и обработаны **13 759 кадров из всех 7 bag**. Числа совпали с metadata.
На каждом кадре исторического preview применялась тогдашняя preview-геометрия, выполнены
очистка и ROI. Все результаты: `INVALID_GEOMETRY / CALIBRATION_UNVERIFIED`, `UNKNOWN`,
`distance_m=null`, `evaluation_region_valid=false`. Ошибок декодирования и TF lookup в
этом прогоне нет.

Это проверка обработки входа и защитных условий; монтаж и качество детектора не подтверждены.

| Bag | Кадров | Geometry ROI, min–max точек | Detection ROI, min–max точек | Сессий |
|---|---:|---:|---:|---:|
| doubleT_obstacle | 201 | 340810–345993 | 180807–184157 | 1 |
| doubleT_platform | 345 | 154856–190378 | 114103–180063 | 1 |
| roundT_doubleT | 252 | 177707–191154 | 117305–190015 | 1 |
| roundT_pressureGate_roundT | 268 | 160550–190770 | 160271–190493 | 1 |
| roundT_squareT_pressureGate_squareT | 545 | 167068–191076 | 150986–190715 | 1 |
| squareT_platform_squareT_switch | 877 | 156046–189993 | 126359–189788 | 1 |
| new_data | 11271 | 152356–191094 | 75550–190374 | 1 |

Число сессий увеличивается при скачке валидного `header.stamp` назад. Нулевые,
отрицательные и некорректные stamp не меняют session/source binding и обрабатываются
fail-closed как `BAD_INPUT / INVALID_TIMESTAMP`. TF выбирается только на валидном времени
измерения; bag record time отдельно сохраняется в каждой строке JSONL.

## Текущее состояние A02

- `forward_sector_assumed.yaml` использует `source_frame_mode: bind_first`: имя source frame
  привязывается к первому кадру каждой session, а явная гипотеза Rz(+π/2) и нулевая
  translation остаётся `ASSUMED`, а не измеренной калибровкой.
- `full_scan_unresolved.yaml` и `full_scan_preview.yaml` оставляют 360° облако в нативном
  `lidar_livox` frame без выдуманного направления движения поезда и fail-closed до D03.
- `forward_sector_preview.yaml` по-прежнему является только визуальной гипотезой
  `hesai_lidar -> lidar_preview`, Rz(+π/2), translation=0 и `calibration_verified=false`.
  Таким образом, два preview-профиля намеренно больше не имеют одинаковый TF.
- Online и offline пути используют только валидный положительный stamp измерения; zero-time
  никогда не превращается в latest-TF fallback. Offline evaluator воспроизводит из bag `/tf`
  и `/tf_static`, а не ограничивается только статическим TF из sensor profile.
- PointCloud2 decoder до чтения XYZ проверяет layout/strides, уникальность и непересечение
  scalar FLOAT32/FLOAT64 полей x/y/z. NaN/Inf не проходят в геометрию: они учитываются как
  invalid points, а кадр без валидной геометрии остаётся fail-closed.
- Помимо `max_points` действует независимый `max_cloud_bytes` (по умолчанию 256 MiB,
  допустимый максимум 1 GiB). Оба лимита одинаково применяются online и offline; evaluator
  поддерживает явные overrides `MAX_POINTS` и `MAX_CLOUD_BYTES`.
- Latest-only worker имеет модель 1 processing + 1 pending: новый pending кадр заменяет
  предыдущий. Unit и integration/load smoke проверяют overwrite, newest-frame-wins,
  reset isolation и bounded shutdown.

### Контракт параметров online/offline

Аудит перед PR разделяет параметры на две группы:

- Общие для результата preprocessing: sensor profile, `max_points` и
  `max_cloud_bytes`. Online получает лимиты из ROS parameters, offline evaluator — из
  одноимённых positional overrides; `evaluate_all.py` пробрасывает оба значения и
  записывает их в manifest. Значения по умолчанию одинаковы: 2 000 000 точек и 256 MiB.
- Timestamp, source-frame binding, TF lookup, decode и причины fail-closed проходят через
  общие helpers. Invalid measurement stamp в online monitor и offline monitor нормализуется
  в `0`, а сам frame остаётся `BAD_INPUT / INVALID_TIMESTAMP`.
- Offline TF replay имеет окно `TF_LOOKAHEAD_S` / `--tf-lookahead-s` (0.05 с по
  умолчанию), измеряемое по bag record time. При сравнении с online согласовать
  его с `tf_wait_timeout_s`; это не симуляция wall-clock scheduling.
- `input_reliability`, `max_processing_age_s`, `tf_wait_timeout_s` и `timeout_s` —
  realtime-only параметры транспорта, очереди, ожидания TF и watchdog. Offline exporter
  намеренно не имитирует wall-clock scheduling/heartbeat. Поэтому они не являются частью
  алгоритмического online/offline контракта.

## Автоматические проверки

Текущий CI не фиксирует в этом документе хрупкие абсолютные количества тестов; актуальным
источником является зелёный workflow для HEAD. Он проверяет:

- сборку и пакетные тесты ROS 2 Humble, clang-format, flake8 и pep257;
- core/preprocessing, TF на `header.stamp`, invalid/missing/extrapolated TF и конфигурацию;
- `smoke_perception`, online/offline A02 parity, default lidar-only profile, namespace
  isolation и latest-only worker under load;
- установленный `runtime` image без source mount, dataset и сети: базовый smoke, A02 parity
  и default lidar-only profile;
- `runtime-desktop`: сборку установленного desktop target и наличие `rviz2` и bringup package.

GUI/RViz интерактивно CI не проверяет.

Сырые JSONL и summary.json именно исторического прогона сохранены локально в
`results/a02-20260921/` (исключены из Git). Там же логи того прогона.
Команды воспроизведения: [calibration.md](calibration.md).

A03–A07 и временное подтверждение кандидатов ещё не входят в результат A02.
D03 остаётся открытым: нет измеренной установки лидара относительно поезда.
Кастомные ROS parameter overrides offline сверх поддерживаемых evaluator overrides не
считаются полностью эквивалентными произвольному online launch. Timings A02 не являются
оценкой будущего детектора.
