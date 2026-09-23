# Паспорт шести коротких LiDAR bag (D1)

Проверено 23.09.2026. `new_data` в паспорт и текущую разметку не входит.

## Источник и получение данных

Все шесть записей — содержимое архива организаторов `for_hackathon.zst`
(внутри `датасет.zip` → `archive/for_hackathon.zst`). Данные приватные,
выданы участникам хакатона; в Git и в образы они не попадают, публикация не
предусмотрена. Получить их можно только у организаторов.

Единый корень данных — распакованный каталог `for_hackathon/`:

```bash
tar --zstd -xf for_hackathon.zst          # создаёт for_hackathon/<bag_id>/
mv for_hackathon/* rosbags/               # или ln для экономии места
cd rosbags && sha256sum -c ../evaluation/short-bags.sha256
```

`rosbags/` монтируется в контейнер как `/data` (`compose.yaml`,
`compose.local.yaml`), поэтому все прогоны используют `--dataset-root /data`,
а `evaluation/dataset.yaml` задаёт пути `<bag_id>` относительно этого корня.
Пути в `evaluation/short-bags.sha256` тоже относительны корню.

## Состав и полная покадровая проверка

`scripts/audit_bags.py` читает **все** кадры входной темы (не выборку) и
проверяет frame_id, раскладку полей, размеры буферов, конечность XYZ и
временные метки. Результат последнего прогона — `results/d1-audit/audit.json`
(каталог игнорируется Git).

```bash
python3 scripts/audit_bags.py --dataset-root /data --output /results/d1-audit/audit.json
```

| Bag | Тема | frame_id | Кадров | Длит., с | Точек в кадре | Пропуски > 150 мс |
| --- | --- | --- | ---: | ---: | ---: | --- |
| `doubleT_obstacle` | `/sensing/lidar/hesai128/pointcloud` | `lidar_livox` | 201 | 20,39 | 921 600 | 2 (до 0,40 с) |
| `doubleT_platform` | `/lidar_points` | `hesai_lidar` | 345 | 34,42 | 307 200 | 0 |
| `roundT_doubleT` | `/lidar_points` | `hesai_lidar` | 252 | 25,07 | 307 200 | 0 |
| `roundT_pressureGate_roundT` | `/lidar_points` | `hesai_lidar` | 268 | 26,70 | 307 200 | 0 |
| `roundT_squareT_pressureGate_squareT` | `/lidar_points` | `hesai_lidar` | 545 | 55,38 | 307 200 | 1 (1,10 с) |
| `squareT_platform_squareT_switch` | `/lidar_points` | `hesai_lidar` | 877 | 88,21 | 307 200 | 1 (0,70 с) |

Во всех шести записях:

- одна тема `sensor_msgs/msg/PointCloud2`, других тем (в том числе `/tf`,
  `/tf_static`, odom, IMU) нет — данные lidar-only;
- во всех кадрах один `frame_id` и одна раскладка полей: `x, y, z, intensity,
  ring, timestamp`, `point_step = 26`, little-endian; дефектов `row_step`/длины
  буфера нет;
- число точек в кадре постоянно, все XYZ конечны. Облако организованное:
  отсутствие отражения кодируется точкой, а не NaN, поэтому «конечные XYZ» не
  означают «валидный возврат» — пустые точки отбрасывает очистка A02;
- bag- и header-метки строго возрастают, номинальный период 100 мс (10 Гц).
  Пропуски видны в обеих шкалах, то есть кадры потеряны при записи, а не при
  воспроизведении;
- `bag_stamp_ns − header.stamp` постоянен в пределах записи (разброс ≈ 40 мс)
  и составляет ≈ 841 666 с (≈ 9,74 сут): часы лидара и записывающей машины не
  синхронизированы. Разметка использует `bag_stamp_ns`, TF и measurement time —
  `header.stamp`; смешивать шкалы нельзя.

`doubleT_obstacle` отличается от остальных: другая тема, `frame_id` и плотность
(921 600 точек, полный скан). Название `lidar_livox` не совпадает с темой
`hesai128`; это свойство записи, мы его не исправляем.

## Оси, габарит и калибровка — принятое решение

Измеренных extrinsics, монтажа и габарита подвижного состава организаторы не
предоставили, а в bag нет TF. По контракту lidar-only (`PLAN.md`, раздел 2)
это не блокирует работу, но ограничивает доверие:

- пять секторных записей обрабатываются профилем
  `metro_perception_ros/config/forward_sector_assumed.yaml`: `forward = −Y`,
  `left = +X`, `up = +Z` датчика, перенос нулевой, `calibration_verified: false`;
- для `doubleT_obstacle` штатный `full_scan_unresolved.yaml` не выбирает
  направление движения и оставляет `UNKNOWN`; поворот `−Y → X` в G2 —
  исследовательское допущение, не измерение;
- полуширина коридора 2 м и высота 3,5 м — параметры алгоритма, не
  согласованный габарит;
- все результаты имеют trust `ASSUMED`; без калибровки отсутствие кандидата
  остаётся `UNKNOWN`, а дальности указаны в предполагаемой системе лидара.

Если организаторы передадут калибровку или габарит, их вносят в профиль
отдельным изменением с источником; до этого `ASSUMED` не заменяется на
`VERIFIED`. Подробности — в [calibration.md](calibration.md) и
[lidar-only-default.md](lidar-only-default.md).
