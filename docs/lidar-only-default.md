# Режим по умолчанию: lidar-only профили

Измеренной монтажной калибровки, TF, odometry и IMU в предоставленных bag нет.
Поэтому проект не выдаёт одну геометрическую гипотезу за универсальную калибровку.

## Профили

Для стандартного forward-sector входа `/lidar_points` без явного
`sensor_profile` используется установленный
`metro_perception_ros/config/forward_sector_assumed.yaml`. Имя исходного ROS
frame заранее не фиксируется: режим `source_frame_mode: bind_first` принимает
`header.frame_id` первого валидного облака и закрепляет его до конца session.

Он фиксирует только подтверждённую по сектору гипотезу:

- target frame: `lidar_assumed`;
- начало координат: оптический центр лидара;
- X вперёд = исходная −Y;
- Y влево = исходная +X;
- Z вверх = исходная +Z;
- `calibration_verified: false`;
- `allow_unverified_calibration: true`;
- `calibration_trust=ASSUMED`;
- имя source frame не является идентификатором модели лидара и не выбирает профиль.

Имя `base_link` для этой гипотезы не используется. Настоящий `base_link`
оставлен для измеренной или независимо подтверждённой геометрии машины.

Для 360-градусного входа `/sensing/lidar/hesai128/pointcloud` с frame
`lidar_livox` используется `full_scan_unresolved.yaml`. Облако остаётся в
своём native frame, направление движения не угадывается, а результат остаётся
fail-closed (`UNKNOWN`) до появления проверенной ориентации.

`scripts/evaluate_all.py` выбирает эти профили по `sensor_profile` из
`evaluation/dataset.yaml`, а не по имени bag. Неизвестный sensor_profile
считается ошибкой конфигурации. Topic routing не хранится в geometry-профиле:
он задаётся отдельно через `input_topic` в launch или через `input_topic` metadata
в `evaluation/dataset.yaml`. Поэтому выбор геометрии и выбор ROS topic не смешаны.

## Доверие к калибровке

`calibration_trust` проходит через core → FrameAnalysis → monitor →
PathAssessment:

- `UNKNOWN` — TF/геометрия не установлены;
- `ASSUMED` — используется явно документированная гипотеза;
- `VERIFIED` — калибровка подтверждена.

Пустой список кандидатов при `ASSUMED` не может подтверждать
`NO_OBSTACLE_DETECTED`. Для подтверждения свободного пути требуется
`VERIFIED`.

## Запуск

Forward-sector default:

```bash
ros2 launch metro_perception_bringup perception.launch.py \
  input_topic:=/lidar_points use_sim_time:=true
ros2 bag play /data/private_bag --clock
```

Обычный headless launch не публикует assumed transform в глобальный TF graph.
Для RViz `demo.launch.py` включает публикацию profile TF отдельным
`tf2_ros/static_transform_publisher`. Для bind-first профиля имя child frame
нужно передать явно, например для текущих открытых bag:
`sensor_frame_override:=hesai_lidar`. Для приватного bag указывается фактический
`PointCloud2.header.frame_id`. Perception-нода сама не владеет `/tf_static`.

Offline forward-sector без пятого аргумента использует тот же default:

```bash
ros2 run metro_perception_ros evaluate_bag \
  /data/private_bag /lidar_points /results/private.jsonl
```

Для всего зарегистрированного набора:

```bash
python3 scripts/evaluate_all.py --dataset-root /data --output-dir /results/run
```

Для full-scan отдельный профиль выбирается автоматически только в
`evaluate_all.py` по dataset metadata. При ручном запуске его нужно передать
явно. Автовыбора профиля по имени topic/frame/размеру облака в runtime нет:
bind-first определяет только имя уже выбранной системы координат, а не модель
лидара и не геометрическую ориентацию.

## Настоящая калибровка

Шаблоны `forward_sector.yaml` и `full_scan.yaml` в bringup используют
`target_frame: base_link` и предназначены для реальной геометрии. После
измерения монтажа нужно заполнить translation/RPY и `calibration_source`, а
`calibration_verified=true` ставить только после независимой проверки.

A03–A07 пока не реализованы; текущий A02 остаётся preprocessing-каркасом.
