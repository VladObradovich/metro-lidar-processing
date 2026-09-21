# Демонстрация

Существующая визуализация глубины описана в [README](../README.md).
Для просмотра каркаса в desktop-образе после пересборки:

```bash
bash scripts/desktop.sh up
bash scripts/desktop.sh exec ros2 launch metro_perception_bringup demo.launch.py \
  publish_sensor_tf:=true sensor_frame_override:=hesai_lidar
# Во втором терминале:
bash scripts/desktop.sh exec ros2 bag play /data/my_bag
```

`hesai_lidar` в примере — frame исходных forward-sector bag. Для другой записи
указать фактический `PointCloud2.header.frame_id`. Без аргументов demo тоже
запускается, но не публикует TF: для просмотра облака в RViz нужен внешний TF
либо `fixed_frame`, совпадающий с frame облака.

Для второго входа передать `input_topic:=/sensing/lidar/hesai128/pointcloud
fixed_frame:=lidar_livox` и выбрать профиль `full_scan_unresolved.yaml` из
`metro_perception_ros/config`; публикация profile TF для него не нужна.
Это показывает исходное облако и UNKNOWN. Bbox и
дальность появятся после реализации детектора. `rviz:=false` отключает GUI.
Player пока запускается отдельно; автоматическую готовность/запуск bag добавить
в R03 вместо фиксированного sleep.

Сценарий финальной сдачи (пока не выполнен): облако → corridor → объект →
расстояние → остановка входа/UNKNOWN. Сохранить видео, слайды по шаблону и
run manifest в results; указать SHA/config/image для каждого артефакта.
