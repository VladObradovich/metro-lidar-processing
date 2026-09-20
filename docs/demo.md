# Демонстрация

Существующая визуализация глубины описана в [README](../README.md).
Для просмотра каркаса в desktop-образе после пересборки:

```bash
bash scripts/desktop.sh up
bash scripts/desktop.sh exec ros2 launch metro_perception_bringup demo.launch.py
# Во втором терминале:
bash scripts/desktop.sh exec ros2 bag play /data/my_bag
```

Для второго входа добавить к demo `input_topic:=/sensing/lidar/hesai128/pointcloud
fixed_frame:=lidar_livox`. Это показывает исходное облако и UNKNOWN. Bbox и
дальность появятся после реализации детектора. `rviz:=false` отключает GUI.
Player пока запускается отдельно; автоматическую готовность/запуск bag добавить
в R03 вместо фиксированного sleep.

Сценарий финальной сдачи (пока не выполнен): облако → corridor → объект →
расстояние → остановка входа/UNKNOWN. Сохранить видео, слайды по шаблону и
run manifest в results; указать SHA/config/image для каждого артефакта.
