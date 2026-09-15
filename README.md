# metro-lidar-processing

ROS 2 package for processing the hackathon metro lidar recordings.

## Установка и разработка в VS Code

Для проекта используется отдельный Dev Container **Ubuntu 22.04 / ROS 2 Humble**.
Он включает Python 3.10, C++/CMake, Eigen/PCL, NumPy/SciPy/OpenCV, RViz2,
rosbag2, Cyclone DDS, отладчик и средства тестирования. Сейчас реализована
визуализация глубины; детектор препятствий будет разрабатываться дальше.

### 1. Подготовить хост

На Linux-хосте нужны Docker Engine, VS Code и расширение
**Dev Containers** (`ms-vscode-remote.remote-containers`). ROS на хосте не нужен.
Проверьте Docker в обычном терминале хоста:

```bash
docker context show
docker version
docker ps
code --install-extension ms-vscode-remote.remote-containers
```

Docker должен быть доступен тому же пользователю, который запускает VS Code.
Если локальный Engine работает в контексте `default`, а выбран другой,
переключитесь через `docker context use default`.

Ожидаемое расположение данных:

```text
hahaton/
├── metro-lidar-processing/   # Открывать эту папку в VS Code
│   ├── .devcontainer/
│   ├── docker/
│   └── scripts/
├── archive/for_hackathon/    # Распакованные bag-файлы
└── videos/                  # Результаты и видео
```

Оба соседних каталога должны существовать до открытия контейнера. В текущем
проекте они уже есть. При другом расположении измените `mounts` в
`.devcontainer/devcontainer.json`.

### 2. Открыть отдельный контейнер

1. Если VS Code подключён к другому контейнеру, вернитесь на хост через
   **Dev Containers: Reopen Folder Locally** либо откройте новое локальное окно.
2. Откройте **именно `hahaton/metro-lidar-processing`**, а не родительскую
   `hahaton`: конфигурация Dev Containers находится в корне репозитория.
3. Выполните **Dev Containers: Reopen in Container** из палитры команд.
4. Дождитесь сборки образа и выполнения `postCreateCommand` — он собирает
   существующий ROS-пакет. Первая сборка скачивает несколько ГБ зависимостей.
5. Откройте новый терминал VS Code и проверьте:

```bash
echo "$ROS_DISTRO"                 # humble
python3 --version                  # Python 3.10.x
ros2 pkg prefix metro_lidar_processing
```

Название среды в VS Code: **Metro LiDAR · ROS 2 Humble**.

### 3. Где находятся файлы

| В контейнере | Назначение |
|---|---|
| `/workspaces/metro-lidar-processing` | Исходники с хоста; изменения сразу видны в Git |
| `/data` | `archive/for_hackathon`, подключён только для чтения |
| `/results` | Каталог `videos` на хосте; результаты сохраняются после пересоздания |
| `~/ros_ws/src/metro-lidar-processing` | Ссылка на исходники |
| `~/ros_ws/build`, `install`, `log` | Новая сборка Humble внутри контейнера |

Сборочные каталоги внутри контейнера могут исчезнуть при **Rebuild Container**;
они создаются повторно автоматически. Исходники, bag и `/results` сохраняются.
Старые `build/install/log` в репозитории не используются и не удаляются.

Процессы работают от пользователя `dev`; VS Code подстраивает его UID/GID
под пользователя хоста. Для установки дополнительных инструментов есть `sudo`.
Постоянные зависимости добавляйте в Dockerfile, затем выполняйте
**Dev Containers: Rebuild Container**. Это образ для разработки, не финальный
минимальный образ сдачи.

### 4. Сборка и тесты

Из корня репозитория внутри контейнера:

```bash
bash scripts/build.sh
source ~/ros_ws/install/local_setup.bash
bash scripts/test.sh
```

В новых интерактивных терминалах Humble и готовый workspace подключаются
автоматически. После изменения исходников C++ или launch/setup требуется
повторная сборка. Новые зависимости ROS устанавливайте внутри контейнера:

```bash
rosdep update --rosdistro humble
rosdep install --from-paths ~/ros_ws/src --ignore-src --rosdistro humble -y
```

Для воспроизводимости затем отразите необходимые зависимости в `package.xml`
и Dockerfile. Не подключайте старый `install/setup.bash` из Jazzy.

### 5. RViz на Linux / X11 / XWayland

Dev Container передаёт `DISPLAY` и сокет `/tmp/.X11-unix`. Перед первым запуском
GUI разрешите своему локальному пользователю доступ к X-серверу **на хосте**:

```bash
xhost +si:localuser:$(id -un)
```

Затем в контейнере:

```bash
rviz2
```

Для изображения глубины добавьте `Image`, топик `/lidar/depth_image`.
Для 3D-облака добавьте `PointCloud2`, выберите входной топик и установите
Fixed Frame по его `header.frame_id`: `hesai_lidar` либо `lidar_livox`.

По умолчанию включён программный OpenGL (`LIBGL_ALWAYS_SOFTWARE=1`), поэтому
GPU-проброс не требуется. Ошибка `could not connect to display` означает,
что нужно проверить `DISPLAY`, локальный XWayland и разрешение X-сервера.
Разрешение можно отозвать на хосте:

```bash
xhost -si:localuser:$(id -un)
```

На сервере без GUI удалите X11 mount из devcontainer.json; сборка и обработка
bag не требуют RViz. Конфигурация рассчитана на локальный Linux Docker Engine.

### 6. ROS-сеть

По умолчанию `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; используется сеть хоста.
Проигрывайте bag и запускайте обработку в новой среде с одинаковыми настройками.
Domain ID отделяет discovery от обычных запусков с domain 0, но не является
механизмом защиты. Для внешнего датчика потребуется отключить localhost-only
и согласовать домен и сеть.

### 7. Сборка образа без VS Code

В корне репозитория на хосте:

```bash
docker build -t metro-lidar-dev:humble docker
```

Контекст сборки — только маленький каталог `docker`: исходники, архивы,
bag-файлы и видео в Docker-образ не копируются.

Ручной запуск (альтернатива Dev Containers, UID/GID по умолчанию 1000):

```bash
docker run -d --init --name metro-lidar-dev \
  --network host --shm-size 1g \
  -e DISPLAY="$DISPLAY" \
  --mount "type=bind,source=$PWD,target=/workspaces/metro-lidar-processing" \
  --mount "type=bind,source=$PWD/../archive/for_hackathon,target=/data,readonly" \
  --mount "type=bind,source=$PWD/../videos,target=/results" \
  --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly \
  metro-lidar-dev:humble
docker exec -it metro-lidar-dev bash
```

При ручном запуске с другим UID/GID соберите образ с
`--build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)"`.
В контейнере выполните `bash scripts/build.sh`. Для подключения VS Code к уже
запущенной ручной среде доступна команда **Dev Containers: Attach to Running
Container**. Расширения и настройки удобнее устанавливаются при основном
сценарии **Reopen in Container**.

Документация: [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers).

## Панорамное изображение глубины

`depth_image` преобразует `sensor_msgs/msg/PointCloud2` в дальностное изображение лидара.
Горизонтальная ось изображения соответствует азимуту лидара. Каждая из 128 строк
по вертикали соответствует одному физическому лазерному каналу, поэтому тоннель не
искажается из-за произвольно заданного вертикального поля зрения. Для каждого пикселя
сохраняется ближайшая точка. Направления без данных остаются чёрными.

Цветовая раскраска соответствует стандартному colorizer Intel RealSense: используется
палитра Jet с покадровым выравниванием гистограммы. Ближние значения глубины отображаются
синим/голубым, средние — жёлтым, дальние — красным/тёмно-красным.

### Сборка

```bash
cd /workspaces/metro-lidar-processing
bash scripts/build.sh
source ~/ros_ws/install/local_setup.bash
```

### Запуск для пяти bag-файлов с `/lidar_points`

```bash
ros2 launch metro_lidar_processing depth_image.launch.py
```

### Запуск для `doubleT_obstacle`

```bash
ros2 launch metro_lidar_processing depth_image.launch.py \
  input_topic:=/sensing/lidar/hesai128/pointcloud \
  min_azimuth_deg:=-180.0 \
  max_azimuth_deg:=180.0 \
  point_stride:=2
```

В более крупной записи содержится 921600 точек в одном кадре. `point_stride:=2`
обрабатывает каждую вторую точку, если более высокая частота обработки важнее
максимальной угловой детализации.

### Проигрывание bag-файла

```bash
ros2 bag play \
  /data/doubleT_platform \
  --loop
```

Выходной топик — `/lidar/depth_image` типа `sensor_msgs/msg/Image` с кодировкой
`rgb8`. В RViz добавьте отображение `Image` и выберите этот топик. Для `Image`
не требуется настраивать Fixed Frame или TF.

### Запись MP4 с синхронизацией по временным меткам

Нода может напрямую записывать MP4, продолжая одновременно публиковать топик с изображением.
Положение кадров видео определяется исходными временными метками `PointCloud2`. Если обработка
в реальном времени временно не успевает за входным потоком, предыдущий кадр удерживается,
вместо того чтобы делать итоговое видео короче и быстрее.

```bash
ros2 launch metro_lidar_processing depth_image.launch.py \
  video_path:=/results/doubleT_platform_depth.mp4 \
  video_fps:=10.0
```

Запустите `ros2 bag play` один раз, без `--loop`. После окончания воспроизведения
остановите ноду глубины через Ctrl+C, чтобы она корректно завершила MP4-контейнер.

Записи содержат примерно 10 лидарных сканов в секунду. Меньшее значение, которое показывает
`ros2 topic hz /lidar/depth_image`, отражает фактическую скорость преобразования в реальном
времени, а не исходную частоту записи. Если компьютер не успевает обрабатывать 10 сканов/с,
замедлите воспроизведение так, чтобы каждый исходный кадр был обработан, сохранив при этом
исходную временную шкалу итогового видео:

```bash
ros2 bag play \
  /data/doubleT_platform \
  --rate 0.1 \
  --read-ahead-queue-size 2
```

При `--rate 0.1` обработка 88-секундного bag-файла занимает примерно 15 минут,
но итоговый MP4 всё равно длится около 88 секунд. Чтобы снизить нагрузку во время записи,
закройте RViz или отключите в нём отображение `PointCloud2`.

### Параметры

| Параметр | По умолчанию | Назначение |
|---|---:|---|
| `input_topic` | `/lidar_points` | Входной топик `PointCloud2` |
| `output_topic` | `/lidar/depth_image` | Выходной топик RGB-изображения |
| `image_width` | `320` | Ширина, соответствующая измеренному сектору 100° |
| `image_height` | `128` | Высота дальностного изображения; соответствует 128 каналам лидара |
| `min_depth` | `1.0` | Ближайшая отображаемая дальность, м |
| `max_depth` | `300.0` | Максимальная отображаемая дальность, м |
| `min_azimuth_deg` | `-140.0` | Левая граница для пяти bag-файлов с передним сектором |
| `max_azimuth_deg` | `-40.0` | Правая граница для пяти bag-файлов с передним сектором |
| `histogram_equalization` | `true` | Динамическая раскраска глубины в стиле RealSense |
| `point_stride` | `1` | Обрабатывать каждую N-ю входную точку |
| `video_path` | пусто | Необязательный путь для MP4 с синхронизацией по временным меткам |
| `video_fps` | `10.0` | Постоянная частота кадров выходного видео |
