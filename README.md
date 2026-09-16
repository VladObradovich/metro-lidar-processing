# metro-lidar-processing

ROS 2 package for processing the hackathon metro lidar recordings.

## Установка и разработка в VS Code

Для проекта используется Dev Container **Ubuntu 22.04 / ROS 2 Humble** с тремя
профилями. Общая часть включает Python 3.10, C++/CMake, Eigen/PCL,
NumPy/SciPy/OpenCV, rosbag2, Cyclone DDS, отладчик и средства тестирования.

| Профиль | Назначение | Хост |
|---|---|---|
| **Universal (Server, Headless)** | Сборка, тесты и обработка bag без GUI | Прежде всего Linux-серверы и CI |
| **Desktop (Software Graphics)** | Полная среда с RViz и программным OpenGL | Linux через XWayland и Windows через WSL2/WSLg |
| **Desktop (NVIDIA GPU)** | RViz с аппаратным ускорением NVIDIA | Linux или Windows/WSL2 с NVIDIA |

### 1. Подготовить хост

На Linux нужны Docker Engine, VS Code и расширение **Dev Containers**
(`ms-vscode-remote.remote-containers`). ROS на хосте не нужен. Проверьте Docker:

```bash
docker context show
docker version
docker ps
code --install-extension ms-vscode-remote.remote-containers
```

На Windows используйте Docker Desktop с WSL2 backend и включённой интеграцией
с выбранным WSL-дистрибутивом. Установите расширения **WSL** и
**Dev Containers**, храните репозиторий и данные в файловой системе WSL, а не
на диске `C:`. Открывайте проект из терминала WSL:

```bash
cd ~/hahaton/metro-lidar-processing
code .
```

Ожидаемое расположение данных:

```text
hahaton/
├── metro-lidar-processing/   # Открывать эту папку в VS Code
│   ├── .devcontainer/
│   ├── docker/
│   └── scripts/
├── archive/for_hackathon/    # Распакованные bag-файлы
└── videos/                   # Результаты и видео
```

Оба соседних каталога должны существовать до открытия контейнера. При другом
расположении измените `mounts` в выбранном файле
`.devcontainer/<profile>/devcontainer.json`.

### 2. Выбрать и открыть профиль

1. Если VS Code подключён к другому контейнеру, выполните
   **Dev Containers: Reopen Folder Locally**.
2. Откройте **`metro-lidar-processing`**.
3. Выполните **Dev Containers: Reopen in Container**.
4. Выберите один из трёх профилей. Для сервера без GUI выбирайте `Universal`,
   для обычного рабочего компьютера — `Desktop (Software Graphics)`.
5. Дождитесь сборки образа и выполнения `postCreateCommand`. Первая сборка
   скачивает несколько гигабайт зависимостей.
6. В новом терминале проверьте:

```bash
echo "$ROS_DISTRO"                 # humble
python3 --version                  # Python 3.10.x
ros2 pkg prefix metro_lidar_processing
```

### 3. Где находятся файлы

| В контейнере | Назначение |
|---|---|
| `~/metro_ws/src/metro-lidar-processing` | Исходники с хоста; изменения сразу видны в Git |
| `/data` | `archive/for_hackathon`, подключён только для чтения |
| `/results` | Каталог `videos` на хосте; результаты сохраняются после пересоздания |
| `~/metro_ws/build`, `install`, `log` | Сборка Humble внутри контейнера |

Сборочные каталоги внутри контейнера могут исчезнуть при **Rebuild Container**;
они создаются повторно автоматически. Исходники, bag и `/results` сохраняются.
Старые `build/install/log` в репозитории не используются и не удаляются.

Процессы работают от пользователя `dev` с фиксированными UID/GID `1000:1000`.
Для установки дополнительных инструментов есть `sudo`. Постоянные зависимости добавляйте в Dockerfile, затем выполняйте
**Dev Containers: Rebuild Container**. На Linux пользователь хоста должен иметь
UID/GID `1000:1000`, иначе у bind-mounted файлов могут отличаться права.

### 4. Сборка и тесты

Во всех профилях из корня репозитория доступны одинаковые команды:

```bash
bash scripts/build.sh
source ~/metro_ws/install/local_setup.bash
bash scripts/test.sh
```

В новых интерактивных терминалах Humble и готовый workspace подключаются
автоматически. Новые зависимости ROS устанавливайте внутри контейнера:

```bash
rosdep update --rosdistro humble
rosdep install --from-paths ~/metro_ws/src --ignore-src --rosdistro humble -y
```

Для воспроизводимости отразите зависимости в `package.xml` и Dockerfile. Не
подключайте старый `install/setup.bash` из Jazzy.

### 5. RViz на Linux и Windows

RViz установлен только в профилях `Desktop`. RViz из ROS 2 Humble использует
OGRE с GLX, поэтому профили передают X11/XWayland-сокет и `DISPLAY`, даже если
рабочий стол хоста использует Wayland. Профиль `Universal` не зависит от дисплея
и не содержит RViz.

На Linux перед открытием Desktop-профиля проверьте:

```bash
echo "$DISPLAY"                    # например :0 или :1
ls -l /tmp/.X11-unix
```

На Windows обновите WSL (`wsl --update` в PowerShell), включите WSL Integration
в Docker Desktop и запускайте VS Code командой `code .` из WSL. WSLg должен
предоставить `DISPLAY` и каталог `/tmp/.X11-unix` в WSL-сессии.

Desktop-профили автоматически запускают на хосте локальный X11-прокси. Контейнер
подключается к сокету в `.devcontainer/.runtime/x11`, поэтому один и тот же
профиль работает с обычным Docker Engine, Snap Docker и Docker Desktop/WSLg.
Runtime-каталог добавлен в `.gitignore` и не попадает в репозиторий. Если
графическая сессия или XWayland недоступны, создание контейнера остановится с
понятным сообщением `X11 proxy` вместо последующей ошибки Qt.

Внутри Desktop-контейнера запустите:

```bash
rviz2
```

Для изображения глубины добавьте `Image`, топик `/lidar/depth_image`. Для
3D-облака добавьте `PointCloud2`, выберите входной топик и установите Fixed Frame
по его `header.frame_id`: `hesai_lidar` либо `lidar_livox`.

`Desktop (Software Graphics)` задаёт `LIBGL_ALWAYS_SOFTWARE=1` и не требует GPU.
`Desktop (NVIDIA GPU)` запускается с `--gpus=all` и требует заранее настроенной
поддержки NVIDIA Container Toolkit на Linux либо NVIDIA GPU в Docker Desktop с
WSL2 на Windows. CUDA в образ не установлена: текущий алгоритм её не использует.

### 6. ROS-сеть

Во всех профилях по умолчанию используются обычная bridge-сеть,
`ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1` и
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Это подходит, когда bag и обработка
запущены внутри одного контейнера. Domain ID не является механизмом защиты.

Для физического лидара или ROS-узлов на других компьютерах потребуется отдельная
сетевая конфигурация: отключить localhost-only и настроить Cyclone DDS либо явно
включить host networking на поддерживаемом хосте.

### 7. Сборка образа без VS Code

Dockerfile содержит targets `universal` и `desktop`:

```bash
docker build --target universal -t metro-lidar-dev:universal docker
docker build --target desktop -t metro-lidar-dev:desktop docker
```

Без `--target` собирается headless-вариант. Контекст сборки — только каталог
`docker`: исходники, архивы, bag-файлы и видео в образ не копируются.

Ручной запуск Universal-профиля:

```bash
docker run -d --init --name metro-lidar-dev --shm-size 1g \
  --mount "type=bind,source=$PWD,target=/home/dev/metro_ws/src/metro-lidar-processing" \
  --mount "type=bind,source=$PWD/../archive/for_hackathon,target=/data,readonly" \
  --mount "type=bind,source=$PWD/../videos,target=/results" \
  metro-lidar-dev:universal
docker exec -it metro-lidar-dev bash
```

При ручном запуске с другим UID/GID передайте при сборке
`--build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)"`. Для Desktop
нужно дополнительно передать X11-сокет и переменные из соответствующего
`devcontainer.json`; для NVIDIA также добавляется `--gpus=all`.

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
cd ~/metro_ws/src/metro-lidar-processing
bash scripts/build.sh
source ~/metro_ws/install/local_setup.bash
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
