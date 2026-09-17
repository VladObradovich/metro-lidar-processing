# metro-lidar-processing

ROS 2-пакет обработки лидарных записей для хакатона. Текущая нода преобразует
`PointCloud2` в панорамное изображение глубины и при необходимости сохраняет видео.
Обнаружение препятствий пока не реализовано.

## Быстрый запуск на своём датасете

Для запуска нужен Docker. ROS, Python-зависимости и собранный пакет находятся
в образе; VS Code и ROS на хосте не нужны. Основной сценарий — Linux с Docker
Engine. На Windows команды выполняются в WSL2 с интеграцией Docker Desktop;
пути к данным должны быть доступны из WSL.

### 1. Собрать образ один раз

Из корня этого репозитория:

```bash
docker build -f docker/Dockerfile.runtime --target runtime -t metro-lidar:local .
```

Образ использует Ubuntu 22.04 / ROS 2 Humble. Зависимости устанавливаются и пакет
собирается при `docker build`. При изменении кода образ нужно пересобрать.
При запуске ничего скачивать или собирать не требуется. Данные в образ не входят.

### 2. Подключить свой датасет и открыть контейнер

Замените `/absolute/path/to/dataset` на **существующий абсолютный путь** к своему
каталогу с bag-файлами. Его расположение и имя на хосте произвольные:

```bash
docker run --rm -it --init --name metro-lidar \
  --mount "type=bind,source=/absolute/path/to/dataset,target=/data,readonly" \
  metro-lidar:local
```

Например, запись `/absolute/path/to/dataset/my_bag/metadata.yaml` будет доступна
как `/data/my_bag/metadata.yaml`. Подключайте каталог вместе с `metadata.yaml`
и всеми файлами `.db3`, указанными в метаданных. Предоставленные записи имеют
формат rosbag2 SQLite3. Входной каталог подключается только для чтения.

### 3. Запустить обработку в контейнере

```bash
exec ros2 launch -n metro_lidar_processing depth_image.launch.py
```

### 4. Проиграть свою запись из второго терминала хоста

Замените `my_bag` на имя записи в своём датасете:

```bash
docker exec -it metro-lidar /usr/local/bin/metro-entrypoint \
  ros2 bag play /data/my_bag
```

Имена bag не влияют на настройки алгоритма. Если в `/data` смонтирован сам каталог
одной записи, используйте `ros2 bag play /data`.
Обработка и проигрывание работают в одном контейнере; настройка ROS-сети хоста
не требуется. Обёртка `metro-entrypoint` нужна для `docker exec`, потому что Docker
не вызывает entrypoint образа при exec автоматически.

Для просмотра метаданных записи и результата:

```bash
docker exec metro-lidar /usr/local/bin/metro-entrypoint ros2 bag info /data/my_bag
docker exec metro-lidar /usr/local/bin/metro-entrypoint \
  ros2 topic echo /lidar/depth_image --once --no-arr
```

Нода ждёт сообщения до начала проигрывания. `exec` заменяет оболочку процессом
launch, а `-n` включает штатный noninteractive-режим ROS launch, который сам
передаёт сигнал дочерним процессам. Это позволяет `docker stop metro-lidar`
мягко остановить ноду и закрыть видеофайл. Для завершения также можно нажать Ctrl+C в терминале launch;
контейнер завершится и удалится. Если запустить launch без `exec`, сначала
остановите его через Ctrl+C, затем выйдите из оболочки командой `exit`.

## Входные данные и параметры

Вход: `sensor_msgs/msg/PointCloud2`, поля `x`, `y`, `z` — FLOAT32, `ring` — UINT16.
Текущая подписка использует Reliable QoS; предоставленные записи совместимы.
Для нового источника требуется проверить поля и QoS, а не только имя топика.
Выход: `sensor_msgs/msg/Image` в `/lidar/depth_image`.

| Аргумент launch | По умолчанию | Назначение |
|---|---|---|
| `input_topic` | `/lidar_points` | Входной топик из `ros2 bag info` |
| `output_topic` | `/lidar/depth_image` | Изображение глубины |
| `min_azimuth_deg`, `max_azimuth_deg` | `-140.0`, `-40.0` | Сектор проекции в градусах |
| `min_depth`, `max_depth` | `1.0`, `300.0` | Диапазон расстояний в метрах |
| `image_width`, `image_height` | `320`, `128` | Размер изображения |
| `point_stride` | `1` | Шаг выборки точек |
| `histogram_equalization` | `true` | Выравнивание гистограммы раскраски |
| `video_path`, `video_fps` | пустой путь, `10.0` | Запись видео; пустой путь отключает её |
| `rviz` | `false` | Открыть RViz, нужен desktop-образ |
| `fixed_frame` | `hesai_lidar` | Fixed Frame RViz, равный `header.frame_id` облака |

Для пяти предоставленных записей с `/lidar_points` подходят значения по умолчанию.
Для `doubleT_obstacle`:

```bash
exec ros2 launch -n metro_lidar_processing depth_image.launch.py \
  input_topic:=/sensing/lidar/hesai128/pointcloud \
  min_azimuth_deg:=-180.0 max_azimuth_deg:=180.0 point_stride:=2
```

Для неизвестного датасета задайте его топик и подходящий сектор через аргументы
launch. Пересборка образа для смены датасета или этих параметров не нужна.

## Сохранение видео на хост

Перед запуском создайте каталог результатов и добавьте его к `docker run`.
UID/GID пользователя хоста обеспечивают доступ к файлам без последующего `sudo`:

```bash
mkdir -p results
docker run --rm -it --init --name metro-lidar \
  --user "$(id -u):$(id -g)" \
  --mount "type=bind,source=/absolute/path/to/dataset,target=/data,readonly" \
  --mount "type=bind,source=$PWD/results,target=/results" \
  metro-lidar:local
```

В контейнере:

```bash
exec ros2 launch -n metro_lidar_processing depth_image.launch.py \
  video_path:=/results/depth.mp4
```

Запустите bag из второго терминала как выше. После проигрывания остановите launch
через Ctrl+C, чтобы закрыть видеофайл. Результат останется в `results/depth.mp4`
после удаления контейнера. Это видео панорамы глубины, не результат детекции.

## Необязательная визуализация в RViz

Headless-образ не требует дисплея и GPU. Для RViz соберите отдельный вариант:

```bash
docker build -f docker/Dockerfile.runtime --target runtime-desktop \
  -t metro-lidar:desktop .
```

В графической Linux-сессии с X11/XWayland или WSLg выполните из корня репозитория
(нужен Python 3 на хосте; на Linux также `xhost`):

```bash
python3 .devcontainer/scripts/x11_proxy.py start
docker run --rm -it --init --name metro-lidar \
  --user "$(id -u):$(id -g)" \
  -e DISPLAY="$DISPLAY" \
  --mount "type=bind,source=$PWD/.devcontainer/.runtime/x11,target=/tmp/.X11-unix" \
  --mount "type=bind,source=/absolute/path/to/dataset,target=/data,readonly" \
  metro-lidar:desktop
```

В контейнере запустите обработку с `rviz:=true`. Готовый конфиг показывает облако
и изображение; топики следуют аргументам `input_topic` и `output_topic`:

```bash
exec ros2 launch -n metro_lidar_processing depth_image.launch.py rviz:=true
```

Для `doubleT_obstacle` добавьте к команде обработки выше
`rviz:=true fixed_frame:=lidar_livox`. Для другого источника укажите его
`header.frame_id` через `fixed_frame` (посмотреть можно в сообщении PointCloud2).
После начала `ros2 bag play` в RViz появятся данные. Используется программный
OpenGL и не требует доступа к GPU из контейнера.

После выхода из графического контейнера остановите прокси:

```bash
python3 .devcontainer/scripts/x11_proxy.py stop
```

## Масштаб интерфейса RViz (HiDPI)

При старте X11-прокси масштаб определяется на **хосте** и передаётся в контейнер
через файл в уже подключённом каталоге X11. Это работает и для готового
Desktop-образа, и для обоих Desktop Dev Containers. На хосте не меняются настройки
монитора или рабочего стола.

Приоритет: `start --scale` → `METRO_QT_SCALE_FACTOR` → `QT_SCALE_FACTOR` хоста →
масштаб активного монитора Niri/Sway/Hyprland → `GDK_SCALE` → `Xft.dpi / 96` → `1`.
В Niri используется монитор сфокусированного рабочего пространства. Для другого
монитора можно передать `--output DP-1`. Если фокус неизвестен и масштабы мониторов
различаются, скрипт не выбирает произвольный монитор. Источник и значение масштаба
выводятся при запуске прокси.

Ручной выбор (в том числе для WSLg и композиторов без доступного источника масштаба):

```bash
python3 .devcontainer/scripts/x11_proxy.py start --scale 1.5
```

Для Dev Containers можно задать `METRO_QT_SCALE_FACTOR=1.5` в окружении VS Code
перед открытием контейнера. Для ручного `docker run` переменная
`-e QT_SCALE_FACTOR=1.5` переопределяет переданный хостом масштаб.
Поддерживаются значения от `0.5` до `4`, включая дробные.

После обновления файлов пересоберите Desktop-образ или выполните **Rebuild
Container**. Масштаб выбирается при старте прокси; при смене монитора перезапустите
прокси и RViz (в Dev Containers откройте новый терминал). Уже открытое окно RViz
не меняет масштаб автоматически при переносе между экранами.
Для обычного X11 без прокси и без явного масштаба настройки Qt не переопределяются.
Используется [QT_SCALE_FACTOR](https://doc.qt.io/archives/qt-5.15/highdpi.html),
в режиме `METRO_QT_SCALING_MODE=full`. По умолчанию включён режим `font`:
`QT_SCALE_FACTOR=1`, а `QT_FONT_DPI=96 × масштаб`. Так текст и зависящие от шрифта
элементы остаются крупными, но OpenGL-область не масштабируется. Это обход
[мерцания RViz Humble при HiDPI](https://github.com/ros2/rviz/issues/1052).
Некоторые иконки в этом режиме могут оставаться небольшими. Полное масштабирование
можно явно включить через `-e METRO_QT_SCALING_MODE=full`, если на вашем оборудовании
оно не вызывает мерцания. Для переопределения желаемого размера используйте
`METRO_QT_SCALE_FACTOR` (например, `-e METRO_QT_SCALE_FACTOR=1.5`).

Если контейнер уже открыт, до пересборки можно закрыть RViz и проверить обход:

```bash
QT_SCALE_FACTOR=1 QT_SCREEN_SCALE_FACTORS=1 QT_AUTO_SCREEN_SCALE_FACTOR=0 \
  QT_ENABLE_HIGHDPI_SCALING=0 QT_FONT_DPI=192 rviz2
```

Значение `192` в этой разовой команде соответствует экрану с масштабом 2×.
Автоматическая настройка в образе вычисляет DPI из масштаба хоста.

## Проверка готового образа

Из корня репозитория можно проверить обработку внешней записи без сети и без
монтирования исходников. Сам проверочный скрипт передаётся через stdin:

```bash
mkdir -p results
docker run --rm -i --init --network none \
  --user "$(id -u):$(id -g)" \
  --mount "type=bind,source=/absolute/path/to/dataset,target=/data,readonly" \
  --mount "type=bind,source=$PWD/results,target=/results" \
  metro-lidar:local python3 - /data/my_bag \
  --video /results/smoke.mp4 < scripts/smoke_runtime.py
```

Проверка ждёт готовности ноды, проигрывает начало записи, получает пять непустых
изображений, останавливает процессы через SIGINT и декодирует сохранённое видео.
Для второго формата входа добавьте `--input-topic /sensing/lidar/hesai128/pointcloud
--min-azimuth -180.0 --max-azimuth 180.0`. Для повторного запуска задайте новый
`--video`: проверка не перезаписывает существующие результаты.

## Разработка

Далее описана среда разработки с монтированием исходников. Для запуска готового
решения достаточно разделов выше. Образы `metro-lidar-dev:*` содержат инструменты
разработки, а `metro-lidar:local` и `metro-lidar:desktop` — установленный пакет.

## Установка и разработка в VS Code

Для проекта используется Dev Container **Ubuntu 22.04 / ROS 2 Humble** с двумя
профилями. Общая часть включает Python 3.10, C++/CMake, Eigen/PCL,
NumPy/SciPy/OpenCV, rosbag2, Cyclone DDS, отладчик и средства тестирования.

| Профиль | Назначение | Хост |
|---|---|---|
| **Universal (Server, Headless)** | Сборка, тесты и обработка bag без GUI | Прежде всего Linux-серверы и CI |
| **Desktop (Software Graphics)** | Полная среда с RViz и программным OpenGL | Linux через XWayland и Windows через WSL2/WSLg |

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

Ошибка `docker: unknown command: docker buildx` в логе Dev Containers означает,
что на хосте нет Buildx-плагина. Dev Containers умеет продолжить legacy-сборкой;
если ниже присутствует `Successfully built`, эта строка не является причиной
падения. Чтобы убрать предупреждение и сохранить поддержку будущих версий
Docker, установите Buildx-плагин из пакетов своего дистрибутива.

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

Desktop-профиль автоматически запускает на хосте локальный X11-прокси. Контейнер
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

`Desktop (Software Graphics)` задаёт `LIBGL_ALWAYS_SOFTWARE=1` и работает без
доступа к GPU из контейнера.

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
bash scripts/build.sh
source ~/metro_ws/install/local_setup.bash
```

При ручном запуске с другим UID/GID передайте при сборке
`--build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)"`. Для Desktop
нужно дополнительно передать X11-сокет и переменные из соответствующего
`devcontainer.json`.

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
