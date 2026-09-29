(chapter-3)=
# 3 Среды, установка и доставка

<p class="lead">Выбор между установленным runtime, контейнером разработки и графической средой.</p>

(s-3-1)=
## 3.1 Варианты среды

<div class="table-caption">Таблица 4. Среды проекта</div>

<table><colgroup><col style="width:23%"/><col style="width:30%"/><col style="width:47%"/></colgroup><thead><tr><th>Вариант</th><th>Назначение</th><th>Исходники / сборка</th></tr></thead><tbody><tr><td>runtime</td><td>Детектор и офлайн-инструменты без GUI</td><td>docker/Dockerfile.runtime, target runtime; пакеты установлены в образе.</td></tr><tr><td>runtime-desktop</td><td>RViz, depth-видео и текстовая панель</td><td>Тот же Dockerfile, target runtime-desktop; графические библиотеки и RViz.</td></tr><tr><td>universal (dev)</td><td>Сборка, отладка, тесты, измеренные прогоны</td><td>docker/Dockerfile, target universal; исходники монтируются и собираются отдельно.</td></tr><tr><td>desktop (dev)</td><td>Разработка с графикой</td><td>docker/Dockerfile, target desktop; инструменты dev и RViz.</td></tr><tr><td>Dev Container</td><td>Работа из редактора</td><td>.devcontainer/universal и .devcontainer/desktop; выберите нужную конфигурацию при открытии.</td></tr></tbody></table>

<p>Базовая среда — ros:humble-ros-base. Разработческий образ содержит компиляторы, CMake, colcon, rosdep, clang-format/clang-tidy, gdb, инструменты тестирования и Python-пакеты анализа. Рабочий runtime использует CycloneDDS, ROS_DOMAIN_ID=42 и ROS_LOCALHOST_ONLY=1. При этой конфигурации команды взаимодействия с ROS запускаются внутри того же контейнера.</p>

(s-3-2)=
## 3.2 Установленный runtime через Compose

<div class="codebox"><div class="code-label">ХОСТ</div><pre>export METRO_UID="$(id -u)"
export METRO_GID="$(id -g)"
# При другом расположении bag можно задать абсолютный путь:
# export METRO_BAGS_DIR=/absolute/path/to/rosbags
mkdir -p results
docker compose -f compose.local.yaml up -d --build
docker compose -f compose.local.yaml exec local metro-entrypoint \
  ros2 launch metro_perception_bringup perception.launch.py</pre></div>

<p>Сервис local остаётся запущенным с командой sleep infinity. Во втором терминале используйте тот же exec local metro-entrypoint для ros2 bag play или topic echo. Монтируются только данные /data в режиме read-only и /results для вывода; исходники не монтируются.</p>

<div class="codebox"><div class="code-label">ХОСТ</div><pre>docker compose -f compose.local.yaml exec local metro-entrypoint \
  ros2 bag play /data/doubleT_platform
# Выполнить после завершения работы:
docker compose -f compose.local.yaml down</pre></div>

(s-3-3)=
## 3.3 Контейнер разработки

<div class="codebox"><div class="code-label">ХОСТ</div><pre>docker build -f docker/Dockerfile --target universal \
  -t metro-lidar:dev docker
docker run --rm -it --init \
  --user "$(id -u):$(id -g)" \
  -v "$PWD:/ws" -v "$PWD/rosbags:/data:ro" \
  -v "$PWD/results:/results" \
  metro-lidar:dev bash</pre></div>

<div class="codebox"><div class="code-label">ВНУТРИ DEV-КОНТЕЙНЕРА</div><pre>cd /ws
bash scripts/build.sh
source install/local_setup.bash
bash scripts/test.sh</pre></div>

<p>build.sh явно загружает Humble и собирает пакеты metro_perception_* с RelWithDebInfo и compile_commands.json. test.sh запускает pytest для scripts/test, затем colcon test и проверку colcon test-result. Сборка появляется в смонтированном рабочем каталоге: build/, install/ и log/. После открытия нового терминала повторно загрузите install/local_setup.bash.</p>

(s-3-4)=
## 3.4 Работа на стенде без интернета

<p>Подготовьте образы заранее на машине с сетью. Сборка требует Docker Hub, apt и pip; работа уже установленного образа не требует загрузки зависимостей.</p>

<div class="codebox"><div class="code-label">МАШИНА С ИНТЕРНЕТОМ</div><pre>docker build -f docker/Dockerfile.runtime --target runtime \
  -t metro-lidar:local .
docker build -f docker/Dockerfile.runtime --target runtime-desktop \
  -t metro-lidar:desktop .
bash scripts/export_images.sh</pre></div>

<p>Скрипт сохраняет архивы образов и .sha256 в results/images/. Перенесите оба архива при необходимости графики, контрольные суммы и bag-файлы. На стенде поместите архивы и файлы .sha256 в один каталог и проверьте контрольные суммы до загрузки.</p>

<div class="codebox"><div class="code-label">ОФЛАЙН-СТЕНД: КАТАЛОГ С АРХИВАМИ</div><pre>sha256sum -c metro-lidar-local.tar.gz.sha256
docker load -i metro-lidar-local.tar.gz
# Для графического варианта:
sha256sum -c metro-lidar-desktop.tar.gz.sha256
docker load -i metro-lidar-desktop.tar.gz</pre></div>

<p>Для headless-запуска используйте docker run из главы 2. Для Compose после загрузки образа допустим up -d --no-build. scripts/desktop.sh up по умолчанию не заставляет пересобирать образ; не передавайте --build на стенде без сети. CI проверяет runtime с --network none, однако это не заменяет проверку целевой машины.</p>

(s-3-5)=
## 3.5 Файлы, права и ресурсы

<ul><li>Входные bag монтируйте read-only: это предохраняет исходные записи от случайного изменения инструментами.</li><li>UID/GID контейнера должны позволять запись в results/. Для docker run используйте --user, для compose.local.yaml — METRO_UID/METRO_GID.</li><li>Не измеряйте задержку параллельно с тяжёлыми сборками, записью видео или обработкой нескольких bag: нагрузка меняет хвосты времени.</li><li>Размер облака ограничивается max_points и max_cloud_bytes. Значения по умолчанию: 2 000 000 точек и 256 MiB.</li><li>Минимальные RAM и CPU как обязательные требования в репозитории не установлены. Фактическое потребление следует измерить на целевом стенде.</li></ul>

<div class="source">Основание: <code>compose.local.yaml</code>; <code>docker/Dockerfile</code>; <code>docker/Dockerfile.runtime</code>; <code>scripts/export_<wbr/>images.sh</code>; <code>scripts/build.sh</code>; <code>scripts/test.sh</code>.</div>
