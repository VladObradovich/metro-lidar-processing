(chapter-2)=
# 2 Быстрый старт

<p class="lead">Полный путь от подготовленных bag-файлов до первого сообщения о состоянии пути.</p>

(s-2-1)=
## 2.1 Предварительная проверка

<p>Нужны Docker Engine с Compose v2, доступ к интернету для первой сборки и распакованные записи rosbag2. На хосте ROS не требуется. Входные данные не поставляются как часть исходников: каждый bag должен лежать в отдельном каталоге с metadata.yaml и файлами хранилища, обычно .db3.</p>

<div class="codebox"><div class="code-label">ХОСТ: КОРЕНЬ РЕПОЗИТОРИЯ</div><pre>docker --version
docker compose version
mkdir -p results
ls rosbags/doubleT_platform/metadata.yaml</pre></div>

<p>Проверка Docker должна завершиться без ошибки соединения с daemon. Путь к metadata.yaml должен существовать. Если данные расположены в другом каталоге, используйте его абсолютный путь вместо $PWD/rosbags или переменную METRO_BAGS_DIR при запуске Compose.</p>

(s-2-2)=
## 2.2 Сборка установленного runtime

<div class="codebox"><div class="code-label">ХОСТ</div><pre>docker build -f docker/Dockerfile.runtime \
  --target runtime -t metro-lidar:local .</pre></div>

<p>Образ содержит Release-сборку пакетов в /opt/metro/install и не требует монтирования исходников. runtime предназначен для командной строки; runtime-desktop дополнительно содержит RViz. Первая сборка загружает базовый образ и зависимости; последующие используют слои Docker.</p>

(s-2-3)=
## 2.3 Запуск обработки

<div class="codebox"><div class="code-label">ХОСТ: ТЕРМИНАЛ 1</div><pre>docker run --rm --init --name metro-demo \
  --user "$(id -u):$(id -g)" \
  -v "$PWD/rosbags:/data:ro" \
  -v "$PWD/results:/results" \
  metro-lidar:local \
  ros2 launch metro_perception_bringup perception.launch.py</pre></div>

<p>Оставьте терминал открытым. Launch поднимет узлы perception и obstacle_monitor в namespace metro. Пока облако не поступило, assessment имеет UNKNOWN, причину WAITING_FOR_INPUT и stale=true. Штатный вход — /lidar_points.</p>

(s-2-4)=
## 2.4 Воспроизведение записи

<div class="codebox"><div class="code-label">ХОСТ: ТЕРМИНАЛ 2</div><pre>docker exec metro-demo metro-entrypoint \
  ros2 bag info /data/doubleT_platform
docker exec metro-demo metro-entrypoint \
  ros2 bag play /data/doubleT_platform</pre></div>

<p>Информация о bag должна содержать sensor_msgs/msg/PointCloud2 на /lidar_points. Во время проигрывания счётчик кадров растёт, а /metro/analysis и /metro/assessment публикуют сообщения. В конце записи watchdog переводит оценку в UNKNOWN с причиной INPUT_PAUSED_OR_STOPPED.</p>

(s-2-5)=
## 2.5 Чтение результата

<div class="codebox"><div class="code-label">ХОСТ: ТЕРМИНАЛ 3; ЗАПУСКАТЬ ПО ОДНОЙ</div><pre>docker exec metro-demo metro-entrypoint \
  ros2 topic echo /metro/assessment
docker exec metro-demo metro-entrypoint \
  ros2 topic hz /metro/analysis</pre></div>

<div class="table-caption">Таблица 3. Контроль первого запуска</div>

<table><colgroup><col style="width:30%"/><col style="width:70%"/></colgroup><thead><tr><th>Что проверить</th><th>Ожидаемое поведение</th></tr></thead><tbody><tr><td>state</td><td>0 — UNKNOWN; 1 — OBSTACLE; 2 — NO_OBSTACLE_DETECTED. Все три значения возможны и требуют чтения reason.</td></tr><tr><td>distance_valid</td><td>true только при пригодной дистанции до подтверждённого трека; иначе distance_m=NaN.</td></tr><tr><td>calibration_trust</td><td>Для штатного предполагаемого профиля обычно 1 (ASSUMED).</td></tr><tr><td>stale</td><td>false при принятом свежем анализе; true после пропадания входа / устаревания результата.</td></tr><tr><td>evaluated_range_m</td><td>Фактически оценённая область, а не максимальное значение ROI.</td></tr></tbody></table>

<aside class="note"><strong>Условие успеха</strong><p>Успех первого запуска — получение свежих analysis и assessment при воспроизведении, а также переход в stale после остановки входа. Добиваться ответа «свободно» изменением порогов для проверки запуска не требуется.</p></aside>

(s-2-6)=
## 2.6 Запись с другим входным топиком

<p>Для doubleT_obstacle задан топик /sensing/lidar/hesai128/pointcloud. Остановите текущий launch сочетанием Ctrl+C и запустите его с нужным аргументом. Имя bag используется только для выбора входа; профиль выбирают по сенсору и принятой геометрической гипотезе.</p>

<div class="codebox"><div class="code-label">ХОСТ: ТЕРМИНАЛ 1</div><pre>docker run --rm --init --name metro-demo \
  --user "$(id -u):$(id -g)" \
  -v "$PWD/rosbags:/data:ro" -v "$PWD/results:/results" \
  metro-lidar:local \
  ros2 launch metro_perception_bringup perception.launch.py \
  input_topic:=/sensing/lidar/hesai128/pointcloud</pre></div>

<p>Для нового полного скана неизвестной ориентации используйте full_scan_unresolved.yaml и сохраняйте UNKNOWN. Совпадение названия модели сенсора не доказывает направление установки. Работа с профилями подробно описана в главе 6.</p>

(s-2-7)=
## 2.7 Остановка

<p>Ctrl+C в терминале проигрывателя останавливает bag; launch продолжает выдавать heartbeat, после тайм-аута — stale. Ctrl+C в терминале launch завершает узлы и удаляет контейнер, созданный с --rm. Для Compose используется отдельная команда down из главы 3.</p>

<div class="source">Основание: <code>docker/Dockerfile.runtime</code>; <code>metro_<wbr/>perception_<wbr/>bringup/launch/perception.launch.py</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/assessment_<wbr/>monitor.cpp</code>.</div>
