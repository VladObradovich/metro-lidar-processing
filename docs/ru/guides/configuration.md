(chapter-10)=
# 10 Конфигурация

<p class="lead">Аргументы launch, YAML двух типов и безопасный порядок настройки.</p>

(s-10-1)=
## 10.1 Аргументы perception.launch.py

<div class="table-caption">Таблица 27. Полный список аргументов основного launch</div>

<table><colgroup><col style="width:28%"/><col style="width:29%"/><col style="width:43%"/></colgroup><thead><tr><th>Аргумент</th><th>По умолчанию</th><th>Назначение</th></tr></thead><tbody><tr><td><code>sensor_<wbr/>profile</code></td><td><code>пустая строка</code></td><td>Путь к обычному YAML-профилю; пусто означает installed forward_sector_assumed.yaml.</td></tr><tr><td><code>namespace</code></td><td><code>metro</code></td><td>Namespace процессов и относительных выходных топиков.</td></tr><tr><td><code>input_<wbr/>topic</code></td><td><code>/lidar_<wbr/>points</code></td><td>Вход PointCloud2.</td></tr><tr><td><code>use_<wbr/>sim_<wbr/>time</code></td><td><code>false</code></td><td>Включить ROS sim time; требуется /clock.</td></tr><tr><td><code>algorithm_<wbr/>config</code></td><td><code>config/algorithm.yaml</code></td><td>ROS parameter YAML: max_points/max_cloud_bytes; не detector:-профиль.</td></tr><tr><td><code>runtime_<wbr/>config</code></td><td><code>config/runtime.yaml</code></td><td>ROS parameter YAML для QoS, возраста, TF-wait и watchdog.</td></tr><tr><td><code>rviz</code></td><td><code>false</code></td><td>Запустить labelled cloud, depth_image, visualizer и RViz.</td></tr><tr><td><code>visualizer</code></td><td><code>false</code></td><td>Публиковать маркеры без запуска RViz/depth.</td></tr><tr><td><code>fixed_<wbr/>frame</code></td><td><code>lidar_<wbr/>assumed</code></td><td>Fixed frame RViz; не задаёт frame алгоритма.</td></tr><tr><td><code>publish_<wbr/>sensor_<wbr/>tf</code></td><td><code>false</code></td><td>Опциональный static_transform_publisher в bringup.</td></tr><tr><td><code>publish_<wbr/>bound_<wbr/>transform</code></td><td><code>false</code></td><td>Опциональная публикация TF самим perception после bind_first.</td></tr><tr><td><code>sensor_<wbr/>frame_<wbr/>override</code></td><td><code>пустая строка</code></td><td>Явное имя источника для publish_sensor_tf при bind_first.</td></tr></tbody></table>

<p>demo.launch.py — обёртка прежнего демонстрационного входа; rviz там включён по умолчанию. depth_image.launch.py отдельно запускает проекцию облака и не заменяет perception/monitor. Фактические default для используемого launch можно посмотреть через ros2 launch … --show-args.</p>

(s-10-2)=
## 10.2 Два вида YAML

<div class="table-caption">Таблица 28. Не смешивать форматы конфигурации</div>

<table><colgroup><col style="width:35%"/><col style="width:34%"/><col style="width:31%"/></colgroup><thead><tr><th>Файл</th><th>Структура</th><th>Кто читает</th></tr></thead><tbody><tr><td>algorithm.yaml / runtime.yaml</td><td>/**: → ros__parameters:</td><td>ROS parameter loader launch/узлов.</td></tr><tr><td>forward_sector_assumed.yaml</td><td>source_frame_mode, transform, ROI, detector:, temporal:</td><td>load_preprocessing и load_temporal_config через yaml-cpp.</td></tr><tr><td>evaluation/dataset.yaml</td><td>bags: с topic, sensor_profile, path, annotations</td><td>evaluate_all.py и инструменты оценки.</td></tr></tbody></table>

<p>corridor_half_width_m относится к detector: в sensor_profile, а не к ROS-параметрам algorithm_config. Значения, не заданные в sensor_profile, берутся из AlgorithmConfig/TemporalConfig. Поэтому значения по умолчанию в C++ и эффективные значения shipped-профиля различаются. Полный справочник с обеими колонками — приложение А.</p>

(s-10-3)=
## 10.3 Runtime-параметры узлов

<div class="table-caption">Таблица 29. Пределы входа и свежести</div>

<table><colgroup><col style="width:35%"/><col style="width:20%"/><col style="width:45%"/></colgroup><thead><tr><th>Параметр</th><th>Default</th><th>Допустимо / смысл</th></tr></thead><tbody><tr><td><code>max_<wbr/>points</code></td><td>2 000 000</td><td>Целое [1; 10 000 000]; предел width×height.</td></tr><tr><td><code>max_<wbr/>cloud_<wbr/>bytes</code></td><td>268 435 456</td><td>Целое [1; 1 073 741 824]; предел data.size().</td></tr><tr><td><code>input_<wbr/>reliability</code></td><td>auto</td><td>auto / reliable / best_effort.</td></tr><tr><td><code>max_<wbr/>processing_<wbr/>age_<wbr/>s</code></td><td>0,30</td><td>Конечное (0;10]; срок от приёма до публикации perception.</td></tr><tr><td><code>tf_<wbr/>wait_<wbr/>timeout_<wbr/>s</code></td><td>0,05</td><td>Конечное [0;1]; bounded wait exact-stamp TF.</td></tr><tr><td><code>timeout_<wbr/>s</code></td><td>0,5</td><td>Конечное &gt;0; watchdog принятого analysis.</td></tr></tbody></table>

<p>Эти значения читаются при создании узлов. В коде нет общего callback для перестройки детектора или подписки по динамическому ros2 param set. Для воспроизводимого изменения подготовьте конфиг, перезапустите launch и зафиксируйте новый профиль в manifest прогона.</p>

(s-10-4)=
## 10.4 Создать пользовательский профиль

<div class="codebox"><div class="code-label">КОПИЯ НА ХОСТЕ, LAUNCH В ROS-КОНТЕЙНЕРЕ</div><pre># В source/dev-среде; создаёт только отдельную копию.
cp metro_perception_ros/config/forward_sector_assumed.yaml \
  results/my-sensor-profile.yaml
# Отредактируйте копию, затем запускайте внутри контейнера:
ros2 launch metro_perception_bringup perception.launch.py \
  sensor_profile:=/results/my-sensor-profile.yaml</pre></div>

<p>Для установленного runtime исходный профиль лежит в package share, а /results виден через mount. Копировать из исходного дерева удобно на хосте. Одна и та же копия должна передаваться perception и monitor: основной launch делает это автоматически через sensor_profile.</p>

(s-10-5)=
## 10.5 Основные ограничения валидации

<ul><li>ROI: три конечных числа в min/max, min&lt;max по каждой оси; detection ROI целиком вложена в geometry ROI.</li><li>Габарит: полуширина (0;5] м, высота (0;8] м; obstacle_min_height положителен и не больше высоты.</li><li>Пол: ground_max_slope (0;0,5], tolerance (0;0,5], gap [5;50] м, ground_max_x [30;200] м; ground_hold_frames≤10.</li><li>История: background_history_frames≤60; если не ноль, background_lag_frames от 1 до history−1.</li><li>Угловая ячейка: [0,05;1] градуса; min_candidate_cells и min_candidate_points положительны.</li><li>Подтверждение: 1≤hits≤window≤32 для каждой пары; release_misses от 1 до 32; max_tracks [1;1024].</li><li>Route smoothing: [0;1), route_min_radius≥20 м; значения расстояний/скоростей/допусков должны быть конечными и удовлетворять взаимным ограничениям.</li><li>assumed_clear_min_range_m должен быть конечным положительным числом; shipped значение — 50 м.</li></ul>

<aside class="note"><strong>Загрузка неизвестных ключей</strong><p>YAML-loader читает известные поля по именам; неизвестный ключ не гарантирует ошибку старта. Опечатку в параметре проверяйте по точному имени из приложения А и исходнику preprocessing.cpp. Значения валидируются после чтения, но это не строгая проверка схемы всех ключей.</p></aside>

(s-10-6)=
## 10.6 Порядок настройки

<ol><li>Зафиксировать проблему на размеченных development-данных; отделить ложную тревогу, пропуск, UNKNOWN и ошибку дальности.</li><li>Сначала проверить topic/frame/ориентацию, габарит и высоту установки. Изменение геометрии сильнее влияет на решение, чем отдельный порог.</li><li>Менять ограниченное число параметров за один вариант; сохранять копию профиля и предыдущий baseline.</li><li>Сравнивать object hits, ложные события, долю UNKNOWN, working range и задержку. Один frame recall недостаточен.</li><li>Профиль выбирать по сенсору/геометрии; исключить условия вида «для bag X другой порог».</li><li>После принятия варианта выполнить последовательный воспроизводимый прогон на чистом checkout; слепую часть смотреть только суммарно.</li></ol>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>bringup/launch/perception.launch.py</code>; <code>metro_<wbr/>perception_<wbr/>bringup/config/runtime.yaml</code>; <code>metro_<wbr/>perception_<wbr/>core/include/metro_<wbr/>perception_<wbr/>core/config.hpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/preprocessing.cpp</code>.</div>
