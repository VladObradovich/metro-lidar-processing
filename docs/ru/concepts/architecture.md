(chapter-5)=
# 5 Архитектура системы

<p class="lead">Границы пакетов, потоки данных и разделение вычислений с транспортом ROS.</p>

(s-5-1)=
## 5.1 Основные потоки

<figure><img alt="Рисунок 1. Онлайн-цепочка и общий C++ алгоритм для офлайн-оценки; схема компонентов." src="/_static/diagrams/figure-01.svg"/><figcaption>Рисунок 1. Онлайн-цепочка и общий C++ алгоритм для офлайн-оценки; схема компонентов.</figcaption></figure>

<p>Онлайн-цепочка: PointCloud2 → perception_node → FrameAnalysis → obstacle_monitor_node → PathAssessment. Узел perception выполняет декодирование, преобразование, отбор ROI и геометрический анализ. Монитор принимает результаты, проверяет их порядок, подтверждает треки и выдаёт состояние пути. Визуализатор и графические инструменты подписываются на результаты, не участвуя в решении.</p>

<p>Офлайн-цепочка: rosbag2 → evaluate_bag → общий адаптер PointCloud2 + PerceptionPipeline + TemporalMonitor → frames.jsonl → metrics.py → quality.json. Повторное использование ядра удерживает семантику геометрии и решения одинаковой; транспорт, очередь и watchdog онлайн-режима в офлайн-оценщике не моделируются.</p>

(s-5-2)=
## 5.2 Пакеты

<div class="table-caption">Таблица 8. Шесть пакетов текущего дерева</div>

<table><colgroup><col style="width:29%"/><col style="width:36%"/><col style="width:35%"/></colgroup><thead><tr><th>Пакет</th><th>Содержимое</th><th>Граница ответственности</th></tr></thead><tbody><tr><td><code>metro_<wbr/>perception_<wbr/>core</code></td><td>pipeline, detector, speed_tracker, temporal_monitor, labels</td><td>C++17, собственные структуры; без ROS-сообщений и DDS.</td></tr><tr><td><code>metro_<wbr/>perception_<wbr/>interfaces</code></td><td>FrameAnalysis, PathAssessment, ObstacleCandidate, ObstacleTrack, CorridorSegment</td><td>Схема данных ROS; числовые коды и поля.</td></tr><tr><td><code>metro_<wbr/>perception_<wbr/>ros</code></td><td>Узлы, pointcloud_adapter, preprocessing, assessment_monitor, evaluate_bag</td><td>PointCloud2, YAML, TF, сессии, сериализация и ROS-транспорт.</td></tr><tr><td><code>metro_<wbr/>perception_<wbr/>bringup</code></td><td>launch/, config/, rviz/</td><td>Композиция процессов и значения запуска; вычислительного алгоритма нет.</td></tr><tr><td><code>metro_<wbr/>perception_<wbr/>tools</code></td><td>depth_image, metrics, report, inspect_bag</td><td>Python: визуализация, подсчёт качества, формирование отчёта.</td></tr><tr><td><code>metro_<wbr/>perception_<wbr/>rviz</code></td><td>assessment_text, assessment_panel</td><td>Текст и панель RViz; Qt-плагин собирается при доступном rviz_common.</td></tr></tbody></table>

<p>Хотя ранние организационные материалы говорят о пяти основных пакетах, текущее дерево дополнительно содержит metro_perception_rviz. Его текстовая библиотека собирается и без RViz. GUI-плагин зависит от наличия rviz_common при сборке. Поэтому установленный runtime-desktop следует проверить на наличие libassessment_panel.so, если панель отсутствует.</p>

(s-5-3)=
## 5.3 Latest-only обработчик

<p>Подписка perception не запускает тяжёлый алгоритм внутри callback. Она передаёт ConstSharedPtr в слот: один кадр обрабатывается worker-потоком, ещё один ожидает. Новый вход заменяет старый ожидающий кадр и увеличивает overwritten_frames. Неограниченной FIFO-очереди облаков нет.</p>

<p>Перед началом обработки проверяется локальный возраст. Перед публикацией повторно проверяются возраст и текущая сессия. Максимальный возраст по runtime.yaml — 0,30 с. Результат, который уже устарел или относится к старой сессии, отбрасывается. Перезапись pending и отказ в публикации учитываются разными счётчиками; они не являются взаимоисключающими категориями входных сообщений.</p>

<div class="table-caption">Таблица 9. Границы проверки данных</div>

<table><colgroup><col style="width:22%"/><col style="width:41%"/><col style="width:37%"/></colgroup><thead><tr><th>Этап</th><th>Функция / класс</th><th>Проверка</th></tr></thead><tbody><tr><td>Приём</td><td>PerceptionNode::on_cloud</td><td>Время приёма, sequence, скачок stamp назад, pending-slot.</td></tr><tr><td>Выбор кадра</td><td>LatestFrameSlot</td><td>Последний ожидающий кадр; остановка потока.</td></tr><tr><td>Обработка</td><td>process_cloud_with_context / PerceptionPipeline</td><td>Структура PointCloud2, TF, геометрия, детектор.</td></tr><tr><td>Публикация</td><td>SessionGate::publish_if_current</td><td>Линеаризация смены сессии и публикации.</td></tr><tr><td>Принятие анализа</td><td>FrameKeyGate / AssessmentMonitor</td><td>Источник, сессия, sequence, stamp и корректный processing_age_ms.</td></tr></tbody></table>

(s-5-4)=
## 5.4 Зависимости и размещение

<p>Core предоставляет shared library и заголовки. ROS-адаптер использует yaml-cpp, tf2/tf2_ros, rclcpp и sensor_msgs. Офлайн-оценщик использует rosbag2_cpp. Интерфейсы опираются на std_msgs, geometry_msgs и vision_msgs. Визуализация использует visualization_msgs, RViz/Qt и Python-инструменты проекции.</p>

<p>Исходники сосредоточены в src/ и include/ соответствующего пакета; unit-тесты — test/. Скрипты интеграции и исследований находятся в scripts/, наборы данных и разметка — evaluation/. Файлы rosbags/ и результаты вычислений не являются исходниками проекта и не должны попадать в коммиты.</p>

(s-5-5)=
## 5.5 Что общее и что различается онлайн/офлайн

<ul><li>Общие: проверка структуры облака, геометрический профиль, фильтрация, детектор, временное подтверждение, состояния и коды доверия.</li><li>Онлайн: QoS, очередь latest-only, возраст приёма, wall heartbeat, монитор сессий и сторожевой таймер.</li><li>Офлайн: последовательный обход облаков bag, replay TF с lookahead по времени записи, запись каждой обработанной строки в JSONL и расчёт processing_ms.</li><li>Офлайн-время кадра не равно онлайн-возрасту результата: transport и ожидание в очереди в него не входят.</li></ul>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>ros/src/perception_<wbr/>node.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/evaluate_<wbr/>bag.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/include/metro_<wbr/>perception_<wbr/>ros/latest_<wbr/>frame_<wbr/>slot.hpp</code>; <code>metro_<wbr/>perception_<wbr/>rviz/CMakeLists.txt</code>.</div>
