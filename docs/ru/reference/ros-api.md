(chapter-9)=
# 9 ROS API и сообщения

<p class="lead">Топики, условия публикации и полный справочник полей интерфейсов.</p>

(s-9-1)=
## 9.1 Топики стандартного launch

<div class="table-caption">Таблица 20. Публичные топики при namespace=metro</div>

<table><colgroup><col style="width:29%"/><col style="width:32%"/><col style="width:39%"/></colgroup><thead><tr><th>Топик</th><th>Тип</th><th>Назначение / условие</th></tr></thead><tbody><tr><td><code>/lidar_<wbr/>points</code></td><td>sensor_msgs/msg/PointCloud2</td><td>Вход по умолчанию; заменяется input_topic.</td></tr><tr><td><code>/metro/analysis</code></td><td>metro_perception_interfaces/msg/FrameAnalysis</td><td>Принятый результат обработки кадра.</td></tr><tr><td><code>/metro/assessment</code></td><td>metro_perception_interfaces/msg/PathAssessment</td><td>Решение и wall heartbeat; основной интерфейс.</td></tr><tr><td><code>/metro/markers</code></td><td>visualization_msgs/msg/MarkerArray</td><td>Полные маркеры; visualizer=true либо rviz=true.</td></tr><tr><td><code>/metro/corridor_<wbr/>markers</code></td><td>visualization_msgs/msg/MarkerArray</td><td>Линии габарита; тот же visualizer.</td></tr><tr><td><code>/metro/labelled_<wbr/>points</code></td><td>sensor_msgs/msg/PointCloud2</td><td>Очищенное преобразованное облако с rgb/label; стандартный launch включает при rviz=true.</td></tr><tr><td><code>/metro/depth_<wbr/>image</code></td><td>sensor_msgs/msg/Image</td><td>Проекция labelled_points; стандартный launch включает при rviz=true.</td></tr></tbody></table>

<p>Имена входов/выходов узлов изначально private (~/input/…, ~/output/…). Launch remap связывает их относительными analysis/assessment, а namespace задаёт общий префикс. При namespace:=train1 выход становится /train1/assessment. Изоляция namespace проверяется integration smoke.</p>

(s-9-2)=
## 9.2 QoS

<p>Вход perception использует SensorDataQoS с keep_last(1). input_reliability=auto начинает с reliable, подходящего для rosbag2, и до первого принятого облака проверяет издателей раз в секунду. Если все издатели best_effort, подписка переключается на best_effort. После приёма облака адаптация прекращается. Смешанный набор издателей не вызывает такого переключения.</p>

<p>Analysis и assessment публикуются с глубиной очереди 1. Совместимость QoS входного драйвера проверяйте ros2 topic info --verbose. Большие облака могут теряться при best_effort из-за потерь фрагментов DDS; счётчик received_frames отражает только доставленные callback сообщения.</p>

<div class="codebox"><div class="code-label">ВНУТРИ ROS-КОНТЕЙНЕРА</div><pre>ros2 topic list
ros2 topic info /lidar_points --verbose
ros2 interface show metro_perception_interfaces/msg/PathAssessment</pre></div>

(s-9-3)=
## 9.3 FrameAnalysis

```{include} ../../_generated/ru/FrameAnalysis.html
```

(s-9-4)=
## 9.4 PathAssessment

```{include} ../../_generated/ru/PathAssessment.html
```

(s-9-5)=
## 9.5 ObstacleCandidate

```{include} ../../_generated/ru/ObstacleCandidate.html
```

(s-9-6)=
## 9.6 ObstacleTrack

```{include} ../../_generated/ru/ObstacleTrack.html
```

(s-9-7)=
## 9.7 CorridorSegment

```{include} ../../_generated/ru/CorridorSegment.html
```

(s-9-8)=
## 9.8 Формат labelled_points

<div class="table-caption">Таблица 26. Поля облака визуализации</div>

<table><colgroup><col style="width:28%"/><col style="width:22%"/><col style="width:50%"/></colgroup><thead><tr><th>Поле</th><th>Offset, байт</th><th>Тип</th></tr></thead><tbody><tr><td>x / y / z</td><td>0 / 4 / 8</td><td>FLOAT32</td></tr><tr><td>rgb</td><td>12</td><td>FLOAT32 с упакованными RGB-битами</td></tr><tr><td>label</td><td>16</td><td>UINT8: 0 фон, 1 коридор, 2 кандидат-препятствие</td></tr></tbody></table>

<p>Выходное облако имеет height=1, point_step=20, is_bigendian=false и is_dense=true. Оно содержит geometry_points, а не исходный полный организованный кадр. Серый уровень меняется по дальности; зелёный используется для коридора, красный для области кандидата. Поля intensity/ring исходного сенсора в это облако не копируются.</p>

(s-9-9)=
## 9.9 Правила интеграции потребителя

<ol><li>Читать state и stale совместно; для управляющей логики явно определить собственную реакцию на UNKNOWN.</li><li>Использовать distance_m только при distance_valid и finite/positive проверке; не заменять NaN нулём.</li><li>Дедуплицировать наблюдения по ключу источника/сессии/sequence, чтобы heartbeat не добавлял новые hits.</li><li>Различать reported_objects, tracks.confirmed и tracks.coasting. Геометрия разных наблюдений не должна смешиваться.</li><li>Использовать header.frame_id для всех bbox/nearest_point/corridor; корректный топик сам по себе не доказывает верную геометрию.</li><li>Сохранять calibration_trust и evaluated_range_m рядом с состоянием в журналах и downstream API.</li></ol>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>ros/src/perception_<wbr/>node.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/assessment_<wbr/>monitor.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/labelled_<wbr/>cloud.cpp</code>.</div>
