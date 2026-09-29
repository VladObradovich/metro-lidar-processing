(chapter-4)=
# 4 Эксплуатация и RViz

<p class="lead">Рабочие сценарии, визуальная проверка и трактовка состояния пути.</p>

(s-4-1)=
## 4.1 Графическая демонстрация

<div class="codebox"><div class="code-label">ХОСТ: ТЕРМИНАЛ 1</div><pre>bash scripts/desktop.sh up
bash scripts/desktop.sh exec ros2 launch \
  metro_perception_bringup perception.launch.py rviz:=true</pre></div>

<div class="codebox"><div class="code-label">ХОСТ: ТЕРМИНАЛ 2</div><pre>bash scripts/desktop.sh exec ros2 bag play /data/cloud_with_fake_obj</pre></div>

<p>desktop.sh настраивает X11-прокси, права пользователя и графические устройства. При наличии NVIDIA Container Toolkit он может подключить GPU; --no-nvidia выбирает вариант без NVIDIA. GPU используется для RViz, вычислительное ядро детектора работает на CPU. Остановка сервиса: bash scripts/desktop.sh down.</p>

<div class="table-caption">Таблица 5. Штатная конфигурация RViz</div>

<table><colgroup><col style="width:28%"/><col style="width:72%"/></colgroup><thead><tr><th>Область интерфейса</th><th>Что показывает</th></tr></thead><tbody><tr><td>Depth, слева сверху</td><td>Развёртка облака по вертикальным каналам и азимуту ±50°; в launch установлены 480×128 пикселей и диапазон глубины 1–150 м.</td></tr><tr><td>Assessment, слева снизу</td><td>Текст PathAssessment: состояние, причина, дистанция, проверенная дальность, калибровка, возраст и треки.</td></tr><tr><td>Облако, справа</td><td>Анализируемые точки во фрейме lidar_assumed с раскраской label/rgb.</td></tr><tr><td>Коридор</td><td>Четыре продольные линии: низ/верх слева и справа. Зелёный при пригодной геометрии/покрытии, иначе жёлтый.</td></tr><tr><td>Detection</td><td>Полный набор маркеров: боксы, треки, ближайшая точка и надпись. По умолчанию выключен; включается в Displays.</td></tr></tbody></table>

<aside class="note"><strong>Цвет точки и решение</strong><p>Красные точки обозначают области кандидатов, включая неподтверждённые. Наличие красного фрагмента в облаке не означает OBSTACLE: решение зависит от подтверждения трека и пригодной положительной дистанции.</p></aside>

(s-4-2)=
## 4.2 Прочитать assessment без графики

<div class="codebox"><div class="code-label">ХОСТ: ВЫПОЛНЯТЬ В ОТДЕЛЬНЫХ ТЕРМИНАЛАХ</div><pre>docker exec metro-demo metro-entrypoint ros2 topic echo \
  /metro/assessment --field reason
docker exec metro-demo metro-entrypoint ros2 topic echo \
  /metro/assessment --field distance_m</pre></div>

<p>Для автоматической интеграции читайте целое сообщение, а не отдельное числовое поле. Оно связывает state, distance_valid, stale и идентичность наблюдения. Служебный heartbeat публикуется даже при отсутствии нового облака; повторный stamp не является новой фиксацией объекта.</p>

<div class="table-caption">Таблица 6. Семантика состояний</div>

<table><colgroup><col style="width:25%"/><col style="width:55%"/><col style="width:20%"/></colgroup><thead><tr><th>state</th><th>Как понимать</th><th>Дистанция</th></tr></thead><tbody><tr><td>0: UNKNOWN</td><td>Данных или подтверждения недостаточно. reason указывает причину.</td><td>Недействительна; NaN.</td></tr><tr><td>1: OBSTACLE</td><td>Есть подтверждённый трек с конечной дистанцией &gt; 0.</td><td>distance_valid=true; возможен coasting.</td></tr><tr><td>2: NO_OBSTACLE_DETECTED</td><td>Нет кандидатов в пригодной оценённой области; при ASSUMED нужны ≥50 м.</td><td>Дистанции до препятствия нет; NaN.</td></tr></tbody></table>

(s-4-3)=
## 4.3 Основные причины

<div class="table-caption">Таблица 7. Причины, часто встречающиеся в эксплуатации</div>

<table><colgroup><col style="width:45%"/><col style="width:55%"/></colgroup><thead><tr><th>reason</th><th>Интерпретация / действие</th></tr></thead><tbody><tr><td><code>WAITING_<wbr/>FOR_<wbr/>INPUT</code></td><td>Узел запущен, свежий analysis ещё не принят. Проверьте проигрыватель и топик.</td></tr><tr><td><code>BASELINE_<wbr/>WARMUP</code></td><td>Накопление истории фона в начале обработки или после сброса.</td></tr><tr><td><code>CANDIDATE_<wbr/>UNCONFIRMED</code></td><td>Есть свидетельство объекта, но подтверждения трека недостаточно, либо кандидат у края.</td></tr><tr><td><code>OBSTACLE_<wbr/>WITH_<wbr/>ASSUMED_<wbr/>CALIBRATION</code></td><td>Подтверждённое препятствие при явно предполагаемой калибровке.</td></tr><tr><td><code>OBSTACLE_<wbr/>COASTING_<wbr/>WITH_<wbr/>ASSUMED_<wbr/>CALIBRATION</code></td><td>Подтверждённый трек удержан без нового измерения; положение спрогнозировано.</td></tr><tr><td><code>NO_<wbr/>CANDIDATE_<wbr/>ASSUMED_<wbr/>CALIBRATION</code></td><td>Пустая пригодная область при ASSUMED и разрешённом assumed_clear.</td></tr><tr><td><code>EVALUATED_<wbr/>RANGE_<wbr/>TOO_<wbr/>SHORT</code></td><td>Область короче порога assumed_clear_min_range_m; сохраняется UNKNOWN.</td></tr><tr><td><code>GROUND_<wbr/>UNSUPPORTED</code></td><td>Нет достаточной опоры поверхности пути.</td></tr><tr><td><code>INPUT_<wbr/>PAUSED_<wbr/>OR_<wbr/>STOPPED</code></td><td>Watchdog: вход остановлен либо результат устарел; stale=true.</td></tr></tbody></table>

(s-4-4)=
## 4.4 Симуляционное время и повтор записи

<div class="codebox"><div class="code-label">ВНУТРИ КОНТЕЙНЕРА: ДВА ТЕРМИНАЛА</div><pre>ros2 launch metro_perception_bringup perception.launch.py \
  use_sim_time:=true
# В другом терминале той же ROS-среды:
ros2 bag play /data/doubleT_platform --clock</pre></div>

<p>use_sim_time=true должен сопровождаться источником /clock. Watchdog и ограничение локального возраста используют steady clock, поэтому пауза /clock не останавливает контроль свежести. При возврате header.stamp назад perception создаёт новую сессию и сбрасывает историю. Для повторного эксперимента предпочтительнее начать новый процесс и новый каталог результатов.</p>

(s-4-5)=
## 4.5 Наблюдение нагрузки

<p>Смотрите processing_age_ms, queue_age_ms и tf_wait_ms в analysis, result_age_ms в assessment, а также overwritten_frames и rejected_frames. Перезапись ожидающего кадра — предусмотренное поведение latest-only: оно ограничивает задержку ценой пропуска кадров. Устойчивый рост пропусков при проигрывании 1× требует проверки Release-сборки, нагрузки CPU, QoS и частоты входа.</p>

<div class="source">Основание: <code>scripts/desktop.sh</code>; <code>metro_<wbr/>perception_<wbr/>bringup/launch/perception.launch.py</code>; <code>metro_<wbr/>perception_<wbr/>bringup/rviz/detector.rviz</code>; <code>metro_<wbr/>perception_<wbr/>core/include/metro_<wbr/>perception_<wbr/>core/labels.hpp</code>.</div>
