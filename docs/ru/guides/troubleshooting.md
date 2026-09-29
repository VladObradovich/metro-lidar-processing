(chapter-15)=
# 15 Диагностика проблем

<p class="lead">Проверки от входного сообщения к геометрии и временной оценке.</p>

(s-15-1)=
## 15.1 Сначала проверить цепочку

<ol><li>Container/process жив: docker ps, затем список ROS-узлов в той же ROS-среде.</li><li>Bag проигрывается; правильный тип и топик видны в bag info и topic info.</li><li>Получается /metro/analysis; received_frames растёт.</li><li>processing_status=OK или есть конкретная ошибка входа/TF/геометрии.</li><li>В assessment проверить stale, result_age, source/session/sequence и rejection diagnostics.</li><li>После этого анализировать corridor, evaluated range, candidates и tracks.</li></ol>

<div class="codebox"><div class="code-label">ВНУТРИ КОНТЕЙНЕРА: ПО ОДНОЙ КОМАНДЕ</div><pre>ros2 node list
ros2 topic info /lidar_points --verbose
ros2 topic hz /metro/analysis
ros2 topic echo /metro/analysis --once
ros2 topic echo /metro/assessment --once</pre></div>

(s-15-2)=
## 15.2 Запуск и доставка данных

<div class="table-caption">Таблица 43. Инфраструктурная диагностика</div>

<table><colgroup><col style="width:29%"/><col style="width:30%"/><col style="width:41%"/></colgroup><thead><tr><th>Симптом</th><th>Что проверить</th><th>Действие</th></tr></thead><tbody><tr><td>ros2: command not found через docker exec</td><td>Загружено ли окружение installed runtime.</td><td>Использовать docker exec … metro-entrypoint ros2 …</td></tr><tr><td>WAITING_FOR_INPUT</td><td>Topic, проигрыватель, container, namespace, QoS.</td><td>bag info; input_topic; topic info --verbose; все ROS-команды в одном контейнере при localhost-only.</td></tr><tr><td>Input виден, analysis нет</td><td>Совместимость reliable / best_effort; очень крупное облако.</td><td>Сверить QoS издателя; input_reliability; перезапустить узел с корректным runtime YAML.</td></tr><tr><td>Permission denied в results</td><td>UID/GID владельца каталога и контейнера.</td><td>--user либо METRO_UID/METRO_GID; создать writable каталог до запуска.</td></tr><tr><td>Docker пытается скачивать на офлайн-стенде</td><td>Загружены ли нужные теги images.</td><td>docker image ls; docker load; Compose --no-build; не передавать --build desktop.sh.</td></tr></tbody></table>

(s-15-3)=
## 15.3 UNKNOWN и геометрия

<div class="table-caption">Таблица 44. Причины недостаточности наблюдения</div>

<table><colgroup><col style="width:34%"/><col style="width:29%"/><col style="width:37%"/></colgroup><thead><tr><th>reason / симптом</th><th>Смысл</th><th>Проверка</th></tr></thead><tbody><tr><td>TF_UNAVAILABLE</td><td>Нет пригодного transform на stamp.</td><td>source/target frame, bind/exact, static profile, внешний TF и временная область.</td></tr><tr><td>CALIBRATION_UNVERIFIED</td><td>Unverified запрещён; ориентация не доказана.</td><td>Это штатно для unresolved; установить калибровку либо явно принятую гипотезу.</td></tr><tr><td>EMPTY_GEOMETRY_ROI</td><td>После фильтрации нет опоры.</td><td>Оси, transform, units, ROI и счётчики invalid/blind/outside.</td></tr><tr><td>GROUND_UNSUPPORTED</td><td>Пол не оценён с достаточной опорой.</td><td>Окклюзия, station trough, slopes, ориентация и density; не объявлять пустой путь.</td></tr><tr><td>EVALUATED_RANGE_TOO_SHORT</td><td>ASSUMED область &lt;50 м.</td><td>Фактическая видимость поверхности. Увеличение ROI не даёт наблюдений пола.</td></tr><tr><td>CANDIDATE_UNCONFIRMED</td><td>Недостаточно temporal evidence / EDGE.</td><td>Каналы, hits, age, offset, route support и подтверждение.</td></tr><tr><td>BACKGROUND_CANNOT_CONFIRM_CLEAR</td><td>Разностный фон не доказал свободную область.</td><td>Валидная ось и GAUGE, история; состояние UNKNOWN корректно.</td></tr></tbody></table>

(s-15-4)=
## 15.4 Скорость и устаревание

<div class="table-caption">Таблица 45. Диагностика времени</div>

<table><colgroup><col style="width:36%"/><col style="width:64%"/></colgroup><thead><tr><th>Симптом</th><th>Вероятное направление проверки</th></tr></thead><tbody><tr><td>INPUT_PAUSED_OR_STOPPED после конца bag</td><td>Штатный watchdog; объекты очищаются. Для нового измерения возобновить поток.</td></tr><tr><td>Много overwritten_frames</td><td>Worker медленнее входа; проверить Release, CPU нагрузку и replay rate. Latest-only не накопит очередь.</td></tr><tr><td>Растёт queue_age_ms</td><td>Ожидание worker: тяжёлая обработка или contention CPU.</td></tr><tr><td>Большой tf_wait_ms</td><td>Внешний TF приходит поздно/не приходит; проверить stamps и ограничение ожидания.</td></tr><tr><td>Большой processing_age_ms при небольшом queue_age</td><td>Вычисления/TF; разделить профилирование и GUI.</td></tr><tr><td>Большой rejected_frames</td><td>Возраст &gt;max_processing_age, смена сессии либо ошибка анализа; смотреть status/reason и счётчики совместно.</td></tr><tr><td>rejected_analyses растёт</td><td>Порядок источников/сессий/sequence или невалидный возраст; читать last_rejection_reason.</td></tr></tbody></table>

(s-15-5)=
## 15.5 Проблемы графики

<div class="table-caption">Таблица 46. Диагностика RViz</div>

<table><colgroup><col style="width:36%"/><col style="width:64%"/></colgroup><thead><tr><th>Симптом</th><th>Действие</th></tr></thead><tbody><tr><td>RViz не открылся</td><td>Использовать runtime-desktop и desktop.sh; проверить DISPLAY и X11-прокси.</td></tr><tr><td>Assessment panel отсутствует</td><td>Проверить installed libassessment_panel.so; rviz_common должен существовать на этапе сборки desktop-образа.</td></tr><tr><td>Облако во неверном frame</td><td>Fixed frame должен соответствовать target_frame. labelled_points уже преобразован; raw cloud требует TF.</td></tr><tr><td>Нет depth/labelled cloud</td><td>Штатный perception.launch.py включает их только при rviz=true; visualizer=true даёт лишь маркеры.</td></tr><tr><td>Есть красные точки, state UNKNOWN</td><td>Проверить tentative/EDGE. Раскраска кандидата не равна подтверждению.</td></tr><tr><td>GUI замедляет обработку</td><td>Проверить GPU/Mesa, входную частоту и CPU; повторить headless замер для сравнения.</td></tr></tbody></table>

(s-15-6)=
## 15.6 Ошибки офлайн-оценки

<ul><li>Output already exists: выбрать новый JSONL. evaluate_bag не перезаписывает файл.</li><li>Requested PointCloud2 topic not found: проверить bag info и input_topic из dataset.</li><li>FileExistsError в evaluate_all/metrics: новый каталог/файл обязателен.</li><li>run_reproducible=false: прочитать run_issues; наличие quality.json не доказывает корректный provenance.</li><li>scoring_current=false: прочитать scoring_issues; разметка, splits либо scorer изменились после прогона.</li><li>Recall высокий, нужный объект не найден: проверить object_confirmed, wrong_object и confirmed tracks, а не только state-level.</li><li>N/A в precision/ошибке: нет положительной или matched-выборки; это не 100% качества.</li></ul>

(s-15-7)=
## 15.7 Что сохранить для разбора

<p>Минимальный диагностический пакет: команда запуска, commit и profile, input topic и frame, фрагмент analysis/assessment с ключом наблюдения и возрастами, counters, описание ожидаемого поведения и результаты затронутой проверки. Для bag-прогона добавьте manifest, quality/summary и соответствующий reference interval development-данных. Из слепой части сохраняется только разрешённый суммарный итог.</p>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>ros/src/perception_<wbr/>node.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/assessment_<wbr/>monitor.cpp</code>; <code>metro_<wbr/>perception_<wbr/>core/src/pipeline.cpp</code>; <code>scripts/evaluate_<wbr/>all.py</code>; <code>scripts/desktop.sh</code>.</div>
