(chapter-11)=
# 11 Офлайн-оценка и артефакты

<p class="lead">Запуск оценщика, JSONL-контракт и воспроизводимость результатов.</p>

(s-11-1)=
## 11.1 Оценить один bag

<div class="codebox"><div class="code-label">ВНУТРИ ROS-КОНТЕЙНЕРА</div><pre>mkdir -p /results/manual-single
ros2 run metro_perception_ros evaluate_bag \
  /data/doubleT_platform /lidar_points \
  /results/manual-single/frames.jsonl</pre></div>

<p>Минимальный вызов использует installed forward_sector_assumed.yaml. Путь OUTPUT.jsonl должен быть новым; существующий файл оценщик не перезаписывает. Каталог назначения следует создать заранее. TOPIC должен существовать и иметь тип sensor_msgs/msg/PointCloud2.</p>

<div class="codebox"><div class="code-label">ПОЗИЦИОННЫЕ АРГУМЕНТЫ: СИНТАКСИС</div><pre>evaluate_bag BAG TOPIC OUTPUT.jsonl \
  [SENSOR_PROFILE.yaml] [MAX_POINTS] \
  [MAX_CLOUD_BYTES] [TF_LOOKAHEAD_S]</pre></div>

<div class="table-caption">Таблица 30. Необязательные аргументы evaluate_bag</div>

<table><colgroup><col style="width:36%"/><col style="width:64%"/></colgroup><thead><tr><th>Аргумент</th><th>По умолчанию / границы</th></tr></thead><tbody><tr><td>SENSOR_PROFILE.yaml</td><td>Пустой/не задан → installed forward_sector_assumed.yaml.</td></tr><tr><td>MAX_POINTS</td><td>2 000 000; [1;10 000 000].</td></tr><tr><td>MAX_CLOUD_BYTES</td><td>268 435 456; [1;1 073 741 824].</td></tr><tr><td>TF_LOOKAHEAD_S</td><td>0,05; конечное [0;1]. Буфер replay читает TF с ограниченным lookahead по bag time.</td></tr></tbody></table>

(s-11-2)=
## 11.2 Измеренный прогон набора

<p>Для привязки к исходникам используйте scripts/evaluate_in_container.sh. Он собирает read-only checkout вне дерева в /tmp, подхватывает полученный install и запускает evaluate_all.py. Фиксируется commit, dirty, Image ID, хеши бинарников, библиотек, профилей, разметки и input-файлов. Новый каталог /results/NAME обязателен.</p>

<div class="codebox"><div class="code-label">ХОСТ: ЗАМЕНИТЬ RUN_NAME НОВЫМ ИМЕНЕМ</div><pre>mkdir -p results
docker build -f docker/Dockerfile --target universal \
  -t metro-lidar:dev docker
metro_image_id="$(docker image inspect metro-lidar:dev --format '{{.Id}}')"
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e IMAGE_ID="$metro_image_id" \
  -v "$PWD:/repo:ro" -v "$PWD/rosbags:/data:ro" \
  -v "$PWD/results:/results" metro-lidar:dev \
  bash /repo/scripts/evaluate_in_container.sh RUN_NAME</pre></div>

<p>При дополнительном --bags передайте ID, зарегистрированные в dataset.yaml. Профили выбираются по sensor_profile, а тема облака — по input_topic. Если нужно исследовать новый неориентированный полный скан, используйте --full-scan-unresolved. --preview, --full-scan-research и --full-scan-unresolved взаимоисключающие.</p>

(s-11-3)=
## 11.3 Структура результатов

<div class="codebox"><div class="code-label">АРТЕФАКТЫ ШТАТНОГО СКРИПТА</div><pre>/results/RUN_NAME/
  manifest.json          # происхождение и хеши
  quality.json           # разметка + метрики качества
  doubleT_platform/
    frames.jsonl         # одна строка на облако bag
    summary.json         # состояния и время
  ...
/results/RUN_NAME-build.log</pre></div>

<p>Полный скрипт дополнительно создаёт человекочитаемый отчёт. Для получения только машинных результатов используйте evaluate_bag и metrics отдельно. При ошибке evaluate_all сохраняет manifest через finally, поэтому наличие manifest не гарантирует успешное завершение всех bag.</p>

(s-11-4)=
## 11.4 JSONL-контракт

<div class="table-caption">Таблица 31. Поля export evaluate_bag</div>

<table><colgroup><col style="width:24%"/><col style="width:76%"/></colgroup><thead><tr><th>Группа</th><th>Поля / смысл</th></tr></thead><tbody><tr><td>Идентичность</td><td>schema_version=1, mode, bag_id, session_id, frame_sequence.</td></tr><tr><td>Время</td><td>measurement_stamp_ns — stamp облака; bag_stamp_ns — stamp записи.</td></tr><tr><td>Решение</td><td>state как строка, reason, distance_valid, distance_m (число либо null).</td></tr><tr><td>Кандидаты</td><td>candidate_count, candidates: id, distance_m, support_points, center, size, channels, edge, closest_offset_m.</td></tr><tr><td>Область</td><td>evaluation_region_valid, evaluated_range_m, ground_inliers, ground_plane.</td></tr><tr><td>Скорость</td><td>ego_motion_valid, ego_speed_mps.</td></tr><tr><td>Треки</td><td>id, confirmed, coasting, hits, age, distance_m, center, size, channels.</td></tr><tr><td>Ось</td><td>route: [c1,c2,valid,max_x,c0,rail_slices]; порядок массива значим.</td></tr><tr><td>Диагностика</td><td>processing_status, calibration_trust, transform_applied, calibration_verified/assumed; счётчики фильтрации.</td></tr><tr><td>Производительность</td><td>processing_ms — обработка конкретного облака offline.</td></tr><tr><td>Дополнительное</td><td>rejected при METRO_DEBUG_COMPONENTS; motion при METRO_DEBUG_MOTION.</td></tr></tbody></table>

<p>channels — битовая маска: MOTION=1, GAUGE=2, вместе=3. Center/size — массивы [x,y,z]. JSONL содержит кандидатов и треки отдельно: наличие кандидата нельзя подменять object hit подтверждённого трека. mode=geometric_rolling при включённой истории, geometric_b0 при background_history_frames=0.</p>

(s-11-5)=
## 11.5 Подсчёт метрик отдельно

<div class="codebox"><div class="code-label">ВНУТРИ ROS-КОНТЕЙНЕРА</div><pre>ros2 run metro_perception_tools metrics \
  /results/manual-single/frames.jsonl \
  --output /results/manual-single/summary.json</pre></div>

<div class="codebox"><div class="code-label">SOURCE/DEV-СРЕДА: ИЗ КОРНЯ; НОВЫЙ ФАЙЛ</div><pre>python3 metro_perception_tools/metro_perception_tools/metrics.py \
  results/manual-single/frames.jsonl \
  --annotations evaluation/annotations/doubleT_platform.yaml \
  --output results/manual-single/quality.json</pre></div>

<p>Без annotations одиночный JSONL даёт статистику состояний и времени, но не TP/FP/FN и ошибку дальности. --output создаётся эксклюзивно. Для каталога полного прогона metrics сверяет manifest с исходниками, dataset и splits; путь к root нужен, если пакет установлен вне source tree.</p>

(s-11-6)=
## 11.6 Как читать воспроизводимость

<div class="table-caption">Таблица 32. Две стороны provenance</div>

<table><colgroup><col style="width:35%"/><col style="width:65%"/></colgroup><thead><tr><th>Поле quality.json</th><th>Интерпретация</th></tr></thead><tbody><tr><td>run_reproducible</td><td>Согласованы recorded-происхождение, input/config/executable/result hashes и полнота выбранных bag.</td></tr><tr><td>run_issues</td><td>Причины нарушения: dirty checkout, отсутствующий Image ID, изменённый профиль, неполный export и другие.</td></tr><tr><td>scoring_current</td><td>Скорер, разметка и splits совпадают с записанными хешами.</td></tr><tr><td>scoring_issues</td><td>Отдельный список несоответствий расчёта качества.</td></tr></tbody></table>

<p>Чистый checkout и флаг true не доказывают обобщение на скрытых данных. Они доказывают возможность идентифицировать и повторить зафиксированный запуск. Для измерений выполняйте один bag за раз и фиксируйте CPU, нагрузку, режим сборки и параметры. Debug-массивы в JSONL увеличивают объём и не должны незаметно попадать в сравнение производительности.</p>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>ros/src/evaluate_<wbr/>bag.cpp</code>; <code>scripts/evaluate_<wbr/>in_<wbr/>container.sh</code>; <code>scripts/evaluate_<wbr/>all.py</code>; <code>metro_<wbr/>perception_<wbr/>tools/metro_<wbr/>perception_<wbr/>tools/metrics.py</code>.</div>
