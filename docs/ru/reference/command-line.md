(chapter-Б)=
# Б Инструменты командной строки

<p class="lead">Утилиты эксплуатации, исследования и проверки с контекстом запуска.</p>

(s-B-1)=
## Б.1 Эксплуатация и пакетные команды

<div class="table-caption">Таблица 57. Утилиты запуска</div>

<table><colgroup><col style="width:52%"/><col style="width:48%"/></colgroup><thead><tr><th>Команда / файл</th><th>Назначение</th></tr></thead><tbody><tr><td>ros2 run metro_perception_tools depth_image</td><td>Проекция PointCloud2 в видео; штатный запуск — через perception.launch.py rviz=true.</td></tr><tr><td>ros2 run metro_perception_tools inspect_bag BAG</td><td>Metadata и bounded sample структуры PointCloud2. --sample-count N; --output FILE для нового JSON.</td></tr><tr><td>ros2 run metro_perception_tools metrics RESULTS --output JSON</td><td>Summary либо качество по annotations; глава 11.</td></tr><tr><td>ros2 run metro_perception_tools report QUALITY.json</td><td>Преобразование quality.json в человекочитаемое представление; путь вывода задаётся обязательным --output.</td></tr><tr><td>bash scripts/desktop.sh up / exec / down</td><td>Контейнер RViz, графические устройства и X11-прокси. --source подключает checkout; --nvidia/--no-nvidia управляют графическим вариантом.</td></tr><tr><td>bash scripts/export_images.sh [OUTPUT_DIR]</td><td>Экспорт заранее собранных local/desktop images и checksum.</td></tr></tbody></table>

(s-B-2)=
## Б.2 Аудит данных

<div class="codebox"><div class="code-label">ПЕРВАЯ КОМАНДА: SOURCE-СРЕДА; ВТОРАЯ: ROS-КОНТЕЙНЕР</div><pre>python3 scripts/audit_bags.py \
  --dataset-root rosbags --output results/audit-new.json
ros2 run metro_perception_tools inspect_bag /data/doubleT_platform \
  --sample-count 2 --output /results/layout-new.json</pre></div>

<p>audit_bags.py сверяет реестр и файлы, метаданные/структуру. По умолчанию new_data исключён из аудита скрипта; не используйте выбор отдельных кадров blind holdout для разработки. inspect_bag смотрит только первые bounded samples и не характеризует полный bag.</p>

(s-B-3)=
## Б.3 Ложные события и слепой итог

<div class="codebox"><div class="code-label">SOURCE/DEV-СРЕДА: НОВЫЕ OUTPUT</div><pre>python3 scripts/fp_events.py results/RUN_NAME \
  --output results/fp-events-new.json
python3 scripts/holdout_alarms.py results/RUN_NAME \
  --output results/holdout-totals-new.json</pre></div>

<p>fp_events.py анализирует допустимые события по отрицательной разметке и split-правилам. holdout_alarms.py выдаёт суммарное число тревог на blind interval без инспекции отдельных сцен. Оба инструмента читают уже существующий export; исходные bag для повторного просмотра не требуются.</p>

(s-B-4)=
## Б.4 Синтетика

<div class="codebox"><div class="code-label">DEV-КОНТЕЙНЕР: ИЗ КОРНЯ CHECKOUT</div><pre>python3 scripts/inject_obstacle.py inject \
  /data/doubleT_platform /lidar_points \
  /results/REFERENCE_RUN/doubleT_platform/frames.jsonl \
  /results/synthetic-new --scenario static
python3 scripts/inject_obstacle.py dataset /results/synthetic-new
python3 scripts/evaluate_all.py --dataset-root /results/synthetic-new \
  --dataset /results/synthetic-new/dataset.yaml \
  --output-dir /results/synthetic-evaluation-new
python3 scripts/inject_obstacle.py report \
  /results/synthetic-evaluation-new /results/synthetic-new \
  --output /results/synthetic-report-new.json</pre></div>

<p>Замените REFERENCE_RUN на прогон того же исходного bag; допустимые --scenario: static, box, crossing. Каждый output root должен быть отдельным. Интерпретация recall синтетики зависит от её разметки и физической видимости; сценарий пересечения может включать положения за границей габарита.</p>

(s-B-5)=
## Б.5 Сравнение профилей

<div class="codebox"><div class="code-label">DEV-КОНТЕЙНЕР: ПРИ ПОДГОТОВЛЕННОЙ СИНТЕТИКЕ</div><pre>python3 scripts/sweep_profile.py experiment-name \
  detector.route_smoothing=0.5 --jobs 1</pre></div>

<p>sweep_profile.py принимает KEY=VALUE либо section.KEY=VALUE, --dataset-root, --synthetic-dev, --synthetic, --out-root и --install. Это исследовательский инструмент; default jobs=8 параллелит обработку и не подходит для сравнения latency. Для времени используйте последовательный чистый прогон. --val добавляет validation synthetic set, не независимый holdout.</p>

(s-B-6)=
## Б.6 Диагностический export

<div class="codebox"><div class="code-label">ВНУТРИ ROS-КОНТЕЙНЕРА: НОВЫЙ JSONL</div><pre>METRO_DEBUG_COMPONENTS=1 METRO_DEBUG_MOTION=1 \
  ros2 run metro_perception_ros evaluate_bag \
  /data/doubleT_platform /lidar_points /results/debug-new.jsonl</pre></div>

<p>rejected описывает отвергнутые компоненты с причиной, размером и поддержкой. motion записывает ошибки пробных сдвигов и состояние speed tracker. Расширенная диагностика влияет на объём и скорость I/O; сравнение performance с обычным export должно учитывать это различие.</p>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>tools/setup.py</code>; <code>scripts/audit_<wbr/>bags.py</code>; <code>scripts/fp_<wbr/>events.py</code>; <code>scripts/holdout_<wbr/>alarms.py</code>; <code>scripts/inject_<wbr/>obstacle.py</code>; <code>scripts/sweep_<wbr/>profile.py</code>.</div>
