(chapter-12)=
# 12 Данные, разметка и метрики

<p class="lead">Что именно измеряется и как избежать ложного вывода об обнаруженном объекте.</p>

(s-12-1)=
## 12.1 Реестр набора

<p>evaluation/dataset.yaml — источник ID, относительных путей, входных топиков, типа sensor_profile и файлов разметки. Реестр не содержит сами bag. В текущем наборе семь реальных записей и одна запись с вставленными синтетическими объектами.</p>

<div class="table-caption">Таблица 33. Зарегистрированные записи</div>

<table><colgroup><col style="width:48%"/><col style="width:20%"/><col style="width:32%"/></colgroup><thead><tr><th>ID bag</th><th>Облаков по реестру</th><th>Профиль / split</th></tr></thead><tbody><tr><td><code>doubleT_<wbr/>obstacle</code></td><td>201</td><td><code>full_<wbr/>scan</code><br/>development</td></tr><tr><td><code>doubleT_<wbr/>platform</code></td><td>345</td><td><code>forward_<wbr/>sector</code><br/>development</td></tr><tr><td><code>roundT_<wbr/>doubleT</code></td><td>252</td><td><code>forward_<wbr/>sector</code><br/>validation</td></tr><tr><td><code>roundT_<wbr/>pressureGate_<wbr/>roundT</code></td><td>268</td><td><code>forward_<wbr/>sector</code><br/>development</td></tr><tr><td><code>roundT_<wbr/>squareT_<wbr/>pressureGate_<wbr/>squareT</code></td><td>545</td><td><code>forward_<wbr/>sector</code><br/>validation</td></tr><tr><td><code>squareT_<wbr/>platform_<wbr/>squareT_<wbr/>switch</code></td><td>877</td><td><code>forward_<wbr/>sector</code><br/>validation</td></tr><tr><td><code>new_<wbr/>data</code></td><td>11271</td><td><code>forward_<wbr/>sector</code><br/>development</td></tr><tr><td><code>cloud_<wbr/>with_<wbr/>fake_<wbr/>obj</code></td><td>1510</td><td><code>forward_<wbr/>sector</code><br/>regression</td></tr></tbody></table>

<p>У doubleT_obstacle топик /sensing/lidar/hesai128/pointcloud; у остальных — /lidar_points. Все записи реестра используют sqlite3. Количество здесь — declared_message_count из metadata-реестра, а не результат нового экспорта. Полноту прогонов проверяют по реальному числу JSONL-строк.</p>

(s-12-2)=
## 12.2 Split и слепая часть

<p>Development используется для разработки и выбора параметров. Validation в текущей истории уже просматривалась и сравнивалась, поэтому это post-hoc проверка. cloud_with_fake_obj отнесён к regression. Независимого положительного holdout нет: скрытые контрольные bag остаются главным источником проверки обобщения.</p>

<p>new_data разделён по bag time: первые 595 с — development; дальнейшая часть — blind_holdout. Метаданные splits.yaml задают границу from_bag_stamp_ns. Покадровые сцены и отдельные события слепой части не используются для диагностики и подбора порогов; допустимо смотреть только суммарный итог через holdout_alarms.py.</p>

<aside class="important"><strong>Предел независимости</strong><p>Development, validation и regression уже участвовали в разработке или проверках. Отдельного положительного holdout нет. Суммарные результаты слепого интервала следует рассматривать отдельно от разработочных метрик; повторное использование этого итога для выбора параметров нарушает независимость проверки.</p></aside>

(s-12-3)=
## 12.3 Схема разметки

<p>Разметка YAML имеет schema_version, bag_id, reviewed=true, time_basis=bag_stamp_ns и reviewed_intervals. Метки: positive, negative, uncertain. Вне размеченных интервалов кадр unlabeled. Если интервалы пересекаются на общей границе, приоритет positive → uncertain → negative.</p>

<div class="codebox"><div class="code-label">YAML: МИНИМАЛЬНЫЙ ПРИМЕР, ВРЕМЕНА УСЛОВНЫЕ</div><pre>schema_version: 1
bag_id: example
reviewed: true
time_basis: bag_stamp_ns
reviewed_intervals:
  - start_ns: 1000000000
    end_ns: 2000000000
    label: negative
events: []</pre></div>

<p>Положительные events могут содержать reference_frames с дистанцией, uncertainty_m и ROI в целевой предполагаемой системе. Историческое имя person_roi_assumed_m используется и для коробов: это generic ROI объекта в текущем скорере. Отрицательные события с ROI позволяют отдельно проверить предметы вне/выше габарита.</p>

(s-12-4)=
## 12.4 Frame-level и event-level

<div class="table-caption">Таблица 34. Семантика покадровых и событийных показателей</div>

<table><colgroup><col style="width:27%"/><col style="width:73%"/></colgroup><thead><tr><th>Показатель</th><th>Определение / ограничение</th></tr></thead><tbody><tr><td>TP кадров</td><td>OBSTACLE на positive-интервале. Это совпадение состояния с интервалом, а не доказательство нужного объекта.</td></tr><tr><td>FN кадров</td><td>UNKNOWN и NO_OBSTACLE_DETECTED на positive-интервале. UNKNOWN не скрывает пропуск.</td></tr><tr><td>FP кадров</td><td>OBSTACLE на negative-интервале.</td></tr><tr><td>Frame recall</td><td>TP / число positive-кадров; верхняя оценка object recall при посторонней тревоге.</td></tr><tr><td>Frame precision</td><td>TP / (TP+FP); N/A без positive-кадров.</td></tr><tr><td>Event detection</td><td>Есть хотя бы одна тревога в событии; возможен wrong-object.</td></tr><tr><td>Ложное событие</td><td>Последовательность тревог, объединённых по event-gap-s, по умолчанию 0,5 с.</td></tr><tr><td>Uncertain / unlabeled</td><td>Не включаются в TP/FP/FN; uncertain описывается отдельно.</td></tr></tbody></table>

(s-12-5)=
## 12.5 Object-level

<p>Object hit требует OBSTACLE и совпадение центра decisive object с reference ROI, расширенной на roi-margin-m=0,5 м. В современных exports decisive_objects использует confirmed tracks; для старых форматов предусмотрен fallback к кандидатам. Ближайшая строка должна быть в stamp-tolerance-s=0,05 с от reference bag stamp.</p>

<div class="table-caption">Таблица 35. Результаты сопоставления объектов</div>

<table><colgroup><col style="width:28%"/><col style="width:72%"/></colgroup><thead><tr><th>Исход проверки</th><th>Смысл</th></tr></thead><tbody><tr><td>hit</td><td>Подтверждённый объект совпал с размеченной ROI при OBSTACLE.</td></tr><tr><td>miss</td><td>В подходящем кадре state не OBSTACLE.</td></tr><tr><td>wrong_object</td><td>Тревога есть, но decisive object не совпал с ROI нужного объекта.</td></tr><tr><td>no_frame</td><td>Нет строки в заданном временном допуске.</td></tr></tbody></table>

<p>object_confirmed у события означает наличие хотя бы одного такого object hit. first_matched_distance_m и first_matched_delay_s следует использовать для вывода о первом обнаружении нужного объекта. first_alarm_* — первое любое совпадение состояния; эти величины отвечают на разные вопросы.</p>

(s-12-6)=
## 12.6 Ошибка дальности и покрытие

<p>Ошибка дальности рассчитывается только по object hits с измеренной reference distance. Mean absolute error — среднее |reported−reference|, max_abs_error — максимум среди совпадений. Маленькая ошибка у найденных объектов не означает высокую полноту: пропуски в эту выборку не входят.</p>

<p>coverage отражает долю определённых решений, valid region и причины UNKNOWN. unknown_fraction в summary относится ко всем экспортированным кадрам. Диапазоны object recall строятся по reference-distance: 0–20, 20–40, 40–60, 60–80, 80–100, 100–120, 120–150, 150–200 м и далее.</p>

<aside class="important"><strong>Два разных «диапазона»</strong><p>working_range в metrics — дальний предел detection ROI профиля, записанный в manifest. evaluated_range_m — фактическая пригодная область конкретного кадра по геометрии. В штатном профиле working_range=120 м, а наблюдаемая область обычно заканчивается раньше. Их нельзя подменять друг другом.</p></aside>

(s-12-7)=
## 12.7 Синтетические сценарии

<p>scripts/inject_obstacle.py создаёт axis-aligned объекты в предполагаемой системе координат. Сценарии: стоящий человек 0,5×0,5×1,7 м, короб 1×1×0,5 м на рельсах, человек поперёк пути со скоростью 1 м/с. Лучи, пересекающие объект до исходной поверхности, переносятся на его границу; это сохраняет структуру облака и окклюзию. Траектория и пол берутся из reference JSONL того же bag.</p>

<p>Для синтетики используется видимость не меньше 10 возвратов. Разметка вставленных объектов строится annotate_inserted_objects.py с опорой на evaluation/scene_context.yaml; кадры со слишком слабой видимостью остаются uncertain. Синтетика помогает проверять регрессии, но не заменяет реальные независимые положительные записи.</p>

(s-12-8)=
## 12.8 Минимальный набор выводов по запуску

<ul><li>Полнота экспорта, commit, image, профиль и hashes; отдельно run_reproducible и scoring_current.</li><li>Object hits / misses / wrong_object и первое сопоставленное обнаружение по каждому событию.</li><li>Ложные кадры, ложные события и их длительность на negative-данных.</li><li>UNKNOWN и покрытие решения; отдельно причины недостаточной геометрии и неподтверждённых кандидатов.</li><li>Ошибка дальности только на matched-выборке; явно указать её объём.</li><li>processing p50/p95/p99 и онлайн result_age; CPU и параллельная нагрузка.</li></ul>

<div class="source">Основание: <code>evaluation/dataset.yaml</code>; <code>evaluation/splits.yaml</code>; <code>metro_<wbr/>perception_<wbr/>tools/metro_<wbr/>perception_<wbr/>tools/metrics.py</code>; <code>scripts/inject_<wbr/>obstacle.py</code>; <code>scripts/annotate_<wbr/>inserted_<wbr/>objects.py</code>.</div>
