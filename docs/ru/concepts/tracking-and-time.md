(chapter-8)=
# 8 Треки, время и свежесть

<p class="lead">Временное подтверждение, защита от запоздалых сообщений и выбор состояния.</p>

(s-8-1)=
## 8.1 Кандидат и трек

<p>candidate_id локален одному кадру. reported_objects содержит все кандидаты принятого успешно проанализированного кадра, включая неподтверждённые и кандидатов с непригодной дистанцией. track_id относится к последовательности наблюдений, объединённых временным монитором; tracks содержит и tentative, и confirmed. ID трека не следует считать глобальным идентификатором объекта вне источника и сессии.</p>

<p>Сопоставление выполняется жадно, один-к-одному, по расстоянию центров в плоскости x/y. Допустимый gate растёт с дальностью и интервалом между кадрами. При известной ego-speed предыдущая позиция сдвигается на пройденный поездом путь. Неизвестное собственное движение учитывается дополнительным допуском.</p>

(s-8-2)=
## 8.2 Правила подтверждения

<div class="table-caption">Таблица 17. Штатная temporal-конфигурация</div>

<table><colgroup><col style="width:44%"/><col style="width:56%"/></colgroup><thead><tr><th>Условие</th><th>Штатное правило</th></tr></thead><tbody><tr><td>Обычный объект</td><td>2 hits в окне 3 обработанных кадров.</td></tr><tr><td>Только GAUGE</td><td>2 hits в 3 кадрах; отдельные поля конфигурации.</td></tr><tr><td>Дальше 70 м / за evaluated range</td><td>Дополнительно 4 hits в окне 5 кадров.</td></tr><tr><td>Движение ≥3 м/с при валидной скорости</td><td>Для подтверждения нужен канал GAUGE.</td></tr><tr><td>Стоянка: |v|&lt;0,5 м/с</td><td>Плотное MOTION-свидетельство ≥50 возвратов может подтвердиться на первом hit.</td></tr><tr><td>EDGE</td><td>Измерение не добавляет hit; возможен отзыв прежнего подтверждения при устойчивом уходе наружу.</td></tr><tr><td>Один пропуск после подтверждения</td><td>release_misses=2: удержание с coasting через один пропуск.</td></tr></tbody></table>

<figure><img alt="Рисунок 3. Иллюстрация подтверждения и удержания при пропуске; предполагаются валидная геометрия и дистанция." src="/_static/diagrams/figure-03.svg"/><figcaption>Рисунок 3. Иллюстрация подтверждения и удержания при пропуске; предполагаются валидная геометрия и дистанция.</figcaption></figure>

<p>Coasting означает, что объект не был измерен в текущем кадре: позиция и дальность прогнозируются. При второй последовательной потере измерения подтверждённый трек удаляется. EDGE-retraction использует три последовательных измерения бокового смещения: устойчивый уход к краю снимает подтверждение и историю hits. Мгновенное плотное MOTION-подтверждение на стоянке — отдельное исключение из обычных оконных правил.</p>

(s-8-3)=
## 8.3 Порядок выбора состояния

<ol><li>Неуспешный анализ даёт UNKNOWN с причиной кадра; прежние треки не становятся новым положительным доказательством.</li><li>Подтверждённый трек с конечной положительной дистанцией даёт OBSTACLE; выбирается ближайший.</li><li>Пригодный кандидат без подтверждения или сохранившийся tentative при отсутствии новых кандидатов дают UNKNOWN.</li><li>Кандидаты только с непригодной дистанцией дают UNKNOWN / CANDIDATE_DISTANCE_INVALID.</li><li>Без кандидатов проверяются trust, пригодность и положительная evaluated range. UNKNOWN trust запрещает «свободно».</li><li>При ASSUMED дополнительно нужны assumed_clear=true и evaluated_range_m≥assumed_clear_min_range_m.</li><li>Лишь после этих проверок выдаётся NO_OBSTACLE_DETECTED; иначе UNKNOWN с конкретной причиной.</li></ol>

<aside class="important"><strong>Контракт дистанции</strong><p>OBSTACLE требует distance_valid=true, finite(distance_m) и distance_m&gt;0. Для любой отсутствующей итоговой дистанции используется NaN и distance_valid=false. Ноль не используется как код «нет препятствия». В JSONL аналог отсутствующей дистанции — null.</p></aside>

(s-8-4)=
## 8.4 Три шкалы времени

<div class="table-caption">Таблица 18. Раздельные временные основания</div>

<table><colgroup><col style="width:30%"/><col style="width:37%"/><col style="width:33%"/></colgroup><thead><tr><th>Время</th><th>Для чего</th><th>С чем не путать</th></tr></thead><tbody><tr><td>bag_stamp_ns</td><td>Привязка разметки и событий; порядок по времени записи bag.</td><td>Не подменяет timestamp измерения для TF.</td></tr><tr><td>header.stamp / measurement_stamp_ns</td><td>TF на момент облака; dt для temporal; обнаружение скачка назад.</td><td>Не используется как локальные часы watchdog.</td></tr><tr><td>steady clock</td><td>processing_age, очередь, ожидание TF, watchdog и result_age.</td><td>Не хранит абсолютную дату сенсорного измерения.</td></tr></tbody></table>

<p>processing_age_ms — время от локального приёма облака до подготовки публикации analysis. queue_age_ms выделяет ожидание worker; tf_wait_ms — локальное время resolve_context. result_age_ms добавляет время после получения analysis монитором. Эти поля не измеряют задержку от физической экспозиции сенсора до компьютера.</p>

(s-8-5)=
## 8.5 Источник, сессия и порядок

<p>Ключ наблюдения: source_instance_id + session_id + frame_sequence, с header.stamp как временем измерения. source_instance_id генерируется процессом perception. Сессия увеличивается при скачке входного stamp назад. Монитор принимает новый источник, переводит предыдущий в retired и сбрасывает temporal; возврат retired-источника отклоняется до перезапуска монитора.</p>

<div class="table-caption">Таблица 19. Отказ в принятии FrameAnalysis</div>

<table><colgroup><col style="width:48%"/><col style="width:52%"/></colgroup><thead><tr><th>Проверка</th><th>last_rejection_reason</th></tr></thead><tbody><tr><td>Пустой source_instance_id</td><td>INVALID_SOURCE_ID</td></tr><tr><td>Некорректный stamp</td><td>INVALID_STAMP</td></tr><tr><td>sequence=0</td><td>INVALID_SEQUENCE</td></tr><tr><td>Вернулся retired-источник</td><td>RETIRED_SOURCE</td></tr><tr><td>Сессия меньше принятой</td><td>OLD_SESSION</td></tr><tr><td>sequence не растёт</td><td>DUPLICATE_OR_LATE_SEQUENCE</td></tr><tr><td>stamp ушёл назад в той же сессии</td><td>STAMP_WENT_BACKWARDS</td></tr><tr><td>Возраст NaN/Inf либо отрицательный</td><td>INVALID_PROCESSING_AGE</td></tr></tbody></table>

<p>Rejected analysis не обновляет принятый ключ и не освежает watchdog. Повторное измерение с тем же measurement timestamp не считается новым evidence для temporal, даже если транспорт доставил его повторно. Длинный интервал между анализами больше max_gap_s=0,5 или скачок времени назад очищает треки.</p>

(s-8-6)=
## 8.6 Heartbeat и stale

<p>obstacle_monitor_node публикует результат после принятого analysis и wall-timer каждые 100 мс. Heartbeat сохраняет header исходного наблюдения и frame_sequence. Когда result_age_ms превышает timeout_s×1000, output становится UNKNOWN / INPUT_PAUSED_OR_STOPPED; stale=true, distance_m=NaN, reported_objects, corridor и tracks очищаются. Ключ наблюдения сохраняется, чтобы потребитель видел, какое наблюдение устарело.</p>

<p>До первого анализа результат WAITING_FOR_INPUT уже имеет stale=true. Timeout оценивает возраст принятого результата, включая исходный processing_age_ms, поэтому он не отсчитывает новую полную половину секунды от сообщения, которое уже было старым.</p>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>core/src/temporal_<wbr/>monitor.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/assessment_<wbr/>monitor.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/obstacle_<wbr/>monitor_<wbr/>node.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/include/metro_<wbr/>perception_<wbr/>ros/frame_<wbr/>key_<wbr/>gate.hpp</code>.</div>
