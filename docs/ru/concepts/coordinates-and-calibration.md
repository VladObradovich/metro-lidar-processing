(chapter-6)=
# 6 Координаты и калибровка

<p class="lead">Направление движения, смысл профилей и правила доверия к геометрии.</p>

(s-6-1)=
## 6.1 Целевая система координат

<p>Штатный forward_sector_assumed.yaml предполагает: вперёд поезда — отрицательная ось Y сенсора, влево — положительная X, вверх — положительная Z. Поворот вокруг Z на +π/2 переводит облако в lidar_assumed: x вперёд, y влево, z вверх. Перенос [0,0,0] оставляет начало в лидаре; это не произвольная система, привязанная к переднему краю вагона.</p>

<div class="codebox"><div class="code-label">ШТАТНЫЙ ПРОФИЛЬ: ГЕОМЕТРИЧЕСКОЕ ДОПУЩЕНИЕ</div><pre>x_target = -y_sensor
y_target =  x_sensor
z_target =  z_sensor
translation_m = [0, 0, 0]
rotation_rpy_rad = [0, 0, pi/2]</pre></div>

<p>Дальность вычисляется продольно по x относительно sensor_origin, не как евклидово расстояние до центра bbox и не как длина дуги пути. Для ненулевого переноса sensor_origin переносится в целевой фрейм вместе с облаком. Боковое смещение относительно оси пути используется отдельно для проверки габарита.</p>

<figure><img alt="Рисунок 2. Схема поперечного габарита, осей и высоты установки сенсора." src="/_static/diagrams/figure-02.svg"/><figcaption>Рисунок 2. Схема поперечного габарита, осей и высоты установки сенсора.</figcaption></figure>

(s-6-2)=
## 6.2 Два режима привязки source_frame

<div class="table-caption">Таблица 10. Привязка фрейма</div>

<table><colgroup><col style="width:20%"/><col style="width:50%"/><col style="width:30%"/></colgroup><thead><tr><th>Режим</th><th>Поведение</th><th>Применение</th></tr></thead><tbody><tr><td>bind_first</td><td>Имя фрейма связывается с первым пригодным входом сессии. Другой frame_id в той же сессии не принимается как тот же сенсор.</td><td>Предполагаемый штатный профиль; source_frame=null.</td></tr><tr><td>exact</td><td>frame_id должен совпасть с заданным source_frame.</td><td>Измеренная калибровка или явно известный фрейм.</td></tr></tbody></table>

<p>Verified-калибровка требует exact, явного имени source_frame и непустого calibration_source. В bind_first нельзя одновременно жёстко задать source_frame. Старое значение source_frame="*" читается как bind_first, но новые профили следует писать явно. Имя фрейма не должно быть пустым, начинаться с / или быть wildcard в exact-режиме.</p>

(s-6-3)=
## 6.3 Статический transform и внешний TF

<p>Если профиль содержит translation_m и rotation_rpy_rad, преобразование применяется локально в perception/evaluate_bag. По умолчанию его не публикуют глобально. Если статического преобразования нет, resolve_context ищет внешний TF строго на header.stamp; latest-TF не используется как резервная замена. Время ожидания ограничено tf_wait_timeout_s на steady clock.</p>

<p>Имя header.frame_id результата меняется на target_frame только после применённого transform; header.stamp сохраняется. Пустая ROI, некорректная матрица или недоступный TF не превращаются в пустой пригодный путь.</p>

<div class="table-caption">Таблица 11. Публикация TF для внешних потребителей</div>

<table><colgroup><col style="width:40%"/><col style="width:60%"/></colgroup><thead><tr><th>Аргумент launch</th><th>Когда нужен</th></tr></thead><tbody><tr><td><code>publish_<wbr/>sensor_<wbr/>tf:=true</code></td><td>Запускает static_transform_publisher из bringup. Для bind_first дополнительно нужен sensor_frame_override с реальным именем входного фрейма.</td></tr><tr><td><code>publish_<wbr/>bound_<wbr/>transform:=true</code></td><td>Позволяет perception публиковать transform профиля после привязки source_frame. По умолчанию false.</td></tr><tr><td><code>fixed_<wbr/>frame:=lidar_<wbr/>assumed</code></td><td>Fixed frame RViz. Для преобразованного labelled_points внешняя публикация TF обычно не требуется.</td></tr></tbody></table>

(s-6-4)=
## 6.4 Три уровня доверия

<div class="table-caption">Таблица 12. CalibrationTrust</div>

<table><colgroup><col style="width:12%"/><col style="width:23%"/><col style="width:65%"/></colgroup><thead><tr><th>Код</th><th>Уровень</th><th>Последствие</th></tr></thead><tbody><tr><td>0</td><td>UNKNOWN</td><td>Нет пригодного основания для ответа «свободно».</td></tr><tr><td>1</td><td>ASSUMED</td><td>Применено явно разрешённое геометрическое допущение. Не означает измеренную установку.</td></tr><tr><td>2</td><td>VERIFIED</td><td>Профиль помечен как проверенный; источник калибровки должен быть задан.</td></tr></tbody></table>

<aside class="important"><strong>Действующее правило ASSUMED</strong><p>В shipped-профиле assumed_clear=true. При отсутствии кандидатов и пригодной области не короче 50 м монитор может выдать NO_OBSTACLE_DETECTED, сохраняя ASSUMED. Если assumed_clear=false или область короче, результат UNKNOWN. Старый комментарий в PathAssessment.msg о полном запрете «свободно» при ASSUMED не отражает нынешнее правило.</p></aside>

(s-6-5)=
## 6.5 Как выбрать профиль

<div class="table-caption">Таблица 13. Профили и пределы использования</div>

<table><colgroup><col style="width:36%"/><col style="width:43%"/><col style="width:21%"/></colgroup><thead><tr><th>Профиль</th><th>Смысл</th><th>Решение</th></tr></thead><tbody><tr><td><code>forward_<wbr/>sector_<wbr/>assumed.yaml</code></td><td>Текущая гипотеза −Y вперёд, привязка фрейма на сессию, актуальные геометрия и temporal.</td><td>Штатный профиль; ASSUMED.</td></tr><tr><td><code>full_<wbr/>scan_<wbr/>unresolved.yaml</code></td><td>Полный скан, направление поезда не установлено; native frame lidar_livox.</td><td>UNKNOWN; профиль запрещает unverified calibration.</td></tr><tr><td><code>full_<wbr/>scan_<wbr/>research_<wbr/>assumed.yaml</code></td><td>Архивное сравнение с exact lidar_livox; прежний габарит 2×3,5 м.</td><td>Исследовательский, не актуальный стандартный детектор.</td></tr><tr><td><code>config/sensors/*_<wbr/>preview.yaml</code></td><td>Исторические preview-профили bringup.</td><td>Диагностика, а не штатная конфигурация.</td></tr></tbody></table>

<p>evaluate_all.py выбирает профиль по sensor_profile в dataset.yaml. В текущей версии и forward_sector, и full_scan по умолчанию отображаются на forward_sector_assumed.yaml. Это явная гипотеза оценки известных записей; она не доказывает калибровку нового полного скана. Опция --full-scan-unresolved меняет full_scan на профиль с неустановленной ориентацией.</p>

<div class="keep"><h2 id="s-6-6">6.6 Подключение нового сенсора</h2><ol><li>Проверить bag info: тип, топик, frame_id, частоту, структуру XYZ и объём сообщения.</li><li>Установить направление движения и высоту относительно рельса. При неизвестной ориентации сохранить UNKNOWN.</li><li>Создать отдельный YAML-профиль по геометрии сенсора; exact для измеренной калибровки, bind_first для явного допущения.</li><li>Убедиться, что detection_roi вложена в geometry_roi, а значения находятся в допустимых пределах.</li><li>Проверить онлайн/офлайн согласованность и разметку нескольких сцен, включая пустые, изгибы, станции и объекты у края.</li><li>Зарегистрировать тип профиля в dataset-метаданных без подбора порогов по имени отдельной записи.</li></ol></div>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>ros/config/forward_<wbr/>sector_<wbr/>assumed.yaml</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/preprocessing.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/evaluate_<wbr/>bag.cpp</code>; <code>scripts/evaluate_<wbr/>all.py</code>.</div>
