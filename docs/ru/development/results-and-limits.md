(chapter-13)=
# 13 Зафиксированные результаты и пределы

<p class="lead">Числовые показатели сохранённых прогонов, сверенные с машинными результатами оценки.</p>

<aside class="important"><strong>Происхождение измерений</strong><p>Качество приведено по quality.json и manifest.json прогона final-f13e569; время — по тем же артефактам final-ba69723-idle. В обоих отчётах run_reproducible=true. Значения относятся к указанным историческим коммитам, а не к новому измерению нынешнего снимка bd03ba96d2be. Числа таблиц извлекаются из отдельной проверенной выборки машинных результатов.</p></aside>

(s-13-1)=
## 13.1 Ложные тревоги

<div class="table-caption">Таблица 36. Отрицательные выборки прогона f13e569</div>

<table><colgroup><col style="width:52%"/><col style="width:24%"/><col style="width:24%"/></colgroup><thead><tr><th>Запись / выборка</th><th>Отрицательных кадров</th><th>OBSTACLE на negative</th></tr></thead><tbody><tr><td><code>doubleT_<wbr/>platform</code></td><td>345</td><td>0</td></tr><tr><td><code>roundT_<wbr/>doubleT</code></td><td>252</td><td>0</td></tr><tr><td><code>roundT_<wbr/>pressureGate_<wbr/>roundT</code></td><td>268</td><td>0</td></tr><tr><td><code>roundT_<wbr/>squareT_<wbr/>pressureGate_<wbr/>squareT</code></td><td>545</td><td>0</td></tr><tr><td><code>squareT_<wbr/>platform_<wbr/>squareT_<wbr/>switch</code></td><td>877</td><td>0</td></tr><tr><td>new_data, первые 595 с</td><td>5950</td><td>26</td></tr><tr><td>new_data, слепая часть — только итог</td><td>5321</td><td>16</td></tr><tr><td>cloud_with_fake_obj: отрицательные кадры</td><td>833</td><td>0</td></tr></tbody></table>

<p>На пяти коротких отрицательных записях — 0 ложных кадров. Доля определённых решений на этих выборках составляет 73.4%–93.0%; при нулевых ложных тревогах это доля ответов «свободно». Оставшиеся кадры UNKNOWN не следует считать подтверждением отсутствия препятствия. В разработочной части new_data зафиксированы 26 ложных кадров.</p>

<p>Итог слепой части получен штатным holdout_alarms.py только как сумма: 16 тревожных кадров на 5 321 облаке. Индивидуальные события и сцены этого интервала для подготовки руководства не просматривались.</p>

(s-13-2)=
## 13.2 Синтетическая запись с объектами

<div class="table-caption">Таблица 37. Подтверждение вставленных объектов: object match и состояние кадра</div>

<table><colgroup><col style="width:43%"/><col style="width:12%"/><col style="width:14%"/><col style="width:14%"/><col style="width:17%"/></colgroup><thead><tr><th>Объект в габарите</th><th>Кадров</th><th>Object hits</th><th>OBSTACLE</th><th>Первое совпадение, м</th></tr></thead><tbody><tr><td>Короб 2×2 по центру</td><td>231</td><td>78</td><td>78</td><td>64.2</td></tr><tr><td>Куб 0,3 м по центру</td><td>24</td><td>23</td><td>24</td><td>40.8</td></tr><tr><td>Куб 0,3 м на рельсе</td><td>24</td><td>16</td><td>16</td><td>42.7</td></tr><tr><td>Куб 0,3 м у края габарита</td><td>24</td><td>6</td><td>6</td><td>33.1</td></tr><tr><td>Короб 2×2 у края габарита</td><td>102</td><td>0</td><td>0</td><td>—</td></tr><tr><td>Длинный низкий на рельсах</td><td>50</td><td>26</td><td>26</td><td>63.8</td></tr><tr><td>Тонкая полоса сверху</td><td>10</td><td>2</td><td>2</td><td>8.4</td></tr></tbody></table>

<p>Подтверждён хотя бы раз нужный объект в 6 из 7 положительных событий. На 3 объектах вне/выше габарита object-level тревог — 0. Средняя абсолютная ошибка продольной дистанции — 0.029 м; максимум — 0.297 м на 151 совпадениях.</p>

<p>Колонки Object hits и OBSTACLE отвечают на разные вопросы. У центрального куба 24 тревожных кадра, но 23 совпадения с нужным объектом: одно решение относится к иной области. Ошибка дистанции считается только на совпадениях; ненайденный короб у края не входит в выборку ошибки.</p>

(s-13-3)=
## 13.3 Реальный положительный пример

<div class="table-caption">Таблица 38. События doubleT_obstacle по сохранённой оценке</div>

<table><colgroup><col style="width:40%"/><col style="width:24%"/><col style="width:18%"/><col style="width:18%"/></colgroup><thead><tr><th>Событие</th><th>Кадров в интервале</th><th>OBSTACLE</th><th>Object hits</th></tr></thead><tbody><tr><td>Ближний человек</td><td>56</td><td>12</td><td>0</td></tr><tr><td>Дальний человек</td><td>201</td><td>68</td><td>56</td></tr></tbody></table>

<p>Интервалы двух людей перекрываются. На интервале ближнего человека есть 12 тревог, но ни одного совпадения подтверждённого объекта с его ROI. Дальнему человеку соответствуют 56 object hits. Поэтому наличие OBSTACLE во временном интервале само по себе не доказывает обнаружение именно размеченного человека.</p>

(s-13-4)=
## 13.4 Производительность

<div class="table-caption">Таблица 39. Офлайн, Release; сохранённый прогон ba69723</div>

<table><colgroup><col style="width:55%"/><col style="width:15%"/><col style="width:15%"/><col style="width:15%"/></colgroup><thead><tr><th>Bag</th><th>p50, мс</th><th>p95, мс</th><th>p99, мс</th></tr></thead><tbody><tr><td><code>doubleT_<wbr/>obstacle</code></td><td>32.3</td><td>35.5</td><td>37.7</td></tr><tr><td><code>doubleT_<wbr/>platform</code></td><td>31.7</td><td>37.8</td><td>39.0</td></tr><tr><td><code>roundT_<wbr/>doubleT</code></td><td>23.8</td><td>31.5</td><td>33.4</td></tr><tr><td><code>roundT_<wbr/>pressureGate_<wbr/>roundT</code></td><td>32.0</td><td>50.5</td><td>56.3</td></tr><tr><td><code>roundT_<wbr/>squareT_<wbr/>pressureGate_<wbr/>squareT</code></td><td>44.4</td><td>53.1</td><td>56.0</td></tr><tr><td><code>squareT_<wbr/>platform_<wbr/>squareT_<wbr/>switch</code></td><td>42.8</td><td>52.2</td><td>64.7</td></tr><tr><td><code>new_<wbr/>data</code></td><td>30.6</td><td>48.9</td><td>75.3</td></tr><tr><td><code>cloud_<wbr/>with_<wbr/>fake_<wbr/>obj</code></td><td>48.5</td><td>57.4</td><td>61.3</td></tr></tbody></table>

<p>Время из quality.json включает обработку облака офлайн: декодирование, подготовку, детектор и временной монитор. Release-режим записан в build_info манифеста. В этих артефактах модель CPU не зафиксирована, поэтому таблица не задаёт аппаратную гарантию и не переносится автоматически на другую машину.</p>

<p>Онлайн отдельно измеряют processing_age_ms, queue_age_ms, tf_wait_ms, result_age_ms, overwritten_frames и доставленную частоту. Онлайн-возраст включает локальное ожидание и отличается от processing_ms оценщика. Наличие офлайн p95 ниже периода входа не доказывает отсутствие перегрузки при ROS-транспорте и GUI.</p>

(s-13-5)=
## 13.5 Известные ограничения

<div class="table-caption">Таблица 40. Пределы модели и доказательств</div>

<table><colgroup><col style="width:31%"/><col style="width:69%"/></colgroup><thead><tr><th>Ограничение</th><th>Практическое последствие</th></tr></thead><tbody><tr><td>Наблюдаемая дальность</td><td>На пяти отрицательных записях медианная evaluated range составляет около 60–70 м. Это характеристика конкретных сцен, а не максимум ROI и не гарантированная дальность обнаружения.</td></tr><tr><td>Предметы у края</td><td>EDGE ограничивает подтверждение при неопределённой оси. Короб у края в сохранённом положительном событии не получил object hit.</td></tr><tr><td>Развилки / двухпутный тоннель</td><td>Ось строится по стенам; конструкция другого пути может попасть в предполагаемый габарит.</td></tr><tr><td>ASSUMED калибровка</td><td>Штатный профиль задаёт предполагаемую ориентацию. Другой монтаж требует собственной проверки.</td></tr><tr><td>Низкие предметы</td><td>Height, rail-head и low-bump фильтры требуют достаточной геометрической поддержки.</td></tr><tr><td>Положительный holdout</td><td>В реестре нет отдельной независимой положительной holdout-выборки.</td></tr><tr><td>Deskew / повороты</td><td>Нет компенсации времени отдельных точек и полной внешней траектории.</td></tr></tbody></table>

(s-13-6)=
## 13.6 Что важно проверить на новом стенде

<ol><li>Измерить онлайн processing/result age при 1× и при GUI; убедиться в Release-сборке runtime.</li><li>Проверить крупные облака на надёжность DDS-доставки и согласованность input QoS.</li><li>Проверить переход в UNKNOWN при потере входа, TF и опоры пола; heartbeat не должен освежать измерение.</li><li>Оценить реальные объекты по object match, а не по наличию любого OBSTACLE в интервале.</li><li>Проверить сложную геометрию, края габарита, низкие и подвешенные объекты на независимых данных.</li></ol>

<div class="source">Основание: <code>results/final-f13e569/quality.json</code>; <code>results/final-f13e569/manifest.json</code>; <code>results/final-ba69723-idle/quality.json</code>; <code>results/final-ba69723-idle/manifest.json</code>; <code>docs/data/metrics-evidence.json</code>.</div>
