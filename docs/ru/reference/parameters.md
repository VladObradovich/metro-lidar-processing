(chapter-А)=
# А Полный справочник параметров

```{include} ../../_generated/ru/parameter-summary.html
```

<p>C++ — default структуры, «Профиль» — эффективное значение forward_sector_assumed.yaml после overrides. Boolean записаны как true/false. Все числовые double-параметры должны быть конечными. Сведения о диапазонах отражают validate(); для точных взаимных ограничений авторитетен config.hpp. Смена default структуры не меняет значение, явно записанное в YAML.</p>

(s-A-1)=
## А.1 Вход и диагностические флаги

```{include} ../../_generated/ru/parameters-01.html
```

(s-A-2)=
## А.2 Габарит и поверхность

```{include} ../../_generated/ru/parameters-02.html
```

(s-A-3)=
## А.3 Угловые компоненты и фон

```{include} ../../_generated/ru/parameters-03.html
```

(s-A-4)=
## А.4 Ось пути и неопределённость

```{include} ../../_generated/ru/parameters-04.html
```

(s-A-5)=
## А.5 Обычный GAUGE и головка рельса

```{include} ../../_generated/ru/parameters-05.html
```

(s-A-6)=
## А.6 Дальнее расширение

```{include} ../../_generated/ru/parameters-06.html
```

(s-A-7)=
## А.7 Низкие объекты, подвесы и колонны

```{include} ../../_generated/ru/parameters-07.html
```

(s-A-8)=
## А.8 Собственное движение

```{include} ../../_generated/ru/parameters-08.html
```

(s-A-9)=
## А.9 Временное подтверждение

```{include} ../../_generated/ru/parameters-09.html
```

(s-A-10)=
## А.10 Верхний уровень сенсорного профиля

<div class="table-caption">Таблица 56. Поля профиля</div>

<table><colgroup><col style="width:32%"/><col style="width:25%"/><col style="width:43%"/></colgroup><thead><tr><th>Ключ</th><th>Штатное значение</th><th>Семантика</th></tr></thead><tbody><tr><td><code>source_<wbr/>frame_<wbr/>mode</code></td><td>bind_first</td><td>exact либо bind_first; verified требует exact.</td></tr><tr><td><code>source_<wbr/>frame</code></td><td>null</td><td>В bind_first не задаётся; в exact обязательно корректное имя.</td></tr><tr><td><code>target_<wbr/>frame</code></td><td>lidar_assumed</td><td>Фрейм геометрии после transform.</td></tr><tr><td><code>calibration_<wbr/>verified</code></td><td>false</td><td>Измеренная/проверенная калибровка должна иметь источник.</td></tr><tr><td><code>allow_<wbr/>unverified_<wbr/>calibration</code></td><td>true</td><td>Явное разрешение работать с ASSUMED.</td></tr><tr><td><code>translation_<wbr/>m</code></td><td>[0,0,0]</td><td>Три конечных компонента переноса, м.</td></tr><tr><td><code>rotation_<wbr/>rpy_<wbr/>rad</code></td><td>[0,0,π/2]</td><td>Roll/pitch/yaw, рад; задаются вместе с translation.</td></tr><tr><td><code>calibration_<wbr/>source</code></td><td>Текст гипотезы</td><td>Происхождение transform; непусто для VERIFIED.</td></tr><tr><td><code>geometry_<wbr/>roi</code></td><td>min/max</td><td>Широкая геометрия; bounds см. 7.3.</td></tr><tr><td><code>detection_<wbr/>roi</code></td><td>min/max</td><td>Область анализа, вложенная в geometry ROI.</td></tr><tr><td><code>detector</code></td><td>mapping</td><td>Overrides AlgorithmConfig; не ROS parameters.</td></tr><tr><td><code>temporal</code></td><td>mapping</td><td>Overrides TemporalConfig для monitor и evaluator.</td></tr></tbody></table>

<div class="source">Основание: <code>metro_<wbr/>perception_<wbr/>core/include/metro_<wbr/>perception_<wbr/>core/config.hpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/src/preprocessing.cpp</code>; <code>metro_<wbr/>perception_<wbr/>ros/config/forward_<wbr/>sector_<wbr/>assumed.yaml</code>.</div>
