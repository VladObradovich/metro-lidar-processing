(chapter-14)=
# 14 Разработка и проверка изменений

<p class="lead">Организация кода, тестирование и добавление нового параметра без рассогласования контракта.</p>

(s-14-1)=
## 14.1 Рабочий цикл

<div class="codebox"><div class="code-label">DEV-КОНТЕЙНЕР: КОРЕНЬ CHECKOUT</div><pre>bash scripts/build.sh
source install/local_setup.bash
bash scripts/test.sh
python3 scripts/smoke_monitor.py
python3 scripts/smoke_detector.py</pre></div>

<p>Для локальной итерации допускается выбрать затронутые пакеты аргументом --packages-select. Перед проверкой полного изменения запускать весь Humble-набор. Поведение состояния, TF/сессий и геометрического детектора должно быть покрыто регрессиями, которые воспроизводят значимое изменение контракта.</p>

(s-14-2)=
## 14.2 Карта unit-тестов

<div class="table-caption">Таблица 41. Группы тестов</div>

<table><colgroup><col style="width:36%"/><col style="width:64%"/></colgroup><thead><tr><th>Группа</th><th>Что проверяется</th></tr></thead><tbody><tr><td>Core pipeline</td><td>Вход, фильтрация, transform, ROI, геометрия, кандидаты и интегрированный детектор.</td></tr><tr><td>Core temporal / speed</td><td>Подтверждение и release, coasting, расстояние, движение, сброс времени и треков.</td></tr><tr><td>ROS pointcloud / preprocessing</td><td>Layout и endian, limits, профиль, привязка фрейма, exact-stamp transform.</td></tr><tr><td>ROS gates / worker</td><td>FrameKeyGate, SessionGate, latest-only слот, безопасный enum decode.</td></tr><tr><td>AssessmentMonitor</td><td>Reported objects, trust, возраст, stale, rejected analyses и ключ наблюдения.</td></tr><tr><td>Python metrics / projection</td><td>Разметка и ROI matching, split/holdout, depth image.</td></tr><tr><td>scripts/test</td><td>Инструменты оценки, синтетика, false-alarm events, desktop/X11/scale.</td></tr><tr><td>RViz text</td><td>Форматирование assessment независимо от Qt.</td></tr></tbody></table>

(s-14-3)=
## 14.3 Интеграционные smoke

<div class="table-caption">Таблица 42. Интеграционные инструменты</div>

<table><colgroup><col style="width:43%"/><col style="width:57%"/></colgroup><thead><tr><th>Скрипт</th><th>Контракт / сценарий</th></tr></thead><tbody><tr><td><code>smoke_<wbr/>perception.py</code></td><td>Запуск онлайн цепочки и базового вывода.</td></tr><tr><td><code>smoke_<wbr/>a02.py</code></td><td>Подготовка и онлайн/офлайн согласованность; --default проверяет штатный профиль.</td></tr><tr><td><code>smoke_<wbr/>monitor.py</code></td><td>Порядок анализа, gate, heartbeat и watchdog.</td></tr><tr><td><code>smoke_<wbr/>detector.py</code></td><td>Геометрическое обнаружение на контролируемом облаке.</td></tr><tr><td><code>smoke_<wbr/>ego_<wbr/>motion.py</code></td><td>Компенсация собственного движения.</td></tr><tr><td><code>smoke_<wbr/>namespace_<wbr/>isolation.py</code></td><td>Раздельные потоки в разных namespace.</td></tr><tr><td><code>smoke_<wbr/>worker_<wbr/>overload.py</code></td><td>Latest-only и перегрузка worker.</td></tr><tr><td><code>smoke_<wbr/>tf_<wbr/>replay.py</code></td><td>Офлайн TF replay и регрессии времени.</td></tr><tr><td><code>smoke_<wbr/>runtime.py</code></td><td>Внешний bag, непустые depth-кадры и декодирование сохранённого видео в installed runtime.</td></tr></tbody></table>

<p>CI выполняет lint, сборку и package tests в Humble, затем smoke. Отдельные jobs собирают установленный headless runtime и desktop runtime. Smoke в runtime выполняются без source mount и с --network none. Успешный CI не заменяет детекторные метрики на размеченных bag.</p>

(s-14-4)=
## 14.4 Стиль кода

<ul><li>C++ production — C++17, Google-based .clang-format: отступ 2 пробела, 100 колонок. Проверка clang-format --dry-run --Werror.</li><li>Python — 4 пробела, snake_case, одинарные кавычки, 99 колонок, ament_flake8 и ament_pep257.</li><li>Unit-тесты test_*.cpp / test_*.py; integration smoke_*.py. Core должен оставаться независимым от ROS.</li><li>Новые статусы/поля должны синхронно отражаться в core, .msg, online conversions, JSONL и scorer/tests.</li><li>Часть unit-тестов может использовать более новый C++ стандарт для тестового синтаксиса; это не меняет C++17-контракт production.</li></ul>

(s-14-5)=
## 14.5 Добавить параметр детектора

<ol><li>Добавить поле и разумный default в AlgorithmConfig или TemporalConfig.</li><li>В validate() проверить конечность, диапазон и взаимные ограничения; отключение обозначать явно, если предусмотрено.</li><li>Добавить YAML-чтение в load_preprocessing / load_temporal_config; не полагаться на ROS param loader для detector:-полей.</li><li>Изменить shipped-профиль только при принятом новом поведении; значения должны быть одинаковы online и offline.</li><li>Добавить регрессию, показывающую влияние параметра, и сохранить невариантные safety contracts.</li><li>Оценить object hits, false alarms, UNKNOWN, дальность и latency на допустимых выборках.</li><li>Зафиксировать commit, профиль и manifest в воспроизводимом прогоне; не подбирать по blind holdout.</li></ol>

(s-14-6)=
## 14.6 Расширить API

<p>Изменение .msg затрагивает генерацию интерфейсов и потребителей. Новые enum-коды должны безопасно декодироваться на ROS-границе: неизвестное значение не допускает «свободно». При добавлении bbox/полей валидности нужно обновлять both perception и offline export; поля, которые не вычислены, не объявлять достоверными по умолчанию.</p>

<p>Результат stale должен продолжать очищать объекты, corridor и tracks, сохраняя ключ наблюдения. Missing distance должен оставаться NaN/false в ROS и null/false в JSON. Повтор одного stamp не должен давать новые hits. Сессии и источники нельзя смешивать ради удобства графики.</p>

(s-14-7)=
## 14.7 Рабочие артефакты

<p>Входные bag, результаты и credentials не включаются в коммиты исходного кода. Для каждого изменения детектора сохраняйте отдельный каталог эксперимента, профиль и манифест; происхождение результатов должно быть проверяемым независимо от состояния рабочего дерева. Публикация и доставка выполняются после проверки артефактов, а не самим оценщиком.</p>

<div class="source">Основание: <code>.github/workflows/build-test.yaml</code>; <code>scripts/build.sh</code>; <code>scripts/test.sh</code>; <code>scripts/evaluate_<wbr/>all.py</code>; <code>metro_<wbr/>perception_<wbr/>core/CMakeLists.txt</code>; <code>metro_<wbr/>perception_<wbr/>ros/CMakeLists.txt</code>.</div>
