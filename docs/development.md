# Работа над Metro Perception

**Актуальный режим по умолчанию:** [lidar-only-default.md](lidar-only-default.md).
Для проверки приватных bag измеренная калибровка не требуется.

[Полный обновлённый план](../PLAN.md) · [Архитектура](architecture.md) ·
[Форматы данных](../evaluation/README.md) · [Задачи](work-items.md)

## Что работает в каркасе

Пять пакетов собираются на Humble. C++-адаптер читает XYZ FLOAT32/FLOAT64,
разные offsets, порядок полей, endian и row padding. Ring необязателен.
Нода публикует FrameAnalysis; monitor выдаёт UNKNOWN и heartbeat, при остановке
входа — INPUT_PAUSED_OR_STOPPED. Visualizer показывает текст статуса.
Sequential evaluate_bag использует тот же адаптер и core и записывает JSONL.
inspect_bag выводит метаданные и первые N схем облаков; metrics/report —
счётчики состояний и время, без выдуманных TP/FP/FN.

**Детектора ещё нет.** Реализован A02: очистка, TF на stamp, два ROI и raw indices.
В строгом режиме без TF получается TF_UNAVAILABLE, с неподтверждённой калибровкой —
CALIBRATION_UNVERIFIED; после валидного A02 — NOT_IMPLEMENTED.
Ground, corridor, clustering, bbox и временное подтверждение ещё не реализованы.
[Профили калибровки и запуск без TF в bag](calibration.md).
Оценки габарита/монтажа в YAML оставлены null. PointCloud callback теперь только
принимает сообщение и заменяет latest pending slot; decode/A02 выполняет один worker.
В памяти не накапливается backlog: максимум один processing + один pending кадр.
overwritten_frames считает вытесненные pending кадры, max_processing_age_s не даёт
публиковать устаревший результат. Каркас не доказывает реальное время, дальность
или точность.

## Зоны работы

| Зона | Каталоги | Ответственный по плану |
|---|---|---|
| Геометрия, алгоритм, core-тесты | metro_perception_core | A |
| Адаптер, TF, worker, monitor, evaluator | metro_perception_ros | B |
| Сообщения, launch/config/RViz, Docker/CI | interfaces, bringup, docker, .github | C |
| Инспекция, разметка, метрики | tools, evaluation | T1 |
| Smoke, ошибки входа, нагрузка | scripts, ros/test | T2 |

Сообщения и core types — черновой API. Согласовывать изменения с потребителями,
собирать все пять пакетов после изменения msg. Пакеты остаются в корне checkout;
не создавать второй вложенный src. Новые стадии добавлять вместе с реализацией.

## Сборка

В Humble Dev Container или рабочем окружении с ROS 2 Humble:

```bash
bash scripts/build.sh
source install/local_setup.bash
bash scripts/test.sh
python3 scripts/smoke_perception.py
python3 scripts/smoke_a02.py
```

Зависимости берутся из package.xml:

```bash
rosdep install --from-paths metro_perception_* --ignore-src --rosdistro humble -y
```

Существующий depth_image запускается как раньше. Каркас отдельно:

```bash
ros2 launch metro_perception_bringup perception.launch.py input_topic:=/lidar_points
# Во втором терминале с той же ROS-средой:
ros2 bag play /data/my_bag
# В третьем терминале:
ros2 topic echo /metro/assessment
```

Все процессы должны иметь одинаковые ROS_DOMAIN_ID/RMW. В стандартном Docker
они работают в одном контейнере. GUI не нужен для smoke или evaluator.

## Инспекция и последовательный прогон

```bash
mkdir -p /results/run-001
ros2 run metro_perception_tools inspect_bag /data/my_bag --sample-count 2 --output /results/run-001/inspection.json
ros2 run metro_perception_ros evaluate_bag /data/my_bag /lidar_points /results/run-001/frames.jsonl
ros2 run metro_perception_tools metrics /results/run-001/frames.jsonl --output /results/run-001/summary.json
ros2 run metro_perception_tools report /results/run-001/summary.json --output /results/run-001/report.md
```

Инструменты отказываются перезаписывать результат. Код 0 evaluator означает
успешный экспорт, а не реализованный детектор: каждая строка содержит
`mode=a02`, UNKNOWN и невалидную область. Пятый аргумент evaluate_bag — путь
к sensor profile YAML. Статический профиль, TF resolver и A02 общие с нодой;
динамический TF из bag evaluator пока не воспроизводит. При изменении ROS-параметра
max_points относительно стандартных 2 000 000 этот override не переносится в evaluator.
Полная runtime/config parity остаётся в E01; worker 1+1 реализован в R01.

Для всех bag из паспорта набора:

```bash
python3 scripts/evaluate_all.py --dataset-root /data --output-dir /results/run-002
```

Скрипт сохраняет SHA, признак dirty checkout, hashes bag/config, manifest запуска
и отдельные результаты по каждой записи. Он не выбирает пороги по имени bag.
В готовом образе скрипты репозитория отдельно примонтировать/передать через stdin;
ROS executables и параметры устанавливаются в /ws/install.

## Уточнения пятидневного плана

- Во второй день B сначала завершает node/monitor; evaluator допускается закончить
  на третий день. До этого baseline оценивается на фиксированных кадрах.
- E02 P0 — измерение B0; сравнение B0/B1 выполняется после появления B1.
- WSL2, composition и NVIDIA не блокируют Ubuntu/headless P0.
- CI включён как рабочая заготовка. Внешний GitHub run считается проверенным
  только после фактического запуска workflow.
- Для lidar-only сдачи измеренный монтаж не обязателен для forward-sector:
  используется explicit ASSUMED-профиль. Full-scan без установленной оси движения
  остаётся UNKNOWN; универсального поворота для всех лидаров нет.

Полный исходный PLAN.md сохранён как основание. Этот раздел уточняет порядок
реализации; проценты/галочки готовности нельзя переносить из наличия файлов.
