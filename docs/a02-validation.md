# Проверка A02 на текущем наборе

Исторический прогон строгого preview. Текущий default разрешает допущенную калибровку:
[lidar-only-default.md](lidar-only-default.md).

Дата: 21.09.2026. ROS 2 Humble, отдельный контейнер, исходные bag смонтированы read-only.

Прочитаны и обработаны **13 759 кадров из всех 7 bag**. Числа совпали с metadata.
На каждом кадре применён static TF preview, выполнены очистка и ROI. Все результаты:
`INVALID_GEOMETRY / CALIBRATION_UNVERIFIED`, `UNKNOWN`, `distance_m=null`,
`evaluation_region_valid=false`. Ошибок декодирования и TF lookup в этом прогоне нет.

Это проверка обработки входа и защитных условий; монтаж и качество детектора не подтверждены.

| Bag | Кадров | Geometry ROI, min–max точек | Detection ROI, min–max точек | Сессий |
|---|---:|---:|---:|---:|
| doubleT_obstacle | 201 | 340810–345993 | 180807–184157 | 1 |
| doubleT_platform | 345 | 154856–190378 | 114103–180063 | 1 |
| roundT_doubleT | 252 | 177707–191154 | 117305–190015 | 1 |
| roundT_pressureGate_roundT | 268 | 160550–190770 | 160271–190493 | 1 |
| roundT_squareT_pressureGate_squareT | 545 | 167068–191076 | 150986–190715 | 1 |
| squareT_platform_squareT_switch | 877 | 156046–189993 | 126359–189788 | 1 |
| new_data | 11271 | 152356–191094 | 75550–190374 | 1 |

Число сессий увеличивается при скачке header.stamp назад. TF выбирается на времени
измерения; bag record time отдельно сохранён в каждой строке JSONL.

Профили: `forward_sector_preview.yaml` для /lidar_points и
`full_scan_preview.yaml` для /sensing/lidar/hesai128/pointcloud.
В обоих случаях target=lidar_preview, Rz(+π/2), translation=0,
calibration_verified=false. Направление полного скана не подтверждено.

## Автоматические проверки

- Сборка всех пяти пакетов Humble прошла.
- colcon: 19 тестов, 0 ошибок; после изменения метрик отдельно повторены Python-тесты tools: 7 passed.
- scripts/test: 13 passed.
- Core: invalid/zero до translation, blind mask, rotation, два ROI, raw indices,
  invalid transform/config, отсутствие накопления.
- TF: интерполяция на header.stamp, отказ при missing/extrapolation/zero stamp.
- smoke_perception: missing TF → UNKNOWN, steady watchdog без /clock.
- smoke_a02: одинаковые online/offline счётчики и статусы, поздний подписчик /tf_static,
  новая session при скачке stamp назад, UNKNOWN и clean shutdown.
- Собран отдельный runtime metro-lidar:a02-check; оба smoke прошли
  без исходников, датасета, сети и GUI. Текущий desktop-контейнер не изменялся.

Сырые JSONL и summary.json с hash исходников/результатов сохранены локально в
`results/a02-20260921/` (исключены из Git). Там же логи сборки и smoke.
Команды воспроизведения: [calibration.md](calibration.md).

A03–A07, worker 1+1 и временное подтверждение кандидатов не входят в этот результат.
D03 остаётся открытым: нет измеренной установки лидара относительно поезда.
Динамический TF из bag и кастомные ROS overrides в offline пока не поддержаны.
GUI/RViz интерактивно не проверялся. Timings A02 не являются оценкой будущего детектора.
