# Ядро — A

Рабочая C++17-библиотека без ROS-типов. `PerceptionPipeline` пока возвращает
`NOT_IMPLEMENTED`, `TemporalMonitor` — только `UNKNOWN`. Это проверяемый каркас,
а не детектор. Координаты `FrameInput` пока остаются в исходном frame лидара.

Следующие задачи: A02–A07 из `../PLAN.md`. Добавлять `preprocessing`,
`ground_estimator`, `corridor_estimator`, `clusterer`, `object_validator`
в `include/metro_perception_core/` и `src/` по мере реализации, с тестами в `test/`.
X03 (подтверждение во времени) расширяет `TemporalMonitor`; watchdog остаётся
в ROS-обёртке. `AlgorithmConfig` не читает YAML: параметры преобразует ROS-слой.

PCL/Eigen добавить в package.xml/CMake вместе с первым использующим их кодом.
