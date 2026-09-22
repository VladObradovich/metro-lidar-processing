# Поддержка GPU в desktop-контейнере

Desktop-контейнер поддерживает опциональное ускорение NVIDIA GPU.

## По умолчанию

Запустите desktop-контейнер без переопределений, специфичных для GPU:

```bash
bash scripts/desktop.sh up
```

С подключением дерева исходников в `/ws`:

```bash
bash scripts/desktop.sh up --source
```

## NVIDIA в Linux

Предварительные требования:

- Драйвер NVIDIA GPU установлен на хосте.
- NVIDIA Container Toolkit настроен для Docker.
- Следующий smoke-тест на стороне хоста проходит успешно:

```bash
docker run --rm --gpus all nvidia/cuda:13.0.0-base-ubuntu24.04 nvidia-smi
```

Запустите проект с:

```bash
bash scripts/desktop.sh up --nvidia
```

Или с монтированием исходников:

```bash
bash scripts/desktop.sh up --nvidia --source
```

Оверлей NVIDIA включает доступ к GPU и возможности графики/дисплея, необходимые RViz. Драйверы NVIDIA хоста не устанавливаются в образ проекта.

## NVIDIA в WSL2 / WSLg

Используйте ту же команду:

```bash
bash scripts/desktop.sh up --nvidia --source
```

Когда доступны `/dev/dxg` и `/usr/lib/wsl/lib`, скрипт запуска автоматически добавляет оверлей WSLg.

Графический путь WSLg использует:

- `/dev/dxg`
- `/usr/lib/wsl`
- Mesa D3D12

Конфигурация, специфичная для WSLg, намеренно вынесена из базового Compose-файла, чтобы не затрагивать пользователей нативного Linux.

## Проверка

Внутри контейнера используйте `glxinfo -B`, чтобы проверить рендерер OpenGL.

В WSL2/WSLg ускоренный рендеринг должен сообщать о рендерере D3D12 NVIDIA, например:

```text
Accelerated: yes
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)
```

В нативном Linux рендерер должен вместо этого сообщать о стеке OpenGL NVIDIA.

`nvidia-smi` можно использовать на хосте или в контейнере с поддержкой CUDA для проверки доступа к CUDA/NVML.
```