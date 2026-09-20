# Набор данных и разметка — T1

`dataset.yaml` содержит пути относительно внешнего корня данных и сведения из
metadata.yaml. Counts имеют статус declared: пересчитать фактически прочитанные
сообщения при полном аудите, поскольку metadata бывает несогласованной.
`input_topic` выбирает источник, а не пороги алгоритма.

В `splits.yaml` пока нет назначения: просмотренный bag нельзя автоматически
объявить независимым holdout. Зафиксировать split до подбора параметров.

Скопировать `annotations/template.yaml` в файл с bag ID, заполнить интервалы
**в bag timestamps, наносекунды**. Положительное событие подтверждается по
PointCloud2, а расстояние измеряется до принятой поверхности от sensor origin.
Пустой events при reviewed=false означает «не размечено», а не negative bag.
Для отрицательных интервалов явно заполнить reviewed_intervals.

`experiments.yaml` — очередь сравнений, не готовые результаты.
Данные и generated frames/CSV/видео хранятся в rosbags/results и не попадают
в образ или Git. В Git остаются разметка, выбранные параметры и методика.

JSONL schema v1 каркаса: bag_id, session_id, frame_sequence, measurement_stamp_ns,
bag_stamp_ns, state, reason, distance_m (null при отсутствии), candidate_count,
evaluation_region_valid, processing_ms, mode=scaffold. Идентификаторы и времена
— целые числа; NaN/Infinity запрещены. Полный формат кандидатов и сопоставление
с разметкой предстоит добавить в E01/E02 до заявления метрик качества.
