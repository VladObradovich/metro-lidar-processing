#!/usr/bin/env python3
"""
Analyze PointCloud2 data in a rosbag2 bag without a running ROS graph.

Runs in the host venv (pip install rosbags matplotlib scipy) or in the
devcontainer. Does not import rclpy/sensor_msgs: PointCloud2 is decoded by hand
from the message's own field table.

Default mode: for each selected frame it drops empty returns (0,0,0 / NaN /
closer than --min-range) and reports the nearest point inside a "front" cone
around --front-center-deg. The metro lidar looks down the tunnel along -Y, so
the default center is -90 deg.

--clusters: detect objects that are not part of the empty tunnel, in the lidar's
own angular grid (ring x azimuth), so the same logic works from 5 m to 200 m:

  1. every frame becomes a range image  R[row, col]  (nearest return per cell);
  2. a baseline of the empty tunnel is the per-cell median of a few earlier
     frames (or of a fixed person-free moment, --baseline-static-at);
  3. a cell is foreground when it is nearer than the baseline by a margin, or
     returns where the baseline never did; only the track corridor is kept;
  4. foreground points are grouped with a per-point neighbor radius eps(R) that
     scales continuously with each point's own range (see adaptive_cluster),
     so 0-50/50-100/100-200/200-300 m all use the same formula and there is no
     cliff in behavior at the boundaries between them;
  5. a candidate is person-like if its cell count and metric size fit a person
     at *its own range* (thresholds scale with range, see looks_person);
  6. a person-like candidate is confirmed only if it persists over several
     consecutive analyzed frames near the same place (see confirm).
"""

import argparse
import sqlite3
import warnings
from collections import OrderedDict, deque
from contextlib import closing
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree

# sensor_msgs/PointField datatype -> numpy scalar code
POINT_FIELD_DTYPES = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}


def decode_points(message) -> np.ndarray:
    """Return a structured array with one record per point and one column per field."""
    byte_order = ">" if message.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for field in message.fields:
        scalar = POINT_FIELD_DTYPES.get(field.datatype)
        if scalar is None:
            raise ValueError(f"Unsupported datatype {field.datatype} in {field.name!r}")
        names.append(field.name)
        formats.append(f"{byte_order}{scalar}")
        offsets.append(field.offset)
    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": message.point_step,
        }
    )
    count = message.width * message.height
    return np.frombuffer(message.data, dtype=dtype, count=count)


def frame_stats(
    points: np.ndarray, min_range: float, center_deg: float, half_deg: float
) -> dict:
    """Range statistics for the valid points of one frame."""
    x = points["x"].astype(np.float64)
    y = points["y"].astype(np.float64)
    z = points["z"].astype(np.float64)
    ranges = np.sqrt(x * x + y * y + z * z)
    valid = np.isfinite(ranges) & (ranges > min_range)
    if not np.any(valid):
        return {
            "total": len(points),
            "valid": 0,
            "min_range": np.nan,
            "min_range_front": np.nan,
        }

    azimuth = np.degrees(np.arctan2(y[valid], x[valid]))
    offset = (azimuth - center_deg + 180.0) % 360.0 - 180.0
    in_front = np.abs(offset) <= half_deg
    valid_ranges = ranges[valid]
    return {
        "total": len(points),
        "valid": int(valid.sum()),
        "min_range": float(valid_ranges.min()),
        "min_range_front": (
            float(valid_ranges[in_front].min()) if np.any(in_front) else np.nan
        ),
    }


def sampled_timestamps(bag: Path, topic: str, every: int) -> list[int] | None:
    """
    Timestamps of every Nth message, read without touching message payloads.

    Returns None when the bag has no sqlite3 files (e.g. mcap); the caller then
    falls back to a full sequential scan.
    """
    db_files = sorted(bag.glob("*.db3"))
    if not db_files:
        return None
    timestamps = []
    for db_file in db_files:
        with closing(sqlite3.connect(f"file:{db_file}?mode=ro", uri=True)) as db:
            timestamps += [
                row[0]
                for row in db.execute(
                    "SELECT m.timestamp FROM messages m "
                    "JOIN topics t ON m.topic_id = t.id WHERE t.name = ?",
                    (topic,),
                )
            ]
    timestamps.sort()
    return timestamps[::every]


def iter_frames(reader, connections, bag: Path, topic: str, every: int):
    """
    Yield (frame_index, connection, timestamp, rawdata) for every Nth frame.

    With every > 1 only the selected messages are read from disk (indexed
    lookup by timestamp); a bag can be tens of GB, so reading all of it just to
    skip most frames is what this avoids.
    """
    selected = sampled_timestamps(bag, topic, every) if every > 1 else None
    if selected is None:
        for index, (connection, timestamp, rawdata) in enumerate(
            reader.messages(connections=connections)
        ):
            if index % every == 0:
                yield index, connection, timestamp, rawdata
        return
    for n, timestamp in enumerate(selected):
        for connection, ts, rawdata in reader.messages(
            connections=connections, start=timestamp, stop=timestamp + 1
        ):
            yield n * every, connection, ts, rawdata
            break


def read_points(reader, connections, timestamp: int) -> np.ndarray:
    """Прочитать ровно одно сообщение по его таймстемпу (индексный запрос)."""
    for connection, _, rawdata in reader.messages(
        connections=connections, start=timestamp, stop=timestamp + 1
    ):
        return decode_points(reader.deserialize(rawdata, connection.msgtype))
    raise SystemExit(f"No message at timestamp {timestamp}")


class Grid:
    """
    Угловая сетка range-картинки: строки - лучи по углу места, столбцы - азимут.

    Ячейка (row, col) - это направление. Ячейка охватывает d_el градусов
    по вертикали и az_res по горизонтали, поэтому её линейный размер
    на дальности R равен R*d (в радианах) и растёт с дальностью.
    Все пороги ниже выражены через эти углы, а не через метры.
    """

    def __init__(self, ring_row: np.ndarray, d_el_deg: float, az_res_deg: float):
        self.ring_row = ring_row
        self.n_rows = len(ring_row)
        self.d_el_deg = d_el_deg
        self.az_res_deg = az_res_deg
        self.n_az = int(round(360.0 / az_res_deg))

    @classmethod
    def from_points(cls, points, min_range, az_res_deg, ring_step_deg=None):
        """
        Определить порядок лучей и шаг по углу места по одному кадру.

        Номер луча (поле ring) - идентификатор лазера. Для каждого луча берём медиану угла места
            el = asin(z / r),
        сортируем лучи по el и присваиваем номер строки = место в сортировке.
        Шаг d_el = медиана разностей соседних отсортированных el.
        """
        x = points["x"].astype(np.float64)
        y = points["y"].astype(np.float64)
        z = points["z"].astype(np.float64)
        r = np.sqrt(x * x + y * y + z * z)
        valid = np.isfinite(r) & (r > min_range)
        ring = points["ring"].astype(np.int64)[valid]
        el = np.degrees(np.arcsin(z[valid] / r[valid]))
        n_rings = int(points["ring"].max()) + 1
        median_el = np.full(n_rings, np.nan)
        for k in range(n_rings):
            m = ring == k
            if m.any():
                median_el[k] = np.median(el[m])
        no_data = np.isnan(median_el)
        if no_data.all():
            raise SystemExit("Calibration frame has no valid returns")
        if no_data.any():
            # Луч без единого возврата в кадре: угол места неизвестен. Ставим его
            # по интерполяции от номера луча, что верно, только если номера идут
            # по возрастанию угла; иначе строки этих лучей будут перепутаны.
            known = np.flatnonzero(~no_data)
            median_el[no_data] = np.interp(
                np.flatnonzero(no_data), known, median_el[known]
            )
            print(
                f"Warning: {int(no_data.sum())} rings had no returns in the calibration "
                "frame; their vertical position is interpolated from the ring number"
            )
        order = np.argsort(median_el)
        ring_row = np.empty(n_rings, dtype=np.int64)
        ring_row[order] = np.arange(n_rings)
        if ring_step_deg is None:
            ring_step_deg = float(
                np.median(np.diff(np.sort(median_el[~np.isnan(median_el)])))
            )
        return cls(ring_row, ring_step_deg, az_res_deg)

    def corridor_columns(self, center_deg: float, half_deg: float) -> np.ndarray:
        """Маска столбцов, чей азимут лежит в секторе center +- half (по сетке)."""
        centers = -180.0 + (np.arange(self.n_az) + 0.5) * self.az_res_deg
        offset = (centers - center_deg + 180.0) % 360.0 - 180.0
        return np.abs(offset) <= half_deg

    def cell_size(self, range_m: float) -> tuple[float, float]:
        """Линейный размер ячейки (горизонталь, вертикаль) на дальности range_m, м."""
        return (
            range_m * np.radians(self.az_res_deg),
            range_m * np.radians(self.d_el_deg),
        )


def polar_cells(points: np.ndarray, grid: Grid, min_range: float):
    """
    Отнести точки к ячейкам сетки.

    r   = sqrt(x^2 + y^2 + z^2)                     дальность
    phi = atan2(y, x), phi in (-180, 180]            азимут
    col = floor((phi + 180) / az_res)                столбец
    row = ring_row[ring]                             строка (по углу места)
    cell = row * n_az + col                          плоский индекс

    Пустые возвраты (0,0,0), NaN и точки ближе min_range считаются невалидными.
    Возвращает (xyz, r, cell, valid).
    """
    x = points["x"].astype(np.float64)
    y = points["y"].astype(np.float64)
    z = points["z"].astype(np.float64)
    r = np.sqrt(x * x + y * y + z * z)
    valid = np.isfinite(r) & (r > min_range)
    ring = points["ring"].astype(np.int64)
    valid &= ring < grid.n_rows

    # Для невалидных точек подставляем безопасные значения (без atan2 от NaN).
    phi = np.degrees(np.arctan2(np.where(valid, y, 0.0), np.where(valid, x, 1.0)))
    col = np.clip(
        np.floor((phi + 180.0) / grid.az_res_deg).astype(np.int64), 0, grid.n_az - 1
    )
    row = grid.ring_row[np.where(valid, ring, 0)]
    return np.column_stack([x, y, z]), r, row * grid.n_az + col, valid


def range_image(points: np.ndarray, grid: Grid, min_range: float) -> np.ndarray:
    """
    Ближайшая дальность в каждой ячейке (нет возврата = NaN).

    В ячейку попадает несколько точек; берём минимальную дальность, то есть
    ближайшую поверхность на этом направлении: R[c] = min_i r_i.
    """
    _, r, cell, valid = polar_cells(points, grid, min_range)
    image = np.full(grid.n_rows * grid.n_az, np.inf, dtype=np.float32)
    np.minimum.at(image, cell[valid], r[valid].astype(np.float32))
    image[np.isinf(image)] = np.nan
    return image.reshape(grid.n_rows, grid.n_az)


class FrameSource:
    """
    Range-картинки кадров с небольшим LRU-кэшем по таймстемпу.

    Скользящий эталон каждый раз читает несколько соседних кадров; кэш не даёт
    перечитывать и пересчитывать одни и те же кадры для соседних позиций окна.
    """

    def __init__(self, reader, connections, grid: Grid, min_range: float, size=48):
        self.reader = reader
        self.connections = connections
        self.grid = grid
        self.min_range = min_range
        self.size = size
        self._cache: OrderedDict = OrderedDict()

    def image(self, timestamp: int, points=None) -> np.ndarray:
        if timestamp in self._cache:
            self._cache.move_to_end(timestamp)
            return self._cache[timestamp]
        if points is None:
            points = read_points(self.reader, self.connections, timestamp)
        image = range_image(points, self.grid, self.min_range)
        self._cache[timestamp] = image
        if len(self._cache) > self.size:
            self._cache.popitem(last=False)
        return image


def build_baseline(images) -> tuple[np.ndarray, np.ndarray]:
    """
    Эталон пустого тоннеля по набору range-картинок.

    Для ячейки c и кадров k = 1..B:
        base[c] = median_k R_k[c]   (NaN-значения пропускаются)
        frac[c] = (число кадров, где в c есть возврат) / B

    Медиана устойчива к выбросам с точкой разрыва 50%: пока объект закрывает
    ячейку меньше чем в половине кадров, base[c] остаётся дальностью пустого
    тоннеля. frac нужна, чтобы отличать "здесь стена на 40 м" от "здесь
    возврата не бывает" (тёмный дальний конец тоннеля).
    """
    stack = np.stack(images)
    with warnings.catch_warnings():
        # Ячейки, пустые во всех кадрах окна, дают "All-NaN slice" - это нормально.
        warnings.simplefilter("ignore", RuntimeWarning)
        base = np.nanmedian(stack, axis=0)
    return base, np.isfinite(stack).mean(axis=0)


def pick_images(source, stamps, lo: int, hi: int, count: int):
    """`count` кадров, равномерно выбранных из индексов stamps[lo:hi]."""
    picks = np.linspace(lo, max(lo, hi - 1), count).astype(int)
    return [source.image(int(stamps[i])) for i in sorted(set(picks))]


def rolling_baseline(source, stamps, timestamp: int, args):
    """
    Эталон для кадра в момент t по окну [t - L - S, t - L] (L=lag, S=span).

    Лидар может ехать, поэтому один эталон на всю запись не годится: геометрия
    меняется (на этой записи медиана |dr| между кадрами при сдвиге 0.1-5 с равна
    0.02-0.06 м, при 15 с - 0.5 м, при 60 с - 1.0 м). Окно должно быть свежим.
    Сдвиг L нужен, чтобы медленно движущийся объект не успел попасть в эталон.
    Ограничение: объект, стоящий на месте дольше L + S секунд, растворится в
    эталоне; для стоячего поезда используйте --baseline-static-at.

    Возвращает None, если истории до кадра меньше, чем L + S секунд.
    """
    end = timestamp - int(args.baseline_lag * 1e9)
    start = end - int(args.baseline_span * 1e9)
    if start < stamps[0]:
        return None
    lo, hi = np.searchsorted(stamps, [start, end])
    return build_baseline(pick_images(source, stamps, lo, hi, args.baseline_frames))


def static_baseline(source, stamps, args):
    """Фиксированный эталон по кадрам вокруг момента без людей (для стоячего лидара)."""
    start = int(stamps[0] + args.baseline_static_at * 1e9)
    end = start + int(args.baseline_span * 1e9)
    lo, hi = np.searchsorted(stamps, [start, end])
    if lo >= len(stamps):
        raise SystemExit("--baseline-static-at is beyond the end of the bag")
    return build_baseline(pick_images(source, stamps, lo, hi, args.baseline_frames))


def foreground_cells(image, base, frac, col_mask, args) -> np.ndarray:
    """
    Булева картинка переднего плана.

    Ячейка c с текущей дальностью R[c] (есть возврат) - передний план, если
      (A) "ближе":   R[c] < base[c] - m(c),   m(c) = margin + margin_rel*base[c]
      (B) "появился": frac[c] < empty_frac    (в эталоне возвратов не было)
    Запас m растёт с дальностью: шум и небольшие сдвиги геометрии дают
    относительную, а не абсолютную ошибку. Ячейки вне коридора отбрасываются.
    """
    has_return = np.isfinite(image)
    with np.errstate(invalid="ignore"):
        margin = args.margin + args.margin_rel * base
        nearer = has_return & (image < base - margin)
    appeared = has_return & (frac < args.empty_frac)
    return (nearer | appeared) & col_mask[None, :]


def eps_for_range(range_m: np.ndarray, grid: Grid, args) -> np.ndarray:
    """
    Радиус соседства eps(R): растёт с дальностью непрерывно, без разбиения на диапазоны.

        eps(R) = clip(eps_k * max(s_h(R), s_v(R)), eps_min, eps_max)

    s_h(R), s_v(R) - линейный размер ячейки сетки на дальности R (уже
    масштабируется с R сам по себе). Множитель eps_k (> 1) добавочно
    компенсирует то, что с ростом дальности возвраты не просто реже стоят по
    сетке, а ещё и чаще пропадают (более рваные разрывы), поэтому одной
    геометрии ячейки недостаточно. eps_min не даёт радиусу выродиться в почти
    ноль на короткой дистанции (там ячейка сама очень маленькая); eps_max не
    даёт ему на 200-300 м дотянуться до соседнего, не связанного объекта.
    Формула одна на весь диапазон 0-300 м: на границах 50/100/200 м она не
    делает скачка, значения eps слева и справа от границы почти совпадают.
    """
    size_h, size_v = grid.cell_size(range_m)
    return np.clip(args.eps_k * np.maximum(size_h, size_v), args.eps_min, args.eps_max)


def adaptive_cluster(
    xyz: np.ndarray, ranges: np.ndarray, grid: Grid, args
) -> np.ndarray:
    """
    Однослойная (single-link) кластеризация с радиусом, своим для каждой точки.

    Каждая точка i получает свой радиус eps_i = eps(r_i). Две точки объединяются
    в один кластер, если реальное расстояние между ними не больше БОЛЬШЕГО из
    их двух радиусов:
        union(i, j)  <=>  ||p_i - p_j|| <= max(eps_i, eps_j)
    Берём максимум, а не минимум или среднее, чтобы точка на границе диапазона
    (скажем, 99 м) корректно дотягивалась и до соседа на 101 м: её сосед там
    имеет больший eps, и по нему связь пройдёт, даже если у самой точки eps
    чуть меньше. Кандидатные пары ищутся k-d деревом (scipy.spatial.cKDTree)
    в радиусе max(eps) по всему кадру, затем каждая пара проверяется точно;
    объединение - через Union-Find (списки смежности не нужны).
    """
    n = len(xyz)
    if n == 0:
        return np.empty(0, dtype=np.int64)
    eps = eps_for_range(ranges, grid, args)
    parent = np.arange(n)

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    tree = cKDTree(xyz)
    pairs = tree.query_pairs(r=float(eps.max()), output_type="ndarray")
    if len(pairs):
        i, j = pairs[:, 0], pairs[:, 1]
        distance = np.linalg.norm(xyz[i] - xyz[j], axis=1)
        for a, b in pairs[distance <= np.maximum(eps[i], eps[j])]:
            union(int(a), int(b))
    labels = np.fromiter((find(i) for i in range(n)), dtype=np.int64, count=n)
    _, labels = np.unique(labels, return_inverse=True)
    return labels


def extract_candidates(points, image, mask, grid: Grid, args) -> list[dict]:
    """
    Кандидаты: группа точек переднего плана + метрические признаки по ним.

    В точки берутся только ячейки переднего плана, только ближайшая
    поверхность ячейки (r <= R[c] + cell_depth_tol; иначе в кандидат попали бы
    точки фона позади цели) и только |x| <= max_lateral (ось пути вдоль Y).
    Группировка - adaptive_cluster (радиус соседства свой для каждой точки).
    Отсева по минимальной поддержке здесь нет: на 200-300 м у настоящей цели
    может быть меньше ячеек, чем любой разумный фиксированный порог, поэтому
    порог считает только looks_person, тоже по формуле от дальности. Признаки:
    число занятых ячеек, медианная дальность R~, центр (среднее точек) и
    размер ограничивающего параллелепипеда size = max - min.
    """
    xyz, r, cell, valid = polar_cells(points, grid, args.min_range)
    mask_flat = mask.ravel()
    image_flat = image.ravel()
    with np.errstate(invalid="ignore"):
        keep = (
            valid
            & mask_flat[cell]
            & (r <= image_flat[cell] + args.cell_depth_tol)
            & (np.abs(xyz[:, 0]) <= args.max_lateral)
        )
    idx = np.flatnonzero(keep)
    if idx.size == 0:
        return []
    labels = adaptive_cluster(xyz[idx], r[idx], grid, args)

    candidates = []
    for label in np.unique(labels):
        members = idx[labels == label]
        pts = xyz[members]
        candidates.append(
            {
                "n_cells": int(np.unique(cell[members]).size),
                "n_pts": len(members),
                "range": float(np.median(r[members])),
                "center": pts.mean(axis=0),
                "size": pts.max(axis=0) - pts.min(axis=0),
            }
        )
    return candidates


def expected_cells(range_m: float, grid: Grid, args) -> float:
    """
    Сколько ячеек занимает человек номинального размера W x H на дальности R.

        n_exp = (W / (R * d_az)) * (H / (R * d_el)) = W*H / (R^2 * d_az * d_el)
    (d_az, d_el - углы ячейки в радианах). Убывает как 1/R^2.
    """
    size_h, size_v = grid.cell_size(range_m)
    return (args.person_width / size_h) * (args.person_height / size_v)


def looks_person(c: dict, grid: Grid, args) -> bool:
    """
    Кандидат подходит под человека НА СВОЕЙ ДАЛЬНОСТИ.

    Пусть R - медианная дальность кандидата, s_h = R*d_az, s_v = R*d_el -
    линейный размер ячейки (один шаг квантования измерения). Условия:
      1) n_cells >= max(support_min, cells_frac * n_exp(R))
      2) h_min - s_v <= dz <= h_max + s_v          (высота с допуском в ячейку)
      3) max(dx, dy) <= w_max + s_h                 (ширина с допуском в ячейку)
    support_min - абсолютный пол (по умолчанию 4, подобран по единственной
    записи с реальным человеком - там минимум был 17 ячеек с большим запасом),
    а не жёсткая привязка к самой малой дальности - на 200-300 м у настоящего
    человека n_exp(R) само может быть 2-4, и слишком высокий фиксированный пол
    отсеял бы его раньше, чем сработает эта проверка. Пока нет записи с
    реальной целью на 200-300 м, это компромисс, а не измеренная граница.
    Допуск по высоте/ширине нужен потому, что размер по точкам занижен на
    величину до одной ячейки: на 150 м s_v ~ 0.44 м, и человек 1.7 м даёт
    dz ~ 1.3 м. Габаритный фильтр грубый: столб или ящик тех же размеров тоже
    пройдёт.
    """
    range_m = c["range"]
    size_h, size_v = grid.cell_size(range_m)
    c["n_exp"] = expected_cells(range_m, grid, args)
    dx, dy, dz = c["size"]
    return (
        c["n_cells"] >= max(args.support_min, args.cells_frac * c["n_exp"])
        and args.person_min_height - size_v <= dz <= args.person_max_height + size_v
        and max(dx, dy) <= args.person_max_width + size_h
    )


def confirm(persons: list[dict], t: float, history: deque, args) -> list[dict]:
    """
    Оставить кандидатов, устойчивых во времени.

    Случайные куски тоннеля редко повторяются на одном месте в нескольких
    кадрах подряд. Кандидат в момент t подтверждён, если в предыдущих
    проанализированных кадрах окна найдено достаточно кандидатов рядом:
        ||center_now - center_prev|| <= r0 + v * |t - t_prev|,
    r0 = --track-radius (ошибка центра), v = --person-speed (макс. скорость
    человека, м/с). Кадров-совпадений (включая текущий) нужно не меньше
    --track-min-hits. Предполагается неподвижный лидар: при движении
    неподвижные объекты смещаются в системе лидара, и радиус надо увеличивать.
    """
    confirmed = []
    for c in persons:
        hits = 1
        for t_prev, centers in history:
            if len(centers) == 0:
                continue
            radius = args.track_radius + args.person_speed * abs(t - t_prev)
            if np.min(np.linalg.norm(centers - c["center"], axis=1)) <= radius:
                hits += 1
        c["hits"] = hits
        if hits >= args.track_min_hits:
            confirmed.append(c)
    history.append(
        (t, np.array([c["center"] for c in persons]) if persons else np.empty((0, 3)))
    )
    return confirmed


def plot_series(times, min_all, min_front, path: Path, counts=None) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = 1 if not counts else 2
    fig, axes = plt.subplots(
        rows, 1, figsize=(11, 4 * rows), sharex=True, squeeze=False
    )
    ax = axes[0][0]
    ax.plot(times, min_all, label="min range (all valid points)", alpha=0.6)
    ax.plot(times, min_front, label="min range (front cone)")
    ax.set_ylabel("distance, m")
    ax.grid(alpha=0.3)
    ax.legend()
    if counts:
        bottom = axes[1][0]
        for label, values in counts.items():
            bottom.step(times, values, where="post", label=label)
        bottom.set_ylabel("objects per frame")
        bottom.grid(alpha=0.3)
        bottom.legend()
    axes[-1][0].set_xlabel("time from start, s")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"Plot saved to {path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bag", type=Path, help="Path to a rosbag2 directory")
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument(
        "--every",
        type=int,
        default=1,
        help="Analyze every Nth frame (skipped frames are not read from disk)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Stop after this many analyzed frames (0 = all)",
    )
    parser.add_argument(
        "--min-range",
        type=float,
        default=0.5,
        help="Drop points closer than this, m (also removes 0,0,0 returns)",
    )
    parser.add_argument(
        "--front-center-deg",
        type=float,
        default=-90.0,
        help="Azimuth of the cone center, deg (default: -Y axis)",
    )
    parser.add_argument(
        "--front-deg", type=float, default=20.0, help="Cone half-angle, deg"
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Print every analyzed frame"
    )
    parser.add_argument(
        "--plot",
        type=Path,
        metavar="PNG",
        help="Save a min-range-over-time plot to this file",
    )

    d = parser.add_argument_group("--clusters: grid, baseline and foreground")
    d.add_argument(
        "--clusters",
        action="store_true",
        help="Detect person-like objects against an empty-tunnel baseline",
    )
    d.add_argument(
        "--az-res-deg", type=float, default=0.1, help="Azimuth cell width, deg"
    )
    d.add_argument(
        "--ring-step-deg",
        type=float,
        default=None,
        help="Vertical cell height, deg (default: measured from the data)",
    )
    d.add_argument(
        "--corridor-center-deg",
        type=float,
        default=-90.0,
        help="Track direction (azimuth), deg",
    )
    d.add_argument(
        "--corridor-half-deg",
        type=float,
        default=3.0,
        help="Corridor half-width as an azimuth angle, deg (scales with range)",
    )
    d.add_argument(
        "--max-lateral",
        type=float,
        default=4.0,
        help="Keep only points with |x| <= this, m (track axis along Y)",
    )
    d.add_argument(
        "--baseline-frames", type=int, default=5, help="Frames used for the baseline"
    )
    d.add_argument(
        "--baseline-lag",
        type=float,
        default=0.5,
        help="Rolling baseline window ends this many seconds before the frame",
    )
    d.add_argument(
        "--baseline-span",
        type=float,
        default=0.5,
        help="Baseline window length, seconds",
    )
    d.add_argument(
        "--baseline-static-at",
        type=float,
        default=None,
        metavar="SEC",
        help="Use one fixed baseline built at SEC seconds from the start "
        "(person-free moment; for a stationary lidar)",
    )
    d.add_argument(
        "--margin",
        type=float,
        default=0.5,
        help="Foreground = nearer than baseline by more than margin + "
        "margin-rel*baseline, m",
    )
    d.add_argument("--margin-rel", type=float, default=0.02)
    d.add_argument(
        "--empty-frac",
        type=float,
        default=0.2,
        help="A baseline cell with returns in fewer than this fraction of "
        "frames counts as empty; any return there is foreground",
    )
    d.add_argument(
        "--eps-k",
        type=float,
        default=2.5,
        help="Neighbor radius = eps_k * local cell size (clipped to eps-min/eps-max), "
        "so grouping scales continuously with range",
    )
    d.add_argument(
        "--eps-min", type=float, default=0.15, help="Neighbor radius floor, m"
    )
    d.add_argument("--eps-max", type=float, default=0.6, help="Neighbor radius cap, m")
    d.add_argument(
        "--cell-depth-tol",
        type=float,
        default=0.5,
        help="Keep points within this distance behind the cell's nearest return, m",
    )

    p = parser.add_argument_group("--clusters: person model and tracking")
    p.add_argument(
        "--support-min",
        type=int,
        default=4,
        help="Minimum cells per candidate at any range (a single cell is never a cluster)",
    )
    p.add_argument(
        "--cells-frac",
        type=float,
        default=0.3,
        help="Required fraction of the cells a person should occupy at that range",
    )
    p.add_argument("--person-width", type=float, default=0.5, help="Nominal width, m")
    p.add_argument("--person-height", type=float, default=1.7, help="Nominal height, m")
    p.add_argument("--person-min-height", type=float, default=1.0)
    p.add_argument("--person-max-height", type=float, default=2.2)
    p.add_argument("--person-max-width", type=float, default=1.2)
    p.add_argument(
        "--track-window",
        type=int,
        default=5,
        help="Analyzed frames considered for persistence (including current)",
    )
    p.add_argument(
        "--track-min-hits",
        type=int,
        default=4,
        help="Frames in the window that must contain the candidate",
    )
    p.add_argument(
        "--track-radius", type=float, default=0.5, help="Center tolerance, m"
    )
    p.add_argument("--person-speed", type=float, default=2.0, help="Max speed, m/s")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.every < 1:
        parser.error("--every must be >= 1")
    if args.baseline_frames < 1:
        parser.error("--baseline-frames must be >= 1")
    if args.track_window < args.track_min_hits:
        parser.error("--track-window must be >= --track-min-hits")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    times, min_all, min_front = [], [], []
    candidate_counts, confirmed_counts = [], []
    history: deque = deque(maxlen=max(args.track_window - 1, 1))
    flagged, max_range, skipped = [], 0.0, 0
    with AnyReader([args.bag], default_typestore=typestore) as reader:
        print("Topics in bag:")
        for topic, info in reader.topics.items():
            print(f"  {topic:30s} {info.msgtype:35s} {info.msgcount} msgs")

        connections = [c for c in reader.connections if c.topic == args.topic]
        if not connections:
            raise SystemExit(f"Topic {args.topic!r} not found in bag")

        if args.clusters:
            stamps = sampled_timestamps(args.bag, args.topic, 1)
            if stamps is None:
                raise SystemExit("--clusters needs an sqlite3 (.db3) bag")
            stamps = np.array(stamps)
            reference = read_points(reader, connections, int(stamps[len(stamps) // 2]))
            grid = Grid.from_points(
                reference, args.min_range, args.az_res_deg, args.ring_step_deg
            )
            col_mask = grid.corridor_columns(
                args.corridor_center_deg, args.corridor_half_deg
            )
            source = FrameSource(reader, connections, grid, args.min_range)
            fixed = (
                static_baseline(source, stamps, args)
                if args.baseline_static_at is not None
                else None
            )
            print(
                f"Grid: {grid.n_rows} rows, vertical step {grid.d_el_deg:.3f} deg, "
                f"azimuth cell {grid.az_res_deg} deg, corridor "
                f"{args.corridor_center_deg:g} +- {args.corridor_half_deg:g} deg "
                f"({int(col_mask.sum())} columns)"
            )
            print(
                "A "
                f"{args.person_width:g}x{args.person_height:g} m person spans about "
                + ", ".join(
                    f"{expected_cells(R, grid, args):.0f} cells at {R} m"
                    for R in (20, 50, 100, 150, 200)
                )
            )

        start_ns = reader.start_time
        for index, connection, timestamp, rawdata in iter_frames(
            reader, connections, args.bag, args.topic, args.every
        ):
            points = decode_points(reader.deserialize(rawdata, connection.msgtype))
            stats = frame_stats(
                points, args.min_range, args.front_center_deg, args.front_deg
            )
            times.append((timestamp - start_ns) / 1e9)
            min_all.append(stats["min_range"])
            min_front.append(stats["min_range_front"])

            if args.clusters:
                image = source.image(timestamp, points)
                baseline = fixed or rolling_baseline(source, stamps, timestamp, args)
                if baseline is None:
                    skipped += 1
                    candidate_counts.append(np.nan)
                    confirmed_counts.append(np.nan)
                    continue
                base, frac = baseline
                mask = foreground_cells(image, base, frac, col_mask, args)
                candidates = extract_candidates(points, image, mask, grid, args)
                persons = [c for c in candidates if looks_person(c, grid, args)]
                confirmed = confirm(persons, times[-1], history, args)
                candidate_counts.append(len(persons))
                confirmed_counts.append(len(confirmed))
                if confirmed:
                    flagged.append(times[-1])
                    max_range = max(max_range, max(c["range"] for c in confirmed))
                if args.verbose or confirmed:
                    print(
                        f"frame {index:5d}  t={times[-1]:8.2f}s  "
                        f"foreground_cells={int(mask.sum()):6d}  "
                        f"candidates={len(candidates)}  person-like={len(persons)}  "
                        f"confirmed={len(confirmed)}"
                    )
                    for c in confirmed if not args.verbose else persons:
                        cx, cy, cz = c["center"]
                        dx, dy, dz = c["size"]
                        tag = "CONFIRMED" if any(c is k for k in confirmed) else ""
                        print(
                            f"    R={c['range']:6.1f}m  cells={c['n_cells']:4d}"
                            f" (expect ~{c['n_exp']:.0f})  pts={c['n_pts']:4d}"
                            f"  center=({cx:6.1f},{cy:6.1f},{cz:5.1f})"
                            f"  size=({dx:4.1f},{dy:4.1f},{dz:4.1f})"
                            f"  hits={c['hits']}  {tag}"
                        )

            if args.verbose and not args.clusters:
                print(
                    f"frame {index:5d}  t={times[-1]:8.2f}s  "
                    f"valid={stats['valid']:6d}/{stats['total']:6d}  "
                    f"min={stats['min_range']:6.2f}m  "
                    f"min_front={stats['min_range_front']:6.2f}m"
                )
            if args.limit and len(times) >= args.limit:
                break

    front = np.array(min_front)
    finite = front[np.isfinite(front)]
    print(
        f"\nAnalyzed {len(times)} frames, "
        f"{np.isfinite(front).sum()} with points in the front cone"
    )
    if finite.size:
        p1, p50, p99 = np.percentile(finite, [1, 50, 99])
        print(
            f"min_front: overall min {finite.min():.2f} m, "
            f"p1 {p1:.2f}, median {p50:.2f}, p99 {p99:.2f}"
        )

    if args.clusters:
        analyzed = len(times) - skipped
        print(
            f"Frames without enough history for a baseline (skipped): {skipped}\n"
            f"Confirmed person-like objects in {len(flagged)} of {analyzed} analyzed frames"
            + (f", farthest at {max_range:.0f} m" if flagged else "")
            + (
                f" (t = {', '.join(f'{t:.0f}' for t in flagged[:15])} s)"
                if flagged
                else ""
            )
        )

    if args.plot:
        counts = (
            {"person-like": candidate_counts, "confirmed": confirmed_counts}
            if args.clusters
            else None
        )
        plot_series(np.array(times), np.array(min_all), front, args.plot, counts)


if __name__ == "__main__":
    main()
