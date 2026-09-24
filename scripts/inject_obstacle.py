#!/usr/bin/env python3
"""
Insert a synthetic obstacle into a negative bag and report recall against distance.

The obstacle is an axis-aligned box in the assumed target frame of forward_sector_assumed.yaml
(x forward = -sensor y, y left = sensor x, z up, origin at the lidar). Returns whose ray enters
the box before their own range are moved onto the box surface, so occlusion is consistent and
the organised cloud layout and all other fields stay unchanged. The train trajectory and floor
height come from a reference evaluate_bag JSONL of the same bag (lidar odometry, ground plane).

Scenarios:
  static    person 0.5 x 0.5 x 1.7 m standing on the route, fixed in the world
  box       object 1.0 x 1.0 x 0.5 m lying on the route, fixed in the world
  crossing  person walking across the route at 1 m/s, fixed longitudinal position

  inject_obstacle.py inject BAG TOPIC REFERENCE.jsonl OUTPUT_ROOT --scenario static
  inject_obstacle.py dataset OUTPUT_ROOT     # dataset.yaml for evaluate_all.py --dataset
  inject_obstacle.py report RUN_DIR OUTPUT_ROOT --output report.json
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

SCENARIOS = {
    'static': {'size': (0.5, 0.5, 1.7), 'lateral': 0.0, 'crossing': False},
    'box': {'size': (1.0, 1.0, 0.5), 'lateral': 0.3, 'crossing': False},
    'crossing': {'size': (0.5, 0.5, 1.7), 'lateral': 0.0, 'crossing': True},
}
CROSSING_SPEED_MPS = 1.0
CROSSING_HALF_WIDTH_M = 1.5
ROI_PAD_M = 0.1
MIN_VISIBLE_POINTS = 10
MAX_EVENT_DISTANCE_M = 200.0
DISTANCE_BINS_M = (0, 20, 40, 60, 80, 100, 120, 150, 200)


def to_target(points):
    """Sensor coordinates to the assumed target frame (yaw +90 degrees)."""
    return np.stack([-points[:, 1], points[:, 0], points[:, 2]], axis=1)


def to_sensor(points):
    """Inverse of to_target."""
    return np.stack([points[:, 1], -points[:, 0], points[:, 2]], axis=1)


def ray_box_entry(directions, lo, hi):
    """Distance along unit rays from the origin to an axis-aligned box; inf on a miss."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    with np.errstate(divide='ignore', invalid='ignore'):
        near = lo / directions
        far = hi / directions
    t_min = np.minimum(near, far)
    t_max = np.maximum(near, far)
    parallel = directions == 0
    inside_slab = (lo <= 0) & (hi >= 0)
    t_min = np.where(parallel, np.where(inside_slab, -np.inf, np.inf), t_min)
    t_max = np.where(parallel, np.where(inside_slab, np.inf, -np.inf), t_max)
    enter = t_min.max(axis=1)
    leave = t_max.min(axis=1)
    return np.where((enter <= leave) & (enter > 0), enter, np.inf)


def occlude(points, lo, hi):
    """Move returns hidden behind the box onto its surface; return points and their count."""
    ranges = np.linalg.norm(points, axis=1)
    valid = np.isfinite(ranges) & (ranges > 0)
    out = points.copy()
    if not valid.any():
        return out, 0
    directions = points[valid] / ranges[valid, None]
    entry = ray_box_entry(directions, lo, hi)
    hidden = entry < ranges[valid] - 1e-6
    index = np.flatnonzero(valid)[hidden]
    out[index] = directions[hidden] * entry[hidden, None]
    return out, int(hidden.sum())


def odometry(rows):
    """Cumulative forward travel per reference frame from the lidar ego-speed estimate."""
    travel = [0.0]
    for previous, current in zip(rows, rows[1:]):
        dt = (current['bag_stamp_ns'] - previous['bag_stamp_ns']) / 1e9
        travel.append(travel[-1] + max(current.get('ego_speed_mps') or 0.0, 0.0) * dt)
    return np.array(travel)


def floor_height(plane, x, y):
    """Height of the normalised plane [nx, ny, nz, d] at (x, y)."""
    return -(plane[0] * x + plane[1] * y + plane[3]) / plane[2]


def choose_window(stamps, travel, window_s):
    """Pick the window of window_s seconds with the largest travel; return (first, last)."""
    best, first = (0, len(stamps) - 1), 0
    best_travel = -1.0
    for last in range(len(stamps)):
        while (stamps[last] - stamps[first]) / 1e9 > window_s:
            first += 1
        if travel[last] - travel[first] > best_travel:
            best_travel = travel[last] - travel[first]
            best = (first, last)
    return best


def box_at(scenario, x_center, time_s, floor_z, centre=0.0):
    """Return (lo, hi) of the obstacle box in the target frame, around the route centre."""
    spec = SCENARIOS[scenario]
    sx, sy, sz = spec['size']
    lateral = spec['lateral']
    if spec['crossing']:
        period = 4 * CROSSING_HALF_WIDTH_M / CROSSING_SPEED_MPS
        phase = (time_s % period) / period
        lateral = CROSSING_HALF_WIDTH_M * (4 * abs(phase - 0.5) - 1)
    lateral += centre
    lo = np.array([x_center - sx / 2, lateral - sy / 2, floor_z])
    hi = np.array([x_center + sx / 2, lateral + sy / 2, floor_z + sz])
    return lo, hi


def estimate_route(points, plane):
    """
    Route centre (c1, c2) of y = c1 x + c2 x^2 from the tunnel walls; None when unsupported.

    Independent of the detector: per 2 m slice, the nearest returns at least 1 m left and
    right of the predicted centre, 0.5-2.5 m above the floor, are walls; their shift from
    the near-range offsets is the centre shift; slices are gated around the running fit.
    """
    x, y = points[:, 0], points[:, 1]
    height = points[:, 2] - floor_height(plane, x, y)
    keep = (x >= 3) & (x < 121) & (np.abs(y) < 12) & (height > 0.5) & (height < 2.5)
    x, y = x[keep], y[keep]
    edges = np.arange(3.0, 121.0, 2.0)
    slices = [y[(x >= a) & (x < a + 2.0)] for a in edges]

    def walls(values, centre):
        left = values[(values >= centre + 1.0) & (values <= centre + 6.0)]
        right = values[(values <= centre - 1.0) & (values >= centre - 6.0)]
        return (left.min() if left.size else np.nan, right.max() if right.size else np.nan)

    near = np.array([walls(values, 0.0) for values in slices[:3]])
    offsets = [np.median(side[np.isfinite(side)]) if np.isfinite(side).sum() >= 2 else np.nan
               for side in near.T]
    xs, shifts, coef = [], [], np.zeros(2)
    for a, values in zip(edges, slices):
        centre_x = a + 1.0
        predicted = coef[0] * centre_x + coef[1] * centre_x ** 2
        found = [wall - offset for wall, offset in zip(walls(values, predicted), offsets)
                 if np.isfinite(wall) and np.isfinite(offset)
                 and abs(wall - offset - predicted) <= 0.3 + 0.01 * centre_x]
        if not found:
            continue
        xs.append(centre_x)
        shifts.append(float(np.mean(found)))
        if len(xs) >= 4:
            design = np.stack([xs, np.square(xs)], axis=1)
            coef = np.linalg.lstsq(design, np.array(shifts), rcond=None)[0]
    if len(xs) < 6 or xs[-1] < 20:
        return None
    return coef


def plan_obstacle(rows, scenario, window_s, final_distance_m):
    """Box per reference frame for the chosen window; the obstacle ends final_distance_m ahead."""
    stamps = [row['bag_stamp_ns'] for row in rows]
    travel = odometry(rows)
    first, last = choose_window(stamps, travel, window_s)
    planes = [row['ground_plane'] for row in rows if row.get('ground_plane')]
    fallback = np.median(np.array(planes), axis=0) if planes else np.array([0, 0, 1, 1.9])
    plan = {}
    for i in range(first, last + 1):
        x_center = final_distance_m + (travel[last] - travel[i])
        plane = np.asarray(rows[i].get('ground_plane') or fallback, float)
        plan[stamps[i]] = (x_center, (stamps[i] - stamps[first]) / 1e9, plane)
    return plan, travel[last] - travel[first]


def estimate_floor(points):
    """
    Floor z = f0 + f1 x as a normalised plane from the lowest returns ahead; None if too few.

    Independent of the detector: per 2 m slice within |y| < 1.2 m, the 10th percentile of z
    is a floor sample (the bed; rails and obstacles stand above it); a line is fitted and
    refitted without samples more than 0.2 m off.
    """
    band = points[(np.abs(points[:, 1]) < 1.2) & (points[:, 0] >= 3) & (points[:, 0] < 60)]
    xs, zs = [], []
    for start in np.arange(3.0, 60.0, 2.0):
        z = band[(band[:, 0] >= start) & (band[:, 0] < start + 2.0), 2]
        if z.size >= 20:
            xs.append(start + 1.0)
            zs.append(np.percentile(z, 10))
    if len(xs) < 5:
        return None
    xs, zs = np.array(xs), np.array(zs)
    keep = np.ones(len(xs), bool)
    for _ in range(3):
        f1, f0 = np.polyfit(xs[keep], zs[keep], 1)
        keep = np.abs(zs - (f0 + f1 * xs)) <= 0.2
        if keep.sum() < 5:
            return None
    return np.array([-f1, 0.0, 1.0, -f0])


def place(scenario, x_center, time_s, plane, points):
    """Box on the floor and route centre estimated from this frame; reference plane fallback."""
    floor = estimate_floor(points)
    plane = plane if floor is None else floor
    route = estimate_route(points, plane)
    centre = 0.0 if route is None else route[0] * x_center + route[1] * x_center ** 2
    floor_z = floor_height(plane, x_center, centre + SCENARIOS[scenario]['lateral'])
    lo, hi = box_at(scenario, x_center, time_s, floor_z, centre)
    return lo, hi, route


def xyz_views(msg, buffer):
    """Writable float32 views of x, y, z over an organised or unorganised PointCloud2 buffer."""
    offsets = {field.name: field.offset for field in msg.fields}
    views = []
    for name in 'xyz':
        views.append(np.ndarray(shape=(msg.height, msg.width), dtype='>f4' if msg.is_bigendian
                                else '<f4', buffer=buffer, offset=offsets[name],
                                strides=(msg.row_step, msg.point_step)))
    return views


def inject(args):
    """Write a shortened copy of BAG with the obstacle and its annotation."""
    from rclpy.serialization import deserialize_message, serialize_message
    import rosbag2_py
    from sensor_msgs.msg import PointCloud2

    rows = [json.loads(line) for line in args.reference.read_text().splitlines() if line.strip()]
    plan, travel = plan_obstacle(rows, args.scenario, args.window_s, args.final_distance_m)
    name = f'{args.bag.name}-{args.scenario}'
    out_dir = args.output_root / name
    out_dir.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out_dir / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=args.topic, type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'))
    log = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if stamp not in plan:
            continue
        msg = deserialize_message(data, PointCloud2)
        buffer = bytearray(msg.data)
        x, y, z = xyz_views(msg, buffer)
        sensor = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1).astype(float)
        target = to_target(sensor)
        lo, hi, route = place(args.scenario, *plan[stamp], target)
        moved, hidden = occlude(target, lo, hi)
        back = to_sensor(moved)
        x[...] = back[:, 0].reshape(x.shape)
        y[...] = back[:, 1].reshape(y.shape)
        z[...] = back[:, 2].reshape(z.shape)
        msg.data = bytes(buffer)
        writer.write(topic, serialize_message(msg), stamp)
        log.append({'bag_stamp_ns': stamp, 'lo': lo.tolist(), 'hi': hi.tolist(),
                    'distance_m': float(lo[0]), 'hidden_points': hidden,
                    'route': None if route is None else [float(v) for v in route]})
    del writer
    (out_dir / 'injection.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in log))
    visible = [row for row in log if row['hidden_points'] >= MIN_VISIBLE_POINTS
               and row['distance_m'] <= MAX_EVENT_DISTANCE_M]
    annotation = {
        'schema_version': 1, 'bag_id': name, 'reviewed': True, 'time_basis': 'bag_stamp_ns',
        'review_method': 'synthetic_raycast_injection',
        'review_limits': (f'Synthetic {args.scenario} obstacle in {args.bag.name}; positive from '
                          f'the first frame with at least {MIN_VISIBLE_POINTS} occluded returns '
                          f'within {MAX_EVENT_DISTANCE_M:.0f} m. Trajectory from lidar odometry.'),
        'reviewed_intervals': [], 'events': [],
    }
    if visible:
        start = visible[0]['bag_stamp_ns']
        end = log[-1]['bag_stamp_ns']
        if start > log[0]['bag_stamp_ns']:
            annotation['reviewed_intervals'].append(
                {'start_ns': log[0]['bag_stamp_ns'], 'end_ns': start, 'label': 'negative',
                 'evidence': 'Synthetic obstacle not yet visible.'})
        annotation['reviewed_intervals'].append(
            {'start_ns': start, 'end_ns': end, 'label': 'positive',
             'evidence': 'Synthetic obstacle visible on the route.'})
        references = [{
            'bag_stamp_ns': row['bag_stamp_ns'], 'distance_m': row['distance_m'],
            'uncertainty_m': 0.05, 'method': 'synthetic_box_near_face',
            'person_roi_assumed_m': {'min': [v - ROI_PAD_M for v in row['lo']],
                                     'max': [v + ROI_PAD_M for v in row['hi']]},
        } for row in log if row['bag_stamp_ns'] >= start]
        annotation['events'].append({
            'id': f'synthetic_{args.scenario}', 'start_ns': start, 'end_ns': end,
            'label': 'positive', 'evidence': 'Raycast synthetic obstacle.',
            'reference_frames': references})
    (out_dir / 'annotations.yaml').write_text(yaml.safe_dump(annotation, sort_keys=False))
    (out_dir / 'topic').write_text(args.topic + '\n')
    print(f'{name}: {len(log)} frames, travel {travel:.0f} m, visible {len(visible)} frames')


def dataset(args):
    """Write OUTPUT_ROOT/dataset.yaml for evaluate_all.py --dataset; safe after parallel runs."""
    bags = []
    injected = (p for p in args.output_root.iterdir() if (p / 'annotations.yaml').is_file())
    for bag_dir in sorted(injected):
        frames = sum(1 for line in (bag_dir / 'injection.jsonl').read_text().splitlines() if line)
        bags.append({
            'id': bag_dir.name, 'path': f'{bag_dir.name}/bag',
            'input_topic': ((bag_dir / 'topic').read_text().strip()
                            if (bag_dir / 'topic').is_file() else '/lidar_points'),
            'storage_id': 'sqlite3',
            'declared_message_count': frames,
            'annotations': str((bag_dir / 'annotations.yaml').resolve()),
            'sensor_profile': 'forward_sector'})
    text = yaml.safe_dump({'schema_version': 1, 'data_root': str(args.output_root.resolve()),
                           'bags': bags}, sort_keys=False)
    (args.output_root / 'dataset.yaml').write_text(text)
    print(f'{len(bags)} bags in {args.output_root / "dataset.yaml"}')


def report(args):
    """Object-level recall per distance bin for every injected bag of a run."""
    result = {}
    injected = (p for p in args.output_root.iterdir() if (p / 'injection.jsonl').is_file())
    for bag_dir in sorted(injected):
        frames = args.run_dir / bag_dir.name / 'frames.jsonl'
        if not frames.is_file():
            continue
        rows = {json.loads(line)['bag_stamp_ns']: json.loads(line)
                for line in frames.read_text().splitlines() if line.strip()}
        bins = {f'{a}-{b}': [0, 0] for a, b in zip(DISTANCE_BINS_M, DISTANCE_BINS_M[1:])}
        first_hit = None
        for entry in map(json.loads, (bag_dir / 'injection.jsonl').read_text().splitlines()):
            if entry['hidden_points'] < MIN_VISIBLE_POINTS:
                continue
            row = rows.get(entry['bag_stamp_ns'])
            distance = entry['distance_m']
            key = next((f'{a}-{b}' for a, b in zip(DISTANCE_BINS_M, DISTANCE_BINS_M[1:])
                        if a <= distance < b), None)
            if row is None or key is None:
                continue
            lo = np.array(entry['lo']) - 0.5
            hi = np.array(entry['hi']) + 0.5
            # Confirmed tracks decide since G4; older runs only have candidates.
            objects = ([t for t in row['tracks'] if t['confirmed']] if 'tracks' in row
                       else row['candidates'])
            hit = row['state'] == 'OBSTACLE' and any(
                np.all((np.array(c['center']) >= lo) & (np.array(c['center']) <= hi))
                for c in objects)
            bins[key][0] += hit
            bins[key][1] += 1
            if hit and first_hit is None:
                first_hit = distance
        result[bag_dir.name] = {
            'first_hit_distance_m': first_hit,
            'recall_by_distance': {k: (None if n == 0 else round(h / n, 3), n)
                                   for k, (h, n) in bins.items()}}
    text = json.dumps(result, indent=2)
    args.output.write_text(text + '\n')
    print(text)


def main():
    """Parse arguments and run a subcommand."""
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    make = sub.add_parser('inject')
    make.add_argument('bag', type=Path)
    make.add_argument('topic')
    make.add_argument('reference', type=Path, help='evaluate_bag JSONL of the same bag')
    make.add_argument('output_root', type=Path)
    make.add_argument('--scenario', choices=sorted(SCENARIOS), required=True)
    make.add_argument('--window-s', type=float, default=20.0)
    make.add_argument('--final-distance-m', type=float, default=8.0)
    listing = sub.add_parser('dataset')
    listing.add_argument('output_root', type=Path)
    score = sub.add_parser('report')
    score.add_argument('run_dir', type=Path)
    score.add_argument('output_root', type=Path)
    score.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    {'inject': inject, 'dataset': dataset, 'report': report}[args.command](args)


if __name__ == '__main__':
    main()
