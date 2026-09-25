#!/usr/bin/env python3
"""
Annotate a bag with inserted objects from the returns the insertion appended to each cloud.

In the organizer bag cloud_with_fake_obj every cloud is the recorded organised cloud (2400
azimuths x 128 channels) without the returns an object hides, followed by the object returns:
a trailing block of returns with intensity exactly 1.0. About 80 % of them also lie off the
lidar channel elevations; off-channel returns before the block are counted as a check of the
block boundary. Coordinates are the assumed target frame of forward_sector_assumed.yaml
(x forward = -sensor y, y left = sensor x, z up, origin at the lidar).

  annotate_inserted_objects.py extract BAG TOPIC FRAMES.jsonl      # needs ROS 2 (rosbag2_py)
  annotate_inserted_objects.py annotate FRAMES.jsonl SCENE_CONTEXT.yaml BAG_ID ANNOTATION.yaml

extract writes one row per cloud with the inserted returns clustered into objects. annotate
follows every object of the organizer scene table of BAG_ID (bags.BAG_ID.objects in
evaluation/scene_context.yaml) back from its closest approach and labels the frames; the rules
are written into the annotation (review_limits).
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

MIN_RANGE_M = 0.3  # Pandar128 minimum range: closer returns are not measurements.
CHANNEL_TOLERANCE_DEG = 0.02  # The finest Pandar128 channel spacing is 0.086 deg.
CHANNEL_SHARE = 0.1
CLUSTER_GAP_X_M = 3.0
CLUSTER_GAP_Y_M = 1.5
# Same visibility rule as the synthetic benchmark (inject_obstacle.py).
MIN_RETURNS = 10
HORIZON_M = 200.0
NEAR_M = 1.0  # Closer than this the object is at the lidar and has no forward face left.
SEED_BEFORE_S, SEED_AFTER_S, SEED_MAX_X_M = 1.0, 0.6, 25.0
DEFAULT_SPEED_MPS = 10.0
BACK_MAX_MISSES, FORWARD_MAX_MISSES = 15, 5

# Frame kinds, the highest priority wins. Before an inside object is observable it counts as
# absent, as in the synthetic benchmark; only its passage and unknown observable returns are
# left unscored.
PRIORITY = {'positive': 0, 'passing': 1, 'unexplained': 2, 'outside': 3, 'unobservable': 4,
            'far': 5, 'empty': 6}
LABEL = {'positive': 'positive', 'passing': 'uncertain', 'unexplained': 'uncertain',
         'outside': 'negative', 'unobservable': 'negative', 'far': 'negative',
         'empty': 'negative'}


def to_target(points):
    """Sensor coordinates to the assumed target frame (yaw +90 degrees)."""
    return np.stack([-points[:, 1], points[:, 0], points[:, 2]], axis=1)


def block_start(intensity):
    """Index where the trailing run of intensity 1.0 returns begins (len if there is none)."""
    other = np.flatnonzero(intensity != 1.0)
    return int(other[-1]) + 1 if len(other) else 0


def elevation_deg(points):
    return np.degrees(np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1])))


def channel_elevations(elevation, tolerance=CHANNEL_TOLERANCE_DEG, share=CHANNEL_SHARE):
    """Channel elevations of recorded returns: sharp elevation values holding many returns."""
    values, counts = np.unique(np.round(elevation, 3), return_counts=True)
    if not len(values):
        return values
    group = np.cumsum(np.r_[True, np.diff(values) > tolerance]) - 1
    group_counts = np.bincount(group, weights=counts)
    centres = np.bincount(group, weights=values * counts) / group_counts
    return centres[group_counts >= share * group_counts.max()]


def off_channel(elevation, channels, tolerance=CHANNEL_TOLERANCE_DEG):
    """Mark elevations farther than tolerance from every channel."""
    if len(channels) < 2:
        return np.zeros(len(elevation), dtype=bool)
    upper = np.clip(np.searchsorted(channels, elevation), 1, len(channels) - 1)
    gap = np.minimum(np.abs(elevation - channels[upper - 1]), np.abs(elevation - channels[upper]))
    return gap > tolerance


def clusters(points):
    """Split target-frame points at gaps along x, then across y."""
    result = []
    points = points[np.argsort(points[:, 0], kind='stable')]
    for part in np.split(points, np.flatnonzero(np.diff(points[:, 0]) > CLUSTER_GAP_X_M) + 1):
        part = part[np.argsort(part[:, 1], kind='stable')]
        cuts = np.flatnonzero(np.diff(part[:, 1]) > CLUSTER_GAP_Y_M) + 1
        result.extend(sub for sub in np.split(part, cuts) if len(sub))
    return result


def inserted_objects(sensor_points, intensity):
    """Cluster the inserted returns of one cloud and check the block boundary."""
    ranges = np.linalg.norm(sensor_points, axis=1)
    valid = np.isfinite(ranges) & (ranges >= MIN_RANGE_M)
    start = block_start(intensity)
    elevation = elevation_deg(sensor_points)
    recorded = valid.copy()
    recorded[start:] = False
    channels = channel_elevations(elevation[recorded])
    inserted = valid.copy()
    inserted[:start] = False
    objects = [{'n': int(len(c)), 'min': np.round(c.min(axis=0), 3).tolist(),
                'max': np.round(c.max(axis=0), 3).tolist()}
               for c in clusters(to_target(sensor_points[inserted]))]
    return {
        'points': int(valid.sum()), 'block': int(len(intensity) - start),
        'block_returns': int(inserted.sum()), 'channels': int(len(channels)),
        'block_off_channel': int(off_channel(elevation[inserted], channels).sum()),
        'recorded_off_channel': int(off_channel(elevation[recorded], channels).sum()),
        'objects': objects,
    }


def cloud_fields(msg, names):
    """Float32 fields of a PointCloud2 as flat arrays."""
    offsets = {field.name: field.offset for field in msg.fields}
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    dtype = '>f4' if msg.is_bigendian else '<f4'
    return [np.ndarray(shape=(msg.height, msg.width), dtype=dtype, buffer=buffer,
                       offset=offsets[name], strides=(msg.row_step, msg.point_step))
            .reshape(-1).astype(np.float64) for name in names]


def extract(args):
    """Write the inserted objects of every cloud of BAG as JSONL."""
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    count = 0
    with args.output.open('x') as stream:
        while reader.has_next():
            _, data, stamp = reader.read_next()
            msg = deserialize_message(data, PointCloud2)
            x, y, z, intensity = cloud_fields(msg, ('x', 'y', 'z', 'intensity'))
            row = {'bag_stamp_ns': stamp,
                   'header_stamp_ns': msg.header.stamp.sec * 1_000_000_000 +
                   msg.header.stamp.nanosec}
            row.update(inserted_objects(np.stack([x, y, z], axis=1), intensity))
            stream.write(json.dumps(row) + '\n')
            count += 1
    print(f'{count} clouds -> {args.output}')


def centre(obj, axis):
    return (obj['min'][axis] + obj['max'][axis]) / 2


def same_object(obj, last):
    """Lateral and vertical continuity; a sparse object shows different parts per frame."""
    lateral = (abs(centre(obj, 1) - centre(last, 1)) <= 0.8 or
               (obj['min'][1] <= last['max'][1] + 0.5 and obj['max'][1] >= last['min'][1] - 0.5))
    return lateral and obj['min'][2] <= last['max'][2] + 2.0 and \
        obj['max'][2] >= last['min'][2] - 2.0


def initial_speed(frames, start, index):
    """
    Approach speed at a seed: median of the single-frame steps just before it.

    The insertion sometimes keeps an object in place for a frame and then moves it by two
    steps; frames without movement are skipped and the median ignores the double step.
    """
    last = frames[start]['objects'][index]
    speeds = []
    for i in range(start - 1, max(start - 7, -1), -1):
        dt = frames[i + 1]['t'] - frames[i]['t']
        steps = [obj['min'][0] - last['min'][0] for obj in frames[i]['objects']
                 if same_object(obj, last) and 0 <= obj['min'][0] - last['min'][0] <= 4.0]
        if not steps or dt <= 0:
            break
        step = min(steps)
        if step > 0.05:
            speeds.append(step / dt)
        last = next(obj for obj in frames[i]['objects']
                    if same_object(obj, last) and obj['min'][0] - last['min'][0] == step)
    return float(np.median(speeds)) if speeds else DEFAULT_SPEED_MPS


def follow(frames, start, index, direction):
    """Cluster index per frame of one object, from a seed frame forward (+1) or back (-1)."""
    found = {}
    last_frame, last = start, frames[start]['objects'][index]
    speed, misses = initial_speed(frames, start, index), 0
    i = start + direction
    while 0 <= i < len(frames):
        dt = abs(frames[i]['t'] - frames[last_frame]['t'])
        predicted = last['min'][0] - direction * speed * dt
        tolerance = max(2.5, 0.08 * predicted)
        best = None
        for k, obj in enumerate(frames[i]['objects']):
            error = abs(obj['min'][0] - predicted)
            frozen = abs(obj['min'][0] - last['min'][0]) < 0.05  # the object skipped a step
            if error > tolerance and not frozen:
                continue
            if not same_object(obj, last):
                continue
            if best is None or error < best[0]:
                best = (error, k)
        if best is not None:
            obj = frames[i]['objects'][best[1]]
            step = direction * (last['min'][0] - obj['min'][0])
            if step > 0.05 and dt > 0:
                speed = 0.7 * speed + 0.3 * step / dt
            found[i] = best[1]
            last_frame, last, misses = i, obj, 0
        else:
            misses += 1
            if direction > 0 and misses > FORWARD_MAX_MISSES:
                break
            if direction < 0 and (misses > BACK_MAX_MISSES or predicted > HORIZON_M + 20):
                break
        i += direction
    return found


def approach_speed(frames, track, indices):
    """Mean approach speed over the given observed frames of a track."""
    if len(indices) < 2:
        return DEFAULT_SPEED_MPS
    first, last = indices[0], indices[-1]
    dx = track['x'][first] - track['x'][last]
    dt = frames[last]['t'] - frames[first]['t']
    return dx / dt if dt > 0 and dx > 0 else DEFAULT_SPEED_MPS


def follow_object(frames, item):
    """Track of one scene object, seeded at its closest approach in the organizer window."""
    end = item['video_s'][1]
    seed = None
    for i, frame in enumerate(frames):
        if not end - SEED_BEFORE_S <= frame['t'] <= end + SEED_AFTER_S:
            continue
        near = [(obj['min'][0], k) for k, obj in enumerate(frame['objects'])
                if NEAR_M <= obj['min'][0] <= SEED_MAX_X_M]
        if near:
            seed = (i, min(near)[1])
    if seed is None:
        raise ValueError(f"{item['id']}: no inserted object near the lidar at {end} s")
    observed = {seed[0]: seed[1]}
    observed.update(follow(frames, seed[0], seed[1], -1))
    observed.update(follow(frames, seed[0], seed[1], +1))
    track = {'id': item['id'], 'class': item['organizer_class'],
             'description': item['description'], 'observed': dict(sorted(observed.items()))}
    track['x'] = {i: frames[i]['objects'][k]['min'][0] for i, k in track['observed'].items()}
    track['n'] = {i: frames[i]['objects'][k]['n'] for i, k in track['observed'].items()}
    ahead = [i for i in track['observed'] if track['x'][i] >= NEAR_M]
    track['speed_end'] = approach_speed(frames, track, ahead[-5:])
    return track


def label_frames(frames, tracks, horizon=HORIZON_M, min_returns=MIN_RETURNS, near=NEAR_M):
    """
    Kind of every frame (see PRIORITY) with the objects behind it, and the span of each object.

    An inside object is positive from its first frame with min_returns returns within the
    horizon to its last frame at least near ahead, then passing until the train reaches it at
    its last approach speed. Outside and above objects make their frames negative.
    """
    kinds = ['empty'] * len(frames)
    notes = [set() for _ in frames]

    def mark(i, kind, note):
        if PRIORITY[kind] < PRIORITY[kinds[i]]:
            kinds[i], notes[i] = kind, set()
        if kind == kinds[i]:
            notes[i].add(note)

    spans = {}
    for track in tracks:
        within = [i for i in track['observed'] if track['x'][i] <= horizon]
        if not within:
            continue
        if track['class'] != 'inside':
            for i in range(within[0], within[-1] + 1):
                mark(i, 'outside', track['id'])
            spans[track['id']] = (within[0], within[-1])
            continue
        ahead = [i for i in within if track['x'][i] >= near]
        observable = [i for i in ahead if track['n'][i] >= min_returns]
        if not observable:
            raise ValueError(f"{track['id']}: never {min_returns} returns within {horizon} m")
        first, last = observable[0], ahead[-1]
        spans[track['id']] = (first, last)
        for i in range(first, last + 1):
            mark(i, 'positive', track['id'])
        for i in within:
            if i < first:
                mark(i, 'unobservable', track['id'])
            elif i > last:
                mark(i, 'passing', track['id'])
        reach = frames[last]['t'] + track['x'][last] / max(track['speed_end'], 1.0)
        i = last + 1
        while i < len(frames) and frames[i]['t'] <= reach + 1e-6:
            mark(i, 'passing', track['id'])
            i += 1
    owned = {(i, k) for track in tracks for i, k in track['observed'].items()}
    unlinked = []
    for i, frame in enumerate(frames):
        for k, obj in enumerate(frame['objects']):
            if obj['min'][0] > horizon:
                mark(i, 'far', 'far')
            elif (i, k) not in owned:
                unlinked.append((i, obj))
                mark(i, 'unexplained' if obj['n'] >= min_returns else 'unobservable', 'unlinked')
    return kinds, notes, spans, unlinked


def evidence(kinds, notes, tracks, lo, hi, horizon=HORIZON_M, min_returns=MIN_RETURNS,
             near=NEAR_M):
    """Evidence text of one interval from the kinds and notes of its frames."""
    names = {kind: sorted(set().union(*(n for k, n in zip(kinds, notes) if k == kind)))
             for kind in set(kinds)}
    if 'positive' in names:
        parts = []
        for track in tracks:
            if track['id'] in names['positive']:
                xs = [track['x'][i] for i in range(lo, hi + 1) if i in track['x']]
                parts.append(f"{track['id']} {max(xs):.1f}->{min(xs):.1f} m")
        return (f'Inside object observed ahead (at least {min_returns} returns within '
                f'{horizon:g} m): ' + ', '.join(parts) + '.')
    if 'passing' in names or 'unexplained' in names:
        parts = []
        if 'passing' in names:
            parts.append(f'inside object closer than {near:g} m or out of view until the train '
                         'reaches it: ' + ', '.join(names['passing']))
        if 'unexplained' in names:
            parts.append('observable inserted returns not linked to a scene object')
        return 'Not scored: ' + '; '.join(parts) + '.'
    parts = []
    if 'outside' in names:
        parts.append('objects the organizers class outside or above the envelope: ' +
                     ', '.join(names['outside']))
    inside = [name for name in names.get('unobservable', []) if name != 'unlinked']
    if inside:
        parts.append(f'inside objects with fewer than {min_returns} returns within '
                     f'{horizon:g} m: ' + ', '.join(inside))
    if 'unlinked' in names.get('unobservable', []):
        parts.append(f'sparse unlinked inserted returns within {horizon:g} m')
    if 'far' in names:
        parts.append(f'inserted objects beyond {horizon:g} m')
    if 'empty' in names:
        parts.append('no inserted object')
    return 'No observable inside object: ' + '; '.join(parts) + '.'


def intervals(frames, kinds, notes, tracks, horizon=HORIZON_M, min_returns=MIN_RETURNS,
              near=NEAR_M):
    """Disjoint reviewed intervals covering every frame, one per label run."""
    def key(i):
        label = LABEL[kinds[i]]
        return label, None if label == 'negative' else frozenset(notes[i])

    result = []
    i = 0
    while i < len(frames):
        j = i
        while j + 1 < len(frames) and key(j + 1) == key(i):
            j += 1
        result.append({
            'start_ns': frames[i]['bag_stamp_ns'], 'end_ns': frames[j]['bag_stamp_ns'],
            'label': LABEL[kinds[i]],
            'evidence': evidence(kinds[i:j + 1], notes[i:j + 1], tracks, i, j, horizon,
                                 min_returns, near)})
        i = j + 1
    return result


def events(frames, tracks, spans):
    """Annotation events with a reference frame for every observed frame of the span."""
    result = []
    for track in tracks:
        if track['id'] not in spans:
            continue
        first, last = spans[track['id']]
        references = []
        for i in range(first, last + 1):
            if i not in track['observed']:
                continue
            obj = frames[i]['objects'][track['observed'][i]]
            references.append({
                'bag_stamp_ns': frames[i]['bag_stamp_ns'], 'distance_m': round(obj['min'][0], 3),
                'uncertainty_m': None, 'method': 'nearest_inserted_return', 'returns': obj['n'],
                'person_roi_assumed_m': {'min': obj['min'], 'max': obj['max']}})
        xs = [r['distance_m'] for r in references]
        result.append({
            'id': track['id'], 'start_ns': frames[first]['bag_stamp_ns'],
            'end_ns': frames[last]['bag_stamp_ns'],
            'label': 'positive' if track['class'] == 'inside' else 'negative',
            'organizer_class': track['class'],
            'evidence': (f"Organizer: {track['description']}, {track['class']}. Inserted returns "
                         f'from {max(xs):.1f} m to {min(xs):.1f} m in {len(references)} of '
                         f'{last - first + 1} frames.'),
            'reference_frames': references})
    return result


def scene_objects(context, bag_id):
    """Organizer scene table of one bag from scene_context.yaml."""
    objects = ((context.get('bags') or {}).get(bag_id) or {}).get('objects')
    if not objects:
        raise ValueError(f'no scene objects for {bag_id} in the scene context')
    return objects


def limits(frames, bag_id):
    recorded = max(frame['recorded_off_channel'] for frame in frames)
    channels = sorted({frame['channels'] for frame in frames if frame['channels']})
    inserted = sum(frame['block_returns'] for frame in frames)
    off = sum(frame['block_off_channel'] for frame in frames)
    return (
        'Generated by scripts/annotate_inserted_objects.py from the clouds and the organizer '
        f'scene table in evaluation/scene_context.yaml (bags.{bag_id}). Inserted returns: the '
        'trailing block of intensity-1.0 returns that the insertion appends after the recorded '
        f'organised cloud. {off / max(inserted, 1):.0%} of them lie off the lidar channels; the '
        f'recorded part has at most {recorded} off-channel returns per cloud (channels found in '
        f"it: {', '.join(map(str, channels))}). Returns closer than {MIN_RANGE_M} m are not "
        'used. Each scene object is followed back from its closest approach in the organizer '
        'window. distance_m is the nearest x of its returns; the ROI is their exact box (the '
        'metrics add their own matching margin); uncertainty_m is not estimated. Visibility rule '
        'of the synthetic benchmark (inject_obstacle.py): an inside object is positive from the '
        f'first frame with at least {MIN_RETURNS} returns within {HORIZON_M:g} m to its last '
        f'frame at least {NEAR_M:g} m ahead; before that it counts as absent. Uncertain (not '
        f'scored): an inside object closer than {NEAR_M:g} m or out of view until the train '
        'reaches it at its last approach speed. Negative: everything else, including objects the '
        'organizers class outside or above the envelope. Inside, outside and above are the '
        'organizer classes; the envelope itself is not given, and the current detector corridor '
        '(forward_sector_assumed.yaml: +-2 m, 3.5 m above the track) also contains objects 5 and '
        '7 and the lower part of 8, so an alarm on them is false against the classes but '
        'expected from that corridor. The positive frames do not depend on a detector: frames '
        'beyond its range stay positive; metrics.py reports object recall by distance and, apart '
        'from the totals, within the range of the run profile. It also counts alarms on the '
        'outside and above objects on every frame, including frames that another object makes '
        'positive. Axes are the assumed forward-sector frame. The legacy person_roi_assumed_m key '
        'is the generic object ROI read by the metrics; returns is the number of inserted returns '
        'behind the reference.')


class Dumper(yaml.SafeDumper):
    """Compact YAML: one line per reference frame."""


Dumper.add_representer(dict, lambda dumper, data: dumper.represent_mapping(
    'tag:yaml.org,2002:map', data,
    flow_style='bag_stamp_ns' in data or set(data) == {'min', 'max'}))
Dumper.add_representer(list, lambda dumper, data: dumper.represent_sequence(
    'tag:yaml.org,2002:seq', data,
    flow_style=len(data) == 3 and all(isinstance(v, (int, float)) for v in data)))


def annotate(args):
    """Write the annotation of one bag from its extracted inserted objects."""
    frames = [json.loads(line) for line in args.frames.read_text().splitlines() if line.strip()]
    for frame in frames:
        frame['t'] = (frame['header_stamp_ns'] - frames[0]['header_stamp_ns']) / 1e9
    objects = scene_objects(yaml.safe_load(args.scene.read_text()), args.bag_id)
    tracks = [follow_object(frames, item) for item in objects]
    owners = {}
    for track in tracks:
        for key in track['observed'].items():
            if key in owners:
                raise ValueError(f"{track['id']} and {owners[key]} share a cluster at {key}")
            owners[key] = track['id']
    kinds, notes, spans, unexplained = label_frames(frames, tracks)
    annotation = {
        'schema_version': 1, 'bag_id': args.bag_id, 'reviewed': True,
        'time_basis': 'bag_stamp_ns', 'review_method': 'inserted_returns_trailing_block',
        'review_limits': limits(frames, args.bag_id),
        'reviewed_intervals': intervals(frames, kinds, notes, tracks),
        'events': events(frames, tracks, spans)}
    with args.output.open('w') as stream:
        yaml.dump(annotation, stream, Dumper=Dumper, sort_keys=False, width=100)
    counts = {label: sum(LABEL[kind] == label for kind in kinds)
              for label in ('positive', 'uncertain', 'negative')}
    print(f"{args.output}: frames {counts}, intervals {len(annotation['reviewed_intervals'])}")
    for event in annotation['events']:
        refs = event['reference_frames']
        print(f"  {event['id']:24s} {event['label']:8s} refs {len(refs):3d} "
              f"{refs[0]['distance_m']:6.1f} -> {refs[-1]['distance_m']:5.1f} m")
    runs = []
    for i, obj in unexplained:
        if runs and i - runs[-1][1] <= 3:
            runs[-1][1:] = [i, runs[-1][2] + [obj]]
        else:
            runs.append([i, i, [obj]])
    for first, last, objs in runs:
        xs = [obj['min'][0] for obj in objs]
        print(f"  unlinked inserted returns {frames[first]['t']:.1f}-{frames[last]['t']:.1f} s: "
              f"x {max(xs):.1f}->{min(xs):.1f} m, at most {max(o['n'] for o in objs)} returns")


def main():
    """Parse arguments and run a subcommand."""
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    read = sub.add_parser('extract')
    read.add_argument('bag', type=Path)
    read.add_argument('topic')
    read.add_argument('output', type=Path, help='new JSONL file')
    label = sub.add_parser('annotate')
    label.add_argument('frames', type=Path, help='JSONL written by extract')
    label.add_argument('scene', type=Path, help='evaluation/scene_context.yaml')
    label.add_argument('bag_id', help='bag whose scene objects are annotated')
    label.add_argument('output', type=Path)
    args = parser.parse_args()
    {'extract': extract, 'annotate': annotate}[args.command](args)


if __name__ == '__main__':
    main()
