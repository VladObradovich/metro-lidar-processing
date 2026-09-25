"""Inserted-object annotation on hand-made clouds and tracks; no ROS or bag required."""

import importlib.util
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    'annotate_inserted_objects',
    Path(__file__).resolve().parents[1] / 'annotate_inserted_objects.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

SECOND = 1_000_000_000


def rays(elevations_deg, azimuths_deg, distance):
    """Sensor-frame returns on the given elevation x azimuth grid; forward is -sensor y."""
    el, az = np.meshgrid(np.radians(elevations_deg), np.radians(azimuths_deg), indexing='ij')
    horizontal = distance * np.cos(el)
    forward, left = horizontal * np.cos(az), horizontal * np.sin(az)
    return np.stack([left.ravel(), -forward.ravel(), (distance * np.sin(el)).ravel()], axis=1)


CHANNELS = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0]


def recorded_cloud():
    points = rays(CHANNELS, np.linspace(-40, 40, 200), 30.0)
    intensity = np.full(len(points), 5.0)
    intensity[::7] = 1.0  # recorded returns can have intensity 1 too
    intensity[-1] = 3.0
    return points, intensity


def frame(t, *objects):
    return {'t': t, 'bag_stamp_ns': int(round(t * SECOND)), 'objects': list(objects)}


def obj(x, y=0.0, n=20, z=(-1.0, 0.0), width=0.3):
    return {'n': n, 'min': [x, y - width / 2, z[0]], 'max': [x + 0.3, y + width / 2, z[1]]}


def hand_track(frames, name, organizer_class, index, speed=15.0):
    """Track that owns cluster index in every given frame."""
    observed = {i: index for i in range(len(frames))}
    return {'id': name, 'class': organizer_class, 'description': name, 'observed': observed,
            'x': {i: frames[i]['objects'][index]['min'][0] for i in observed},
            'n': {i: frames[i]['objects'][index]['n'] for i in observed}, 'speed_end': speed}


class BlockTest(unittest.TestCase):

    def test_block_is_the_trailing_intensity_one_run(self):
        self.assertEqual(tool.block_start(np.array([1.0, 5.0, 1.0, 1.0])), 2)
        self.assertEqual(tool.block_start(np.array([1.0, 5.0, 2.0])), 3)
        self.assertEqual(tool.block_start(np.array([1.0, 1.0])), 0)

    def test_recorded_cloud_has_no_inserted_objects(self):
        points, intensity = recorded_cloud()
        result = tool.inserted_objects(points, intensity)
        self.assertEqual(result['block'], 0)
        self.assertEqual(result['objects'], [])
        self.assertEqual(result['channels'], len(CHANNELS))
        self.assertEqual(result['recorded_off_channel'], 0)

    def test_dense_near_object_is_extracted_whole(self):
        # A face 2 m ahead on its own vertical grid, denser than every recorded channel; part of
        # its rows coincide with channels, which an elevation test alone would miss.
        points, intensity = recorded_cloud()
        face = rays(np.arange(-20.0, 20.0, 0.25), np.linspace(-20, 20, 300), 2.0)
        cloud = np.vstack([points, face, np.zeros((5, 3))])
        values = np.r_[intensity, np.ones(len(face) + 5)]
        result = tool.inserted_objects(cloud, values)
        self.assertEqual(result['block_returns'], len(face))
        self.assertEqual(result['channels'], len(CHANNELS))
        self.assertLess(result['block_off_channel'], len(face))
        self.assertEqual(len(result['objects']), 1)
        self.assertEqual(result['objects'][0]['n'], len(face))
        corner = 2.0 * np.cos(np.radians(20)) ** 2  # the face corner at -20 deg on both axes
        self.assertAlmostEqual(result['objects'][0]['min'][0], corner, 2)

    def test_objects_split_along_x_then_across_y(self):
        points = np.array([[10, 0, 0], [10.5, 0.2, 0], [20, 0, 0], [20.2, 3.0, 0]], float)
        sizes = sorted(len(c) for c in tool.clusters(points))
        self.assertEqual(sizes, [1, 1, 2])


class TrackTest(unittest.TestCase):

    def approach(self):
        """Close in at 15 m/s with a frozen frame, a 3-frame gap and a far decoy."""
        frames = []
        for step in range(40):
            t = step * 0.1
            x = 60.0 - 1.5 * step
            objects = [obj(200.0 + x, y=-2.0, n=3)]  # a far object on another trajectory
            if step == 20:
                x += 1.5  # the object skipped a step
            if step not in (10, 11, 12):
                objects.append(obj(x, n=int(400 / x)))
            frames.append(frame(t, *objects))
        return frames

    def test_follow_back_from_the_closest_approach(self):
        frames = self.approach()
        track = tool.follow_object(frames, {'id': 'a', 'organizer_class': 'inside',
                                            'description': 'test', 'video_s': [0, 3.9]})
        self.assertEqual(len(track['observed']), 37)
        self.assertNotIn(10, track['observed'])
        self.assertEqual(track['observed'][0], 1)
        self.assertAlmostEqual(track['speed_end'], 15.0, 0)

    def test_labels(self):
        frames = self.approach() + [frame(4.0 + 0.1 * k) for k in range(10)]
        frames += [frame(5.0 + 0.1 * k, obj(20.0 - k, y=-3.0)) for k in range(5)]
        inside = tool.follow_object(frames, {'id': 'a', 'organizer_class': 'inside',
                                             'description': 'test', 'video_s': [0, 3.9]})
        outside = tool.follow_object(frames, {'id': 'b', 'organizer_class': 'outside',
                                              'description': 'side', 'video_s': [5.0, 5.4]})
        kinds, notes, spans, unlinked = tool.label_frames(
            frames, [inside, outside], horizon=100.0, min_returns=10, near=1.0)
        # 400 / x >= 10 from x = 40 m (step 14); the object passes out of view at 1.5 m.
        first, last = spans['a']
        self.assertEqual(first, 14)
        self.assertEqual(last, 39)
        self.assertTrue(all(k == 'positive' for k in kinds[14:40]))
        # Before it is observable the object counts as absent, as in the synthetic benchmark.
        self.assertEqual(kinds[0], 'unobservable')
        self.assertEqual(kinds[13], 'unobservable')
        self.assertEqual(tool.LABEL[kinds[13]], 'negative')
        self.assertEqual(kinds[40], 'passing')  # 1.5 m at 15 m/s: reached within 0.1 s
        self.assertEqual(tool.LABEL[kinds[40]], 'uncertain')
        self.assertEqual(kinds[42], 'empty')
        self.assertEqual(kinds[50:55], ['outside'] * 5)
        self.assertEqual(unlinked, [])
        references = tool.events(frames, [inside, outside], spans)[0]['reference_frames']
        self.assertTrue(all(r['uncertainty_m'] is None for r in references))
        self.assertEqual(references[0]['person_roi_assumed_m']['min'][0],
                         references[0]['distance_m'])
        covered = tool.intervals(frames, kinds, notes, [inside, outside], horizon=100.0)
        self.assertEqual(covered[0]['start_ns'], frames[0]['bag_stamp_ns'])
        self.assertEqual(covered[-1]['end_ns'], frames[-1]['bag_stamp_ns'])
        for left, right in zip(covered, covered[1:]):
            self.assertLess(left['end_ns'], right['start_ns'])

    def test_observable_inside_object_outranks_a_near_outside_one(self):
        frames = [frame(0.1 * i, obj(150.0 - 1.5 * i, n=5 if i < 5 else 20),
                        obj(12.0 - i, y=-3.0, n=50)) for i in range(10)]
        tracks = [hand_track(frames, 'a', 'inside', 0), hand_track(frames, 'b', 'outside', 1)]
        kinds, _, spans, _ = tool.label_frames(frames, tracks, horizon=200.0, min_returns=10,
                                               near=1.0)
        self.assertEqual(kinds[:5], ['outside'] * 5)  # the inside object is not observable yet
        self.assertEqual(kinds[5:], ['positive'] * 5)
        self.assertEqual(spans['a'], (5, 9))

    def test_object_at_the_lidar_is_passing(self):
        frames = [frame(0.1 * i, *([obj(x, n=50)] if x is not None else []))
                  for i, x in enumerate([5.0, 3.0, 1.5, 0.5, 0.2, None])]
        track = hand_track(frames[:5], 'a', 'inside', 0)
        kinds, _, _, _ = tool.label_frames(frames, [track], horizon=200.0, min_returns=10,
                                           near=1.0)
        self.assertEqual(kinds, ['positive'] * 3 + ['passing'] * 2 + ['empty'])

    def test_unlinked_returns(self):
        frames = [frame(0.0, obj(30.0, n=50)), frame(0.1, obj(50.0, n=3))]
        kinds, _, _, unlinked = tool.label_frames(frames, [], horizon=200.0, min_returns=10,
                                                  near=1.0)
        self.assertEqual(kinds, ['unexplained', 'unobservable'])
        self.assertEqual([tool.LABEL[kind] for kind in kinds], ['uncertain', 'negative'])
        self.assertEqual(len(unlinked), 2)

    def test_never_observable_inside_object_is_an_error(self):
        frames = [frame(0.1 * k, obj(20.0 - k, n=3)) for k in range(10)]
        track = tool.follow_object(frames, {'id': 'a', 'organizer_class': 'inside',
                                            'description': 'test', 'video_s': [0, 0.9]})
        with self.assertRaises(ValueError):
            tool.label_frames(frames, [track], horizon=100.0, min_returns=10, near=1.0)


class SceneTest(unittest.TestCase):

    def test_scene_objects_come_from_the_bag_entry(self):
        objects = [{'id': 'a', 'organizer_class': 'inside', 'video_s': [0, 1],
                    'description': 'test'}]
        context = {'bags': {'scene': {'objects': objects}, 'plain': {'description': 'no table'}}}
        self.assertEqual(tool.scene_objects(context, 'scene'), objects)
        with self.assertRaises(ValueError):
            tool.scene_objects(context, 'plain')
        with self.assertRaises(ValueError):
            tool.scene_objects(context, 'missing')


if __name__ == '__main__':
    unittest.main()
