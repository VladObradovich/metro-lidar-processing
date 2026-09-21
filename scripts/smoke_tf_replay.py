#!/usr/bin/env python3
"""Regress offline TF ordering, session resets and first-cloud source binding."""
import json
from pathlib import Path
import struct
import subprocess
import tempfile

from geometry_msgs.msg import TransformStamped
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


def cloud(sec, frame='lidar', malformed=False):
    """Create an XYZ cloud with an optional broken data layout."""
    message = PointCloud2()
    message.header.stamp.sec = sec
    message.header.frame_id = frame
    message.width = message.height = 1
    message.point_step = message.row_step = 12
    message.fields = [
        PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate('xyz')
    ]
    message.data = b'' if malformed else struct.pack('<fff', 10, -10, 0)
    return message


def transform(sec):
    """Create an identity dynamic transform at a measurement timestamp."""
    message = TransformStamped()
    message.header.stamp.sec = sec
    message.header.frame_id = 'base_link'
    message.child_frame_id = 'lidar'
    message.transform.rotation.w = 1.0
    return TFMessage(transforms=[message])


def main():
    """Export synthetic bags and check exact frame-level regression outcomes."""
    with tempfile.TemporaryDirectory(prefix='metro-tf-replay-') as temp:
        root = Path(temp)
        profile = root / 'dynamic.yaml'
        profile.write_text(
            'source_frame: lidar\ntarget_frame: base_link\n'
            'calibration_verified: true\ncalibration_source: synthetic_test\n'
            'translation_m: null\nrotation_rpy_rad: null\n'
        )

        def check(name, events, expected, default=False, spacing_ns=10000000, lookahead=None):
            bag = root / name
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''),
            )
            for topic, typename in (
                ('/points', 'sensor_msgs/msg/PointCloud2'),
                ('/tf', 'tf2_msgs/msg/TFMessage'),
                ('/tf_static', 'tf2_msgs/msg/TFMessage'),
            ):
                writer.create_topic(rosbag2_py.TopicMetadata(
                    name=topic, type=typename, serialization_format='cdr'))
            for i, (topic, message) in enumerate(events):
                writer.write(topic, serialize_message(message), 1000000000000 + i * spacing_ns)
            del writer
            output = root / (name + '.jsonl')
            command = [
                'ros2', 'run', 'metro_perception_ros', 'evaluate_bag', str(bag),
                '/points', str(output), '' if default else str(profile),
            ]
            if lookahead is not None:
                command.extend(['2000000', '268435456', str(lookahead)])
            subprocess.run(command, check=True)
            rows = [json.loads(line) for line in output.read_text().splitlines()]
            actual = [(row['session_id'], row['processing_status']) for row in rows]
            assert actual == expected, (name, actual, expected)
            for row in rows:
                if row['processing_status'] == 1:
                    assert row['geometry_point_count'] == 1, (name, row)
                    assert row['reason'] == 'NOT_IMPLEMENTED', (name, row)
            print(f'PASS: {name}: {actual}', flush=True)

        check('tf_before', [('/tf', transform(100)), ('/points', cloud(100))], [(0, 1)])
        check('tf_after', [('/points', cloud(100)), ('/tf', transform(100))], [(0, 1)])
        check('tf_too_late', [('/points', cloud(100)), ('/tf', transform(100))],
              [(0, 3)], spacing_ns=100000000)
        check('tf_custom_window', [('/points', cloud(100)), ('/tf', transform(100))],
              [(0, 1)], spacing_ns=100000000, lookahead=0.2)
        check('tf_zero_window', [('/points', cloud(100)), ('/tf', transform(100))],
              [(0, 3)], lookahead=0)
        check('interpolation', [('/tf', transform(99)), ('/points', cloud(100)),
                                ('/tf', transform(101))], [(0, 1)])
        check('no_latest_fallback', [('/points', cloud(100)), ('/tf', transform(101))], [(0, 3)])
        check('static_after', [('/points', cloud(100)), ('/tf_static', transform(0))], [(0, 1)])
        for stamp in (99, 1):  # Also cover resets beyond the TF buffer's cache duration.
            check(f'reset_before_{stamp}', [
                ('/tf', transform(100)), ('/points', cloud(100)),
                ('/tf', transform(stamp)), ('/points', cloud(stamp)),
            ], [(0, 1), (1, 1)])
            check(f'reset_after_{stamp}', [
                ('/tf', transform(100)), ('/points', cloud(100)),
                ('/points', cloud(stamp)), ('/tf', transform(stamp)),
            ], [(0, 1), (1, 1)])
        check('discard_old_session', [
            ('/tf', transform(99)), ('/tf', transform(100)),
            ('/points', cloud(100)), ('/points', cloud(99)),
        ], [(0, 1), (1, 3)])
        check('malformed_first', [
            ('/points', cloud(100, 'must_not_bind', True)), ('/points', cloud(101)),
        ], [(0, 2), (0, 1)], default=True)


if __name__ == '__main__':
    main()
