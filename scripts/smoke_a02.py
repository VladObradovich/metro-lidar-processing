#!/usr/bin/env python3
"""Synthetic A02 online/offline parity and missing-TF integration check (no dataset)."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from metro_perception_interfaces.msg import FrameAnalysis, PathAssessment


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--default", action="store_true", help="Check automatic lidar-only profile")
    args = parser.parse_args()
    reason = "NOT_IMPLEMENTED" if args.default else "CALIBRATION_UNVERIFIED"
    target = "lidar_assumed" if args.default else "test_preview"
    rclpy.init(args=[])
    node = rclpy.create_node('a02_smoke_client')
    analyses, assessments = [], []
    node.create_subscription(FrameAnalysis, '/a02_smoke/analysis', analyses.append, 10)
    node.create_subscription(PathAssessment, '/a02_smoke/assessment', assessments.append, 10)
    publisher = node.create_publisher(PointCloud2, '/a02_smoke/points', 1)
    with tempfile.TemporaryDirectory(prefix='metro-a02-') as temp:
        root = Path(temp)
        profile = root / 'sensor.yaml'
        profile.write_text('''source_frame: test_lidar
target_frame: test_preview
calibration_verified: false
calibration_source: synthetic fixture
translation_m: [1, 2, 3]
rotation_rpy_rad: [0, 0, 1.5707963267948966]
''')
        launch = subprocess.Popen([
            'ros2', 'launch', 'metro_perception_bringup', 'perception.launch.py',
            'namespace:=a02_smoke', 'input_topic:=/a02_smoke/points',
            'publish_sensor_tf:=true', 'use_sim_time:=true'
        ] + ([] if args.default else [f'sensor_profile:={profile}']),
            start_new_session=True)

        def until(predicate, seconds=15):
            end = time.monotonic() + seconds
            while not predicate():
                if launch.poll() is not None or time.monotonic() > end:
                    raise RuntimeError('A02 smoke deadline or launch failure')
                rclpy.spin_once(node, timeout_sec=0.05)

        try:
            until(lambda: publisher.get_subscription_count() > 0 and bool(assessments))
            cloud = PointCloud2()
            cloud.header.frame_id = 'hesai_lidar' if args.default else 'test_lidar'
            cloud.header.stamp.sec = 123
            points = [(0., 0., 0.), (float('nan'), 1., 2.), (0.1, 0., 0.),
                      (0., -10., -1.), (8., -10., 0.), (0., -200., 0.)]
            cloud.height, cloud.width = 1, len(points)
            cloud.point_step, cloud.row_step = 12, 12 * len(points)
            cloud.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate('xyz')]
            cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
            publisher.publish(cloud)
            until(lambda: any(a.reason == reason for a in analyses))
            online = next(a for a in analyses if a.reason == reason)
            assert online.header.frame_id == target
            assert online.header.stamp == cloud.header.stamp
            assert online.transform_applied and not online.calibration_verified
            assert online.calibration_assumed == args.default
            expected_trust = (FrameAnalysis.CALIBRATION_TRUST_ASSUMED if args.default
                              else FrameAnalysis.CALIBRATION_TRUST_UNKNOWN)
            assert online.calibration_trust == expected_trust
            until(lambda: any(a.frame_sequence == online.frame_sequence for a in assessments))
            assessment = next(a for a in assessments if a.frame_sequence == online.frame_sequence)
            assert assessment.calibration_trust == online.calibration_trust
            assert (online.geometry_point_count, online.detection_point_count,
                    online.invalid_point_count, online.blind_point_count,
                    online.outside_roi_point_count) == (2, 1, 2, 1, 1)
            # A late subscriber (e.g. RViz) must receive the published static TF.
            transforms = []
            node.create_subscription(TFMessage, '/tf_static', transforms.append,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            expected_child = 'hesai_lidar' if args.default else 'test_lidar'
            until(lambda: any(t.child_frame_id == expected_child
                              for msg in transforms for t in msg.transforms))
            tf = next(t for msg in transforms for t in msg.transforms
                      if t.child_frame_id == expected_child)
            assert tf.header.frame_id == target
            assert (tf.transform.translation.x, tf.transform.translation.y,
                    tf.transform.translation.z) == ((0, 0, 0) if args.default else (1, 2, 3))
            bag = root / 'bag'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            writer.create_topic(rosbag2_py.TopicMetadata(
                name='/points', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'))
            writer.write('/points', serialize_message(cloud), 124000000000)
            del writer
            result = root / 'frames.jsonl'
            subprocess.run(['ros2', 'run', 'metro_perception_ros', 'evaluate_bag', str(bag),
                            '/points', str(result)] + ([] if args.default else [str(profile)]), check=True)
            offline = json.loads(result.read_text())
            for field in ('reason', 'processing_status', 'transform_applied',
                          'calibration_verified', 'calibration_assumed', 'calibration_trust',
                          'geometry_point_count', 'detection_point_count',
                          'invalid_point_count', 'blind_point_count', 'outside_roi_point_count'):
                assert offline[field] == getattr(online, field), field
            assert offline['measurement_stamp_ns'] == 123000000000
            assert offline['bag_stamp_ns'] == 124000000000
            # Measurement time goes backwards: new session, same single-frame crop.
            cloud.header.stamp.sec = 120
            publisher.publish(cloud)
            until(lambda: any(a.session_id > online.session_id for a in analyses))
            reset = next(a for a in analyses if a.session_id > online.session_id)
            assert reset.transform_applied and reset.geometry_point_count == 2
            assert reset.reason == reason
            # Same configured profile must never transform an unexpected source frame.
            cloud.header.frame_id = 'unknown_sensor'
            cloud.header.stamp.sec = 124
            publisher.publish(cloud)
            until(lambda: any(a.reason == 'TF_UNAVAILABLE' for a in analyses))
            failed = next(a for a in analyses if a.reason == 'TF_UNAVAILABLE')
            assert not failed.transform_applied and failed.geometry_point_count == 0
            assert failed.header.frame_id == 'unknown_sensor'
            assert failed.calibration_trust == FrameAnalysis.CALIBRATION_TRUST_UNKNOWN
            until(lambda: any(a.reason == 'TF_UNAVAILABLE' for a in assessments))
            until(lambda: any(a.stale and a.frame_sequence > 0 for a in assessments))
            assert all(a.state == PathAssessment.UNKNOWN and not a.distance_valid
                       and not a.evaluation_region_valid for a in assessments)
            print(f'PASS: A02 default={args.default}, online/offline parity, late static TF, reset, missing TF, UNKNOWN/watchdog')
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
                raise RuntimeError('A02 launch did not stop')
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
