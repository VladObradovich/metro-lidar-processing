#!/usr/bin/env python3
"""Check geometric B0 through ROS topics and sequential bag evaluation."""
import json
import os
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

from metro_perception_interfaces.msg import FrameAnalysis, PathAssessment

import rclpy
from rclpy.serialization import serialize_message

import rosbag2_py

from sensor_msgs.msg import PointCloud2, PointField


def cloud(with_obstacle):
    """Build a level floor with an optional object in the path corridor."""
    points = [
        (float(x), y * 0.2, -1.0)
        for x in range(4, 51)
        for y in range(-12, 13)
    ]
    if with_obstacle:
        points += [
            (20.0 + x * 0.08, -0.32 + y * 0.08, -0.7 + z * 0.12)
            for x in range(7)
            for y in range(9)
            for z in range(12)
        ]
    msg = PointCloud2()
    msg.header.frame_id = 'synthetic_lidar'
    msg.header.stamp.sec = 123 if with_obstacle else 124
    msg.height = 1
    msg.width = len(points)
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.fields = [
        PointField(name=name, offset=index * 4,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate('xyz')
    ]
    msg.data = b''.join(struct.pack('<fff', *point) for point in points)
    return msg


def main():
    """Compare online and offline results for positive and clear scenes."""
    rclpy.init(args=[])
    node = rclpy.create_node('detector_smoke_client')
    analyses, assessments = [], []
    node.create_subscription(
        FrameAnalysis, '/detector_smoke/analysis', analyses.append, 10
    )
    node.create_subscription(
        PathAssessment, '/detector_smoke/assessment', assessments.append, 10
    )
    publisher = node.create_publisher(
        PointCloud2, '/detector_smoke/points', 1
    )
    with tempfile.TemporaryDirectory(prefix='metro-detector-') as temp:
        root = Path(temp)
        profile = root / 'sensor.yaml'
        profile.write_text("""source_frame: synthetic_lidar
target_frame: synthetic_lidar
calibration_verified: true
calibration_source: synthetic identity fixture
translation_m: null
rotation_rpy_rad: null
detector:
  background_history_frames: 0
""")
        launch = subprocess.Popen([
            'ros2', 'launch', 'metro_perception_bringup',
            'perception.launch.py',
            'namespace:=detector_smoke',
            'input_topic:=/detector_smoke/points',
            f'sensor_profile:={profile}',
        ], start_new_session=True)

        def wait_until(predicate, seconds=12):
            deadline = time.monotonic() + seconds
            while not predicate():
                if launch.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(
                        'Detector smoke timed out or launch exited'
                    )
                rclpy.spin_once(node, timeout_sec=0.05)

        try:
            wait_until(
                lambda: publisher.get_subscription_count() > 0
                and bool(assessments)
            )
            cases = [
                (True, PathAssessment.OBSTACLE),
                (False, PathAssessment.NO_OBSTACLE_DETECTED),
            ]
            for with_obstacle, expected_state in cases:
                msg = cloud(with_obstacle)
                deadline = time.monotonic() + 12
                while not any(
                    a.header.stamp.sec == msg.header.stamp.sec
                    for a in analyses
                ):
                    if time.monotonic() > deadline:
                        raise RuntimeError('No frame analysis')
                    publisher.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.1)
                analysis = next(
                    a for a in analyses
                    if a.header.stamp.sec == msg.header.stamp.sec
                )
                assert analysis.processing_status == FrameAnalysis.OK, (
                    analysis.reason
                )
                assert analysis.evaluation_region_valid
                wait_until(lambda: any(
                    a.header.stamp.sec == msg.header.stamp.sec
                    and a.frame_sequence == analysis.frame_sequence
                    for a in assessments
                ))
                assessment = next(
                    a for a in assessments
                    if a.header.stamp.sec == msg.header.stamp.sec
                    and a.frame_sequence == analysis.frame_sequence
                )
                assert assessment.state == expected_state, assessment.reason
                assert assessment.distance_valid == with_obstacle
                if with_obstacle:
                    assert analysis.candidates
                    assert assessment.confirmed_objects
                    assert abs(assessment.distance_m - 20.0) < 0.2
                else:
                    assert not analysis.candidates
                    assert not assessment.confirmed_objects

                bag = root / ('positive' if with_obstacle else 'negative')
                writer = rosbag2_py.SequentialWriter()
                writer.open(
                    rosbag2_py.StorageOptions(
                        uri=str(bag), storage_id='sqlite3'
                    ),
                    rosbag2_py.ConverterOptions('', ''),
                )
                writer.create_topic(rosbag2_py.TopicMetadata(
                    name='/points', type='sensor_msgs/msg/PointCloud2',
                    serialization_format='cdr'
                ))
                writer.write(
                    '/points', serialize_message(msg),
                    msg.header.stamp.sec * 10**9
                )
                del writer
                output = root / (bag.name + '.jsonl')
                subprocess.run([
                    'ros2', 'run', 'metro_perception_ros', 'evaluate_bag',
                    str(bag), '/points', str(output), str(profile)
                ], check=True)
                row = json.loads(output.read_text())
                expected = (
                    'OBSTACLE' if with_obstacle else 'NO_OBSTACLE_DETECTED'
                )
                assert row['state'] == expected, row
                assert row['candidate_count'] == len(analysis.candidates)
                assert row['evaluation_region_valid']
                if with_obstacle:
                    assert abs(
                        row['distance_m'] - assessment.distance_m
                    ) < 0.01
                    assert row['candidates']
                    assert row['candidates'][0]['support_points'] > 5
                else:
                    assert row['distance_m'] is None
                    assert not row['candidates']
            print(
                'PASS: geometric B0 positive/negative online and offline agree'
            )
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
