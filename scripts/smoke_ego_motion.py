#!/usr/bin/env python3
"""Exercise lidar motion compensation through ROS topics and bag evaluation."""

import json
import os
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from metro_perception_interfaces.msg import FrameAnalysis, PathAssessment


def moving_tunnel(index, obstacle=False):
    travelled = index  # 10 m/s at 10 Hz.
    points = [(float(x), yi * 0.2, -1.0)
              for x in range(1, 101) for yi in range(-12, 13)]
    spacing = (9.3, 13.7, 8.9, 14.4, 11.8, 10.1, 15.2, 12.2)
    post = 3.0
    i = 0
    while post < 260:
        x = post - travelled
        if 1 <= x <= 100:
            for y in (-1.9, 1.8):
                for dx in (0.0, 0.1, 0.2):
                    for h in range(25):
                        points.append((x + dx, y, -1.0 + h * 0.1))
        post += spacing[i % len(spacing)]
        i += 1
    niche = (0.7, 2.9, 1.3, 2.1, 0.9, 3.0, 1.6, 2.4, 1.1)
    edge = 0.0
    i = 0
    while edge < 260:
        lateral = 3.0 if (i + 1) % 2 else 2.6
        xw = edge
        while xw < edge + niche[(i + 1) % len(niche)]:
            x = xw - travelled
            if 1 <= x <= 100:
                for h in range(1, 10):
                    for side in (-1.0, 1.0):
                        points.append((x, side * lateral, -1.0 + h * 0.3))
            xw += 0.1
        edge += niche[i % len(niche)]
        i += 1
    if obstacle:
        for xi in range(7):
            for yi in range(11):
                for hi in range(18):
                    points.append((20 + xi * 0.08, -0.4 + yi * 0.08,
                                   -0.7 + hi * 0.08))
    msg = PointCloud2()
    msg.header.frame_id = 'synthetic_lidar'
    msg.header.stamp.sec = 123 + index // 10
    msg.header.stamp.nanosec = index % 10 * 100_000_000
    msg.height = 1
    msg.width = len(points)
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.fields = [PointField(name=name, offset=i * 4,
                             datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate('xyz')]
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    return msg


def main():
    rclpy.init(args=[])
    node = rclpy.create_node('ego_motion_smoke_client')
    analyses, assessments = [], []
    node.create_subscription(FrameAnalysis, '/ego_smoke/analysis', analyses.append, 10)
    node.create_subscription(PathAssessment, '/ego_smoke/assessment', assessments.append, 10)
    publisher = node.create_publisher(PointCloud2, '/ego_smoke/points', 1)
    with tempfile.TemporaryDirectory(prefix='metro-ego-smoke-') as temp:
        root = Path(temp)
        profile = root / 'sensor.yaml'
        profile.write_text('''source_frame: synthetic_lidar
target_frame: synthetic_lidar
calibration_verified: true
calibration_source: synthetic identity fixture
translation_m: null
rotation_rpy_rad: null
detector:
  background_history_frames: 10
  background_lag_frames: 5
  ego_motion_compensation: true
''')
        launch = subprocess.Popen([
            'ros2', 'launch', 'metro_perception_bringup', 'perception.launch.py',
            'namespace:=ego_smoke', 'input_topic:=/ego_smoke/points',
            f'sensor_profile:={profile}',
        ], start_new_session=True)

        def wait_for(predicate, seconds=15):
            deadline = time.monotonic() + seconds
            while not predicate():
                if launch.poll() is not None or time.monotonic() > deadline:
                    recent_analyses = [
                        (a.frame_sequence, a.header.stamp.sec,
                         a.header.stamp.nanosec, a.reason)
                        for a in analyses[-3:]
                    ]
                    recent_assessments = [
                        (a.frame_sequence, a.header.stamp.sec,
                         a.header.stamp.nanosec, a.reason)
                        for a in assessments[-3:]
                    ]
                    raise RuntimeError(
                        'Ego-motion ROS smoke timed out or launch exited; '
                        f'analyses={recent_analyses}, '
                        f'assessments={recent_assessments}'
                    )
                rclpy.spin_once(node, timeout_sec=0.05)

        try:
            wait_for(lambda: publisher.get_subscription_count() > 0 and
                     node.count_subscribers('/ego_smoke/analysis') >= 2 and
                     assessments)
            bag = root / 'moving_tunnel'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            writer.create_topic(rosbag2_py.TopicMetadata(
                name='/points', type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr'))
            for i in range(17):
                msg = moving_tunnel(i, obstacle=i == 16)
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                writer.write('/points', serialize_message(msg),
                             stamp[0] * 1_000_000_000 + stamp[1])
                publisher.publish(msg)
                wait_for(lambda: any(
                    a.header.stamp.sec == stamp[0] and
                    a.header.stamp.nanosec == stamp[1]
                    for a in analyses))
                analysis = next(a for a in analyses
                                if a.header.stamp.sec == stamp[0] and
                                a.header.stamp.nanosec == stamp[1])
                wait_for(lambda: any(a.frame_sequence == analysis.frame_sequence
                                     and a.header.stamp.sec == stamp[0]
                                     and a.header.stamp.nanosec == stamp[1]
                                     for a in assessments))
                assessment = next(a for a in assessments
                                  if a.frame_sequence == analysis.frame_sequence
                                  and a.header.stamp.sec == stamp[0]
                                  and a.header.stamp.nanosec == stamp[1])
                if i == 15:
                    assert analysis.processing_status == FrameAnalysis.OK, analysis.reason
                    assert not analysis.candidates
                    assert assessment.state == PathAssessment.UNKNOWN, (
                        assessment.state, assessment.reason, analysis.reason)
                    assert assessment.reason == 'BACKGROUND_CANNOT_CONFIRM_CLEAR'
                if i == 16:
                    assert analysis.processing_status == FrameAnalysis.OK, analysis.reason
                    assert analysis.candidates
                    assert assessment.state == PathAssessment.OBSTACLE
                    assert assessment.distance_valid
                    assert abs(assessment.distance_m - 20.0) < 0.2, (
                        assessment.distance_m,
                        [(c.distance_m, c.bbox.center.position.x)
                         for c in analysis.candidates])
            del writer
            output = root / 'moving_tunnel.jsonl'
            subprocess.run(['ros2', 'run', 'metro_perception_ros', 'evaluate_bag',
                            str(bag), '/points', str(output), str(profile)], check=True)
            rows = [json.loads(line) for line in output.read_text().splitlines()]
            assert len(rows) == 17
            assert rows[15]['ego_motion_valid']
            assert abs(rows[15]['ego_speed_mps'] - 10.0) < 0.6
            assert rows[15]['state'] == 'UNKNOWN'
            assert rows[15]['reason'] == 'BACKGROUND_CANNOT_CONFIRM_CLEAR'
            assert rows[16]['state'] == 'OBSTACLE'
            assert abs(rows[16]['distance_m'] - 20.0) < 0.2
            control_profile = root / 'no_motion.yaml'
            control_profile.write_text(profile.read_text().replace(
                'ego_motion_compensation: true',
                'ego_motion_compensation: false'))
            control_output = root / 'no_motion.jsonl'
            subprocess.run(['ros2', 'run', 'metro_perception_ros', 'evaluate_bag',
                            str(bag), '/points', str(control_output),
                            str(control_profile)], check=True)
            control = [json.loads(line) for line in
                       control_output.read_text().splitlines()]
            assert control[15]['candidate_count'] > 0
            print('PASS: ROS motion-compensated frames and offline parity')
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
