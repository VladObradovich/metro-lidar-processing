#!/usr/bin/env python3
"""Check geometric B0 through ROS topics, markers and sequential bag evaluation."""
import json
import math
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

from visualization_msgs.msg import MarkerArray

STATE_NAMES = {
    PathAssessment.UNKNOWN: 'UNKNOWN',
    PathAssessment.OBSTACLE: 'OBSTACLE',
    PathAssessment.NO_OBSTACLE_DETECTED: 'NO_OBSTACLE_DETECTED',
}


def cloud(kind, stamp_sec):
    """Build a level floor ('clear'), floor and object ('obstacle') or object only."""
    points = []
    if kind != 'unknown':
        points += [
            (float(x), y * 0.2, -1.0)
            for x in range(4, 51)
            for y in range(-12, 13)
        ]
    if kind != 'clear':
        points += [
            (20.0 + x * 0.08, -0.32 + y * 0.08, -0.7 + z * 0.12)
            for x in range(7)
            for y in range(9)
            for z in range(12)
        ]
    msg = PointCloud2()
    msg.header.frame_id = 'synthetic_lidar'
    msg.header.stamp.sec = stamp_sec
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


def same_candidates(reported, analysed):
    """Compare copied candidates field by field (distance NaN equals NaN)."""
    if len(reported) != len(analysed):
        return False
    for a, b in zip(reported, analysed):
        if a.candidate_id != b.candidate_id or a.bbox != b.bbox:
            return False
        if a.distance_valid != b.distance_valid or a.support_points != b.support_points:
            return False
        if not (a.distance_m == b.distance_m
                or (math.isnan(a.distance_m) and math.isnan(b.distance_m))):
            return False
    return True


def main():
    """Compare online and offline results for clear, unknown and positive scenes."""
    rclpy.init(args=[])
    node = rclpy.create_node('detector_smoke_client')
    analyses, assessments, markers = [], [], []
    node.create_subscription(
        FrameAnalysis, '/detector_smoke/analysis', analyses.append, 10
    )
    node.create_subscription(
        PathAssessment, '/detector_smoke/assessment', assessments.append, 10
    )
    node.create_subscription(
        MarkerArray, '/detector_smoke/markers', markers.append, 10
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
            'demo.launch.py', 'rviz:=false',
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

        def evaluate_offline(msg, name):
            bag = root / name
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''),
            )
            writer.create_topic(rosbag2_py.TopicMetadata(
                name='/points', type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr'
            ))
            writer.write(
                '/points', serialize_message(msg), msg.header.stamp.sec * 10**9
            )
            del writer
            output = root / (name + '.jsonl')
            subprocess.run([
                'ros2', 'run', 'metro_perception_ros', 'evaluate_bag',
                str(bag), '/points', str(output), str(profile)
            ], check=True)
            return json.loads(output.read_text())

        try:
            wait_until(
                lambda: publisher.get_subscription_count() > 0
                and bool(assessments)
            )
            # Increasing stamps keep one session; the obstacle comes last for the timeout.
            cases = [
                ('clear', 123, PathAssessment.NO_OBSTACLE_DETECTED,
                 'NO_CANDIDATE_IN_EVALUATED_REGION'),
                ('unknown', 124, PathAssessment.UNKNOWN, 'GROUND_UNSUPPORTED'),
                ('obstacle', 125, PathAssessment.OBSTACLE, 'OBSTACLE_CANDIDATE'),
            ]
            for kind, stamp_sec, expected_state, expected_reason in cases:
                msg = cloud(kind, stamp_sec)
                deadline = time.monotonic() + 12
                while not any(a.header.stamp.sec == stamp_sec for a in analyses):
                    if time.monotonic() > deadline:
                        raise RuntimeError(f'No frame analysis for {kind}')
                    publisher.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.1)
                analysis = next(a for a in analyses if a.header.stamp.sec == stamp_sec)

                def matching():
                    return next((
                        a for a in assessments
                        if a.header.stamp.sec == stamp_sec
                        and a.frame_sequence == analysis.frame_sequence
                        and not a.stale
                    ), None)
                wait_until(lambda: matching() is not None)
                assessment = matching()
                assert assessment.state == expected_state, (kind, assessment.reason)
                assert assessment.reason == expected_reason, (kind, assessment.reason)
                assert assessment.source_instance_id == analysis.source_instance_id
                assert assessment.session_id == analysis.session_id
                if kind == 'unknown':
                    assert analysis.processing_status != FrameAnalysis.OK
                    assert not assessment.reported_objects and not assessment.corridor
                    assert not assessment.evaluation_region_valid
                else:
                    assert analysis.processing_status == FrameAnalysis.OK, analysis.reason
                    assert analysis.evaluation_region_valid
                    # Objects and corridor come from exactly the frame that was decided.
                    assert same_candidates(assessment.reported_objects, analysis.candidates)
                    assert list(assessment.corridor) == list(analysis.corridor)
                    assert assessment.corridor
                    assert assessment.evaluated_range_m == analysis.evaluated_range_m
                if kind == 'obstacle':
                    assert assessment.distance_valid
                    assert abs(assessment.distance_m - 20.0) < 0.2
                    assert assessment.reported_objects
                else:
                    assert not assessment.distance_valid
                    assert math.isnan(assessment.distance_m)
                    assert not assessment.reported_objects

                row = evaluate_offline(msg, kind)
                assert row['state'] == STATE_NAMES[assessment.state], (kind, row)
                assert row['reason'] == assessment.reason, (kind, row)
                assert row['distance_valid'] == assessment.distance_valid
                assert row['candidate_count'] == len(analysis.candidates)
                assert row['evaluation_region_valid'] == analysis.evaluation_region_valid
                if kind == 'obstacle':
                    assert abs(row['distance_m'] - assessment.distance_m) < 0.01
                    assert row['candidates'][0]['support_points'] > 5
                else:
                    assert row['distance_m'] is None
                    assert not row['candidates']

            def drawn(array, namespace):
                return any(m.ns == namespace for m in array.markers)
            wait_until(lambda: any(drawn(a, 'candidate_bbox') and drawn(a, 'corridor')
                                   and drawn(a, 'nearest_point') for a in markers))
            # Stop publishing: the watchdog must drop the old object and corridor.
            wait_until(lambda: any(a.stale and a.header.stamp.sec == 125
                                   for a in assessments), seconds=5)
            stale = next(a for a in assessments if a.stale and a.header.stamp.sec == 125)
            # The heartbeat keeps the key of the last accepted analysis of that frame.
            last = max((a for a in analyses if a.header.stamp.sec == 125),
                       key=lambda a: a.frame_sequence)
            assert stale.frame_sequence == last.frame_sequence
            assert stale.state == PathAssessment.UNKNOWN
            assert stale.reason == 'INPUT_PAUSED_OR_STOPPED'
            assert stale.header.stamp == last.header.stamp
            assert not stale.distance_valid and math.isnan(stale.distance_m)
            assert not stale.reported_objects and not stale.corridor
            assert not stale.evaluation_region_valid
            markers.clear()
            wait_until(lambda: any(
                any('INPUT_PAUSED_OR_STOPPED' in m.text for m in a.markers) for a in markers))
            stale_markers = next(a for a in markers
                                 if any('INPUT_PAUSED_OR_STOPPED' in m.text for m in a.markers))
            assert not drawn(stale_markers, 'candidate_bbox')
            assert not drawn(stale_markers, 'corridor')
            assert stale_markers.markers[0].action == stale_markers.markers[0].DELETEALL
            print(
                'PASS: B0 clear/unknown/obstacle online and offline agree; objects and '
                'corridor follow the frame; timeout clears them'
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
