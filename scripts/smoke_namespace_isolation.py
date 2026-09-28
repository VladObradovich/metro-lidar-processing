#!/usr/bin/env python3
"""Verify two namespaced perception stacks stay isolated under remapping and resets."""
import os
import signal
import struct
import subprocess
import time

import rclpy
from metro_perception_interfaces.msg import FrameAnalysis, PathAssessment
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(stamp_sec, frame_id):
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp.sec = stamp_sec
    cloud.height = 1
    cloud.width = 1
    cloud.point_step = 12
    cloud.row_step = 12
    cloud.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(('x', 'y', 'z'))
    ]
    cloud.data = struct.pack('<fff', 1.0, 0.0, 0.0)
    return cloud


def main():
    rclpy.init(args=[])
    node = rclpy.create_node('namespace_isolation_smoke_client')

    analyses_a = []
    analyses_b = []
    assessments_a = []
    assessments_b = []

    node.create_subscription(FrameAnalysis, '/metro_a/analysis', analyses_a.append, 10)
    node.create_subscription(FrameAnalysis, '/metro_b/analysis', analyses_b.append, 10)
    node.create_subscription(PathAssessment, '/metro_a/assessment', assessments_a.append, 10)
    node.create_subscription(PathAssessment, '/metro_b/assessment', assessments_b.append, 10)

    publisher_a = node.create_publisher(PointCloud2, '/ci/isolation/a/points', 1)
    publisher_b = node.create_publisher(PointCloud2, '/ci/isolation/b/points', 1)

    launch_a = subprocess.Popen([
        'ros2', 'launch', 'metro_perception_bringup', 'perception.launch.py',
        'namespace:=metro_a',
        'input_topic:=/ci/isolation/a/points',
        'use_sim_time:=false',
    ], start_new_session=True)
    launch_b = subprocess.Popen([
        'ros2', 'launch', 'metro_perception_bringup', 'perception.launch.py',
        'namespace:=metro_b',
        'input_topic:=/ci/isolation/b/points',
        'use_sim_time:=false',
    ], start_new_session=True)
    launches = [('metro_a', launch_a), ('metro_b', launch_b)]

    def ensure_alive():
        for name, process in launches:
            if process.poll() is not None:
                raise RuntimeError(f'{name} launch exited with code {process.returncode}')

    def spin_until(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while not predicate():
            ensure_alive()
            if time.monotonic() > deadline:
                raise RuntimeError('Namespace isolation smoke deadline exceeded')
            rclpy.spin_once(node, timeout_sec=0.05)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ensure_alive()
            rclpy.spin_once(node, timeout_sec=0.05)

    def find_analysis(rows, stamp_sec):
        return next(
            (row for row in reversed(rows) if row.header.stamp.sec == stamp_sec),
            None,
        )

    def assessment_seen(rows, analysis):
        return any(
            row.source_instance_id == analysis.source_instance_id
            and row.session_id == analysis.session_id
            and row.frame_sequence == analysis.frame_sequence
            and not row.stale
            for row in rows
        )

    def publish_and_wait(publisher, cloud, rows):
        publisher.publish(cloud)
        spin_until(lambda: find_analysis(rows, cloud.header.stamp.sec) is not None)
        return find_analysis(rows, cloud.header.stamp.sec)

    def publish_until_chain(publisher, cloud, analyses, assessments, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publisher.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.1)
            analysis = find_analysis(analyses, cloud.header.stamp.sec)
            if analysis is not None and assessment_seen(assessments, analysis):
                return analysis
            ensure_alive()
        raise RuntimeError(
            f'No matching analysis/assessment chain for stamp {cloud.header.stamp.sec}'
        )

    try:
        expected_nodes = {
            ('perception', '/metro_a'),
            ('obstacle_monitor', '/metro_a'),
            ('perception', '/metro_b'),
            ('obstacle_monitor', '/metro_b'),
        }
        expected_topics = {
            '/metro_a/analysis',
            '/metro_a/assessment',
            '/metro_b/analysis',
            '/metro_b/assessment',
        }

        def graph_ready():
            topic_names = {name for name, _ in node.get_topic_names_and_types()}
            if not expected_nodes.issubset(set(node.get_node_names_and_namespaces())):
                return False
            if not expected_topics.issubset(topic_names):
                return False
            if publisher_a.get_subscription_count() == 0:
                return False
            if publisher_b.get_subscription_count() == 0:
                return False

            # The test client is one subscriber to each analysis topic; the second one
            # must be the namespaced obstacle monitor. Waiting for both sides of every
            # internal edge avoids losing the first volatile sample during DDS discovery.
            for namespace in ('metro_a', 'metro_b'):
                analysis = f'/{namespace}/analysis'
                assessment = f'/{namespace}/assessment'
                if len(node.get_publishers_info_by_topic(analysis)) < 1:
                    return False
                if len(node.get_subscriptions_info_by_topic(analysis)) < 2:
                    return False
                if len(node.get_publishers_info_by_topic(assessment)) < 1:
                    return False
            return True

        spin_until(graph_ready, seconds=15)

        topic_names = {name for name, _ in node.get_topic_names_and_types()}
        assert '/analysis' not in topic_names
        assert '/assessment' not in topic_names

        first_a = publish_until_chain(
            publisher_a,
            make_cloud(100, 'private_a_lidar'),
            analyses_a,
            assessments_a,
        )
        assert first_a.reason == 'GROUND_UNSUPPORTED'
        assert first_a.transform_applied
        assert first_a.calibration_assumed
        spin_for(0.5)
        assert not analyses_b
        assert all(
            not row.source_instance_id
            or row.source_instance_id == first_a.source_instance_id
            for row in assessments_a
        )
        assert all(
            not row.source_instance_id
            or row.source_instance_id != first_a.source_instance_id
            for row in assessments_b
        )

        a_count_before_b = len(analyses_a)
        first_b = publish_until_chain(
            publisher_b,
            make_cloud(200, 'private_b_lidar'),
            analyses_b,
            assessments_b,
        )
        assert first_b.reason == 'GROUND_UNSUPPORTED'
        assert first_b.transform_applied
        assert first_b.calibration_assumed
        assert first_b.source_instance_id != first_a.source_instance_id
        spin_for(0.5)
        assert len(analyses_a) == a_count_before_b
        assert all(
            not row.source_instance_id
            or row.source_instance_id == first_b.source_instance_id
            for row in assessments_b
        )
        assert all(
            not row.source_instance_id
            or row.source_instance_id != first_b.source_instance_id
            for row in assessments_a
        )

        publisher_a.publish(make_cloud(300, 'private_a_lidar'))
        publisher_b.publish(make_cloud(400, 'private_b_lidar'))
        spin_until(
            lambda: find_analysis(analyses_a, 300) is not None
            and find_analysis(analyses_b, 400) is not None
        )
        simultaneous_a = find_analysis(analyses_a, 300)
        simultaneous_b = find_analysis(analyses_b, 400)
        assert simultaneous_a is not None
        assert simultaneous_b is not None
        assert simultaneous_a.source_instance_id == first_a.source_instance_id
        assert simultaneous_b.source_instance_id == first_b.source_instance_id
        assert simultaneous_a.frame_sequence > first_a.frame_sequence
        assert simultaneous_b.frame_sequence > first_b.frame_sequence

        b_count_before_reset = len(analyses_b)
        reset_a = publish_and_wait(
            publisher_a,
            make_cloud(250, 'private_a_lidar_after_reset'),
            analyses_a,
        )
        assert reset_a is not None
        assert reset_a.session_id > simultaneous_a.session_id
        assert reset_a.source_instance_id == first_a.source_instance_id
        assert reset_a.transform_applied

        spin_for(0.5)
        assert len(analyses_b) == b_count_before_reset

        after_reset_b = publish_and_wait(
            publisher_b,
            make_cloud(500, 'private_b_lidar'),
            analyses_b,
        )
        assert after_reset_b is not None
        assert after_reset_b.session_id == simultaneous_b.session_id
        assert after_reset_b.frame_sequence > simultaneous_b.frame_sequence
        assert after_reset_b.source_instance_id == first_b.source_instance_id

        spin_until(lambda: assessment_seen(assessments_a, reset_a))
        spin_until(lambda: assessment_seen(assessments_b, after_reset_b))

        assert all(
            not row.source_instance_id
            or row.source_instance_id == first_a.source_instance_id
            for row in assessments_a
        )
        assert all(
            not row.source_instance_id
            or row.source_instance_id == first_b.source_instance_id
            for row in assessments_b
        )

        print(
            'PASS: two namespaces keep remapped inputs, outputs, source IDs, '
            'frame sequences and session resets isolated'
        )
    finally:
        errors = []
        for name, process in launches:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for name, process in launches:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                errors.append(f'{name} did not stop after SIGINT')

        node.destroy_node()
        rclpy.shutdown()

        if errors:
            raise RuntimeError('; '.join(errors))


if __name__ == '__main__':
    main()
