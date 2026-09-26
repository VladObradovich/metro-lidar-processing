#!/usr/bin/env python3
"""Check scaffold UNKNOWN, identity, watchdog, best-effort input, demo TF and shutdown."""
import signal
import os
import struct
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from metro_perception_interfaces.msg import PathAssessment


def main():
    rclpy.init()
    node = rclpy.create_node('scaffold_smoke_client')
    received = []
    markers = []
    subscription = node.create_subscription(
        PathAssessment, '/scaffold_smoke/assessment', received.append, 10)
    # A live driver publishes best effort: input_reliability auto must switch to it.
    publisher = node.create_publisher(
        PointCloud2, '/scaffold_smoke/points', qos_profile_sensor_data)
    marker_subscription = node.create_subscription(
        MarkerArray, '/scaffold_smoke/markers', markers.append, 10)
    static_tf = []
    tf_subscription = node.create_subscription(
        TFMessage, '/tf_static', static_tf.append,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    launch = subprocess.Popen([
        'ros2', 'launch', 'metro_perception_bringup', 'demo.launch.py', 'rviz:=false',
        'namespace:=scaffold_smoke', 'input_topic:=/scaffold_smoke/points',
        'use_sim_time:=true',  # No /clock: watchdog must still use steady time.
    ], start_new_session=True)

    def spin_until(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while not predicate():
            if launch.poll() is not None:
                raise RuntimeError('Launch exited before smoke completed')
            if time.monotonic() > deadline:
                raise RuntimeError('Smoke deadline exceeded')
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0 and bool(received))
        cloud = PointCloud2()
        cloud.header.frame_id = 'private_scaffold_lidar'
        cloud.header.stamp.sec = 123
        cloud.height = cloud.width = 1
        cloud.point_step = cloud.row_step = 12
        cloud.fields = [PointField(name=name, offset=index * 4,
                                   datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate(('x', 'y', 'z'))]
        cloud.data = struct.pack('<fff', 0., -10., 0.)
        deadline = time.monotonic() + 10
        while not any(row.reason == 'GROUND_UNSUPPORTED' and not row.stale for row in received):
            if time.monotonic() > deadline:
                raise RuntimeError('No decoded frame assessment')
            publisher.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.1)
        fresh = next(row for row in reversed(received) if row.reason == 'GROUND_UNSUPPORTED')
        # The demo publishes the bound input frame so RViz can draw the cloud.
        spin_until(lambda: any(t.header.frame_id == 'lidar_assumed' and
                               t.child_frame_id == 'private_scaffold_lidar'
                               for message in static_tf for t in message.transforms), seconds=3)
        spin_until(lambda: any(row.reason == 'INPUT_PAUSED_OR_STOPPED' and row.stale
                               for row in received), seconds=3)
        stale = next(row for row in reversed(received) if row.stale and row.frame_sequence)
        assert all(row.state == PathAssessment.UNKNOWN for row in received)
        assert all(not row.distance_valid and not row.evaluation_region_valid for row in received)
        assert stale.header.stamp == fresh.header.stamp
        assert stale.header.frame_id == 'lidar_assumed'
        assert stale.source_instance_id == fresh.source_instance_id
        assert stale.result_age_ms >= 500
        spin_until(lambda: any('INPUT_PAUSED_OR_STOPPED' in marker.text
                               for array in markers for marker in array.markers), seconds=3)
        print(
            'PASS: best-effort lidar-only no-ring cloud -> GROUND_UNSUPPORTED/UNKNOWN '
            '-> bound frame on /tf_static -> steady-clock timeout; stamp preserved'
        )
    finally:
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
            raise RuntimeError('Launch did not stop after SIGINT')
        node.destroy_subscription(subscription)
        node.destroy_subscription(marker_subscription)
        node.destroy_subscription(tf_subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
