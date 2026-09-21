#!/usr/bin/env python3
"""Load-test the full perception node's latest-only worker and reset isolation."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from metro_perception_interfaces.msg import FrameAnalysis
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(stamp_sec):
    """Build a one-point XYZ cloud for deterministic worker timing."""
    cloud = PointCloud2()
    cloud.header.frame_id = 'worker_test_lidar'
    cloud.header.stamp.sec = stamp_sec
    cloud.height = 1
    cloud.width = 1
    cloud.point_step = 12
    cloud.row_step = 12
    cloud.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(('x', 'y', 'z'))
    ]
    cloud.data = struct.pack('<fff', 0.0, -10.0, 0.0)
    return cloud


def main():
    """Run overload, reset-isolation and clean-shutdown checks."""
    rclpy.init(args=[])
    node = rclpy.create_node('worker_overload_smoke_client')
    analyses = []
    node.create_subscription(FrameAnalysis, '/worker_smoke/analysis', analyses.append, 20)
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(PointCloud2, '/worker_smoke/points', qos)

    with tempfile.TemporaryDirectory(prefix='metro-worker-') as temp:
        root = Path(temp)
        profile = root / 'sensor.yaml'
        profile.write_text(
            'source_frame: worker_test_lidar\n'
            'target_frame: worker_test_base\n'
            'calibration_verified: false\n'
            'translation_m: null\n'
            'rotation_rpy_rad: null\n'
        )
        runtime = root / 'runtime.yaml'
        runtime.write_text(
            '/**:\n'
            '  ros__parameters:\n'
            '    input_reliability: reliable\n'
            '    max_processing_age_s: 2.0\n'
            '    tf_wait_timeout_s: 0.25\n'
            '    timeout_s: 1.0\n'
        )

        launch = subprocess.Popen([
            'ros2', 'launch', 'metro_perception_bringup', 'perception.launch.py',
            'namespace:=worker_smoke',
            'input_topic:=/worker_smoke/points',
            f'sensor_profile:={profile}',
            f'runtime_config:={runtime}',
            'use_sim_time:=false',
        ], start_new_session=True)

        def ensure_alive():
            if launch.poll() is not None:
                raise RuntimeError(
                    f'worker smoke launch exited with code {launch.returncode}'
                )

        def spin_until(predicate, seconds=20):
            deadline = time.monotonic() + seconds
            while not predicate():
                ensure_alive()
                if time.monotonic() > deadline:
                    raise RuntimeError('worker overload smoke deadline exceeded')
                rclpy.spin_once(node, timeout_sec=0.05)

        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                ensure_alive()
                rclpy.spin_once(node, timeout_sec=0.05)

        def by_stamp(stamp):
            return next(
                (row for row in reversed(analyses) if row.header.stamp.sec == stamp),
                None,
            )

        try:
            spin_until(lambda: publisher.get_subscription_count() > 0)

            publisher.publish(make_cloud(100))
            spin_until(lambda: by_stamp(100) is not None)
            baseline = by_stamp(100)
            assert baseline is not None

            # Missing TF deliberately keeps frame 200 inside the worker for 250 ms.
            # Subscriber callbacks remain free to exercise the real 1+1 pending slot.
            publisher.publish(make_cloud(200))
            time.sleep(0.05)
            for stamp in (201, 202, 203, 204):
                publisher.publish(make_cloud(stamp))
                time.sleep(0.03)

            spin_until(lambda: by_stamp(204) is not None)
            overloaded = by_stamp(204)
            assert overloaded is not None
            assert by_stamp(200) is not None
            assert by_stamp(201) is None
            assert by_stamp(202) is None
            assert by_stamp(203) is None
            assert overloaded.overwritten_frames > baseline.overwritten_frames
            assert overloaded.received_frames > overloaded.processed_frames
            assert overloaded.frame_sequence <= overloaded.received_frames

            before_reset = len(analyses)
            publisher.publish(make_cloud(400))
            time.sleep(0.05)
            assert by_stamp(400) is None
            publisher.publish(make_cloud(300))
            spin_until(
                lambda: any(
                    row.header.stamp.sec == 300
                    and row.session_id > overloaded.session_id
                    for row in analyses
                )
            )
            reset = next(
                row for row in reversed(analyses)
                if row.header.stamp.sec == 300
                and row.session_id > overloaded.session_id
            )
            assert reset.source_instance_id == overloaded.source_instance_id

            spin_for(0.35)
            assert by_stamp(400) is None
            after_reset = analyses[before_reset:]
            reset_position = after_reset.index(reset)
            assert all(
                row.session_id >= reset.session_id
                for row in after_reset[reset_position:]
            )

            print(
                'PASS: full node overwrites backlog, latest frame wins, '
                'reset blocks stale-session publication, shutdown is bounded'
            )
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
                raise RuntimeError('worker smoke launch did not stop after SIGINT')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
