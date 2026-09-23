#!/usr/bin/env python3
"""Drive obstacle_monitor_node with FrameAnalysis messages: gating, diagnostics, watchdog."""
import math
import os
import signal
import subprocess
import time

from metro_perception_interfaces.msg import (
    CorridorSegment, FrameAnalysis, ObstacleCandidate, PathAssessment)

import rclpy


def analysis(source, sequence, stamp_sec, distances=(), session=0):
    """Build a successful VERIFIED analysis with one corridor segment."""
    msg = FrameAnalysis()
    msg.header.frame_id = 'monitor_smoke'
    msg.header.stamp.sec = stamp_sec
    msg.source_instance_id = source
    msg.session_id = session
    msg.frame_sequence = sequence
    msg.processing_status = FrameAnalysis.OK
    msg.reason = 'OK'
    msg.calibration_trust = FrameAnalysis.CALIBRATION_TRUST_VERIFIED
    msg.evaluation_region_valid = True
    msg.evaluated_range_m = 40.0
    for index, distance in enumerate(distances, start=1):
        candidate = ObstacleCandidate()
        candidate.candidate_id = index
        candidate.distance_m = distance
        candidate.distance_valid = True
        msg.candidates.append(candidate)
    segment = CorridorSegment()
    segment.end.x = 40.0
    segment.width_m = 4.0
    segment.height_m = 3.5
    segment.geometry_valid = segment.coverage_valid = True
    msg.corridor.append(segment)
    return msg


def main():
    """Check the monitor contract through ROS topics only."""
    rclpy.init(args=[])
    node = rclpy.create_node('monitor_smoke_client')
    received = []
    node.create_subscription(PathAssessment, '/monitor_smoke/assessment', received.append, 50)
    publisher = node.create_publisher(FrameAnalysis, '/monitor_smoke/analysis', 10)
    monitor = subprocess.Popen([
        'ros2', 'run', 'metro_perception_ros', 'obstacle_monitor_node', '--ros-args',
        '-r', '__node:=monitor_smoke_node',
        '-r', '~/input/analysis:=/monitor_smoke/analysis',
        '-r', '~/output/assessment:=/monitor_smoke/assessment',
        '-p', 'timeout_s:=3.0',
    ], start_new_session=True)

    def spin_until(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while not predicate():
            if monitor.poll() is not None:
                raise RuntimeError('Monitor exited before smoke completed')
            if time.monotonic() > deadline:
                raise RuntimeError('Monitor smoke deadline exceeded')
            rclpy.spin_once(node, timeout_sec=0.05)

    def latest():
        return received[-1]

    def send(msg):
        publisher.publish(msg)
        spin_until(lambda: latest().source_instance_id == msg.source_instance_id
                   and latest().frame_sequence == msg.frame_sequence and not latest().stale)
        return latest()

    def send_rejected(msg):
        count = latest().rejected_analyses
        publisher.publish(msg)
        spin_until(lambda: latest().rejected_analyses > count)
        return latest()

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0 and bool(received))
        assert latest().reason == 'WAITING_FOR_INPUT' and latest().stale
        assert math.isnan(latest().distance_m)

        first = analysis('source-a', 1, 100, (30.0, float('nan'), 12.5))
        out = send(first)
        assert out.state == PathAssessment.OBSTACLE, out.reason
        assert out.distance_valid and out.distance_m == 12.5
        assert [o.candidate_id for o in out.reported_objects] == [1, 2, 3]
        assert list(out.corridor) == list(first.corridor)

        rejected = [
            (analysis('source-a', 1, 101), 'DUPLICATE_OR_LATE_SEQUENCE'),
            (analysis('source-a', 2, 99), 'STAMP_WENT_BACKWARDS'),
            (analysis('', 2, 101), 'INVALID_SOURCE_ID'),
            (analysis('source-b', 0, 101), 'INVALID_SEQUENCE'),
        ]
        invalid_stamp = analysis('source-b', 5, 0)
        rejected.append((invalid_stamp, 'INVALID_STAMP'))
        for count, (msg, reason) in enumerate(rejected, start=1):
            out = send_rejected(msg)
            assert out.rejected_analyses == count, (reason, out.rejected_analyses)
            assert out.last_rejection_reason == reason, (reason, out.last_rejection_reason)
            # The last valid result is kept untouched.
            assert out.source_instance_id == 'source-a' and out.frame_sequence == 1
            assert out.state == PathAssessment.OBSTACLE and out.distance_m == 12.5

        replacement = analysis('source-b', 1, 50)
        out = send(replacement)
        assert out.source_instance_id == 'source-b'
        assert out.state == PathAssessment.NO_OBSTACLE_DETECTED, out.reason
        assert not out.reported_objects and out.corridor
        out = send_rejected(analysis('source-a', 2, 102, (5.0,), session=3))
        assert out.last_rejection_reason == 'RETIRED_SOURCE'
        assert out.source_instance_id == 'source-b'
        assert out.state == PathAssessment.NO_OBSTACLE_DETECTED

        obstacle = analysis('source-b', 2, 51, (8.0,))
        out = send(obstacle)
        assert out.state == PathAssessment.OBSTACLE and out.distance_m == 8.0
        spin_until(lambda: latest().stale, seconds=8)
        stale = latest()
        assert stale.reason == 'INPUT_PAUSED_OR_STOPPED'
        assert stale.state == PathAssessment.UNKNOWN
        assert stale.header.stamp == obstacle.header.stamp
        assert (stale.source_instance_id, stale.frame_sequence) == ('source-b', 2)
        assert not stale.distance_valid and math.isnan(stale.distance_m)
        assert not stale.reported_objects and not stale.corridor
        assert not stale.evaluation_region_valid and stale.result_age_ms > 3000

        fresh = analysis('source-b', 3, 52, (7.5,))
        out = send(fresh)
        assert not out.stale and out.state == PathAssessment.OBSTACLE
        assert out.distance_m == 7.5 and len(out.reported_objects) == 1
        print('PASS: monitor gating, rejection diagnostics, retired source, timeout recovery')
    finally:
        if monitor.poll() is None:
            monitor.send_signal(signal.SIGINT)
        try:
            monitor.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(monitor.pid, signal.SIGKILL)
            monitor.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
