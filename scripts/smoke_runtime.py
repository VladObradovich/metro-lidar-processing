#!/usr/bin/env python3
"""Check an installed image against an external bag; run inside the container."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import rclpy
from sensor_msgs.msg import Image, PointCloud2


def stop(process):
    if process.poll() is None:
        # ros2 launch forwards the signal to its children itself.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise RuntimeError('Process did not stop after SIGINT')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--input-topic', default='/lidar_points')
    parser.add_argument('--min-azimuth', default='-140.0')
    parser.add_argument('--max-azimuth', default='-40.0')
    parser.add_argument('--video', default='/results/smoke.mp4')
    args = parser.parse_args()
    if Path(args.video).exists():
        parser.error('Choose a new video path; the test will not overwrite a file')

    rclpy.init()
    node = rclpy.create_node('runtime_smoke_check')
    images = []
    frames = set()

    def on_image(message):
        if message.width > 0 and message.height > 0 and any(message.data):
            images.append((message.width, message.height))

    node.create_subscription(Image, '/lidar/depth_image', on_image, 10)
    node.create_subscription(
        PointCloud2, args.input_topic,
        lambda message: frames.add(message.header.frame_id), 10,
    )
    launch = subprocess.Popen([
        'ros2', 'launch', 'metro_lidar_processing', 'depth_image.launch.py',
        f'input_topic:={args.input_topic}',
        f'min_azimuth_deg:={args.min_azimuth}',
        f'max_azimuth_deg:={args.max_azimuth}',
        f'video_path:={args.video}',
    ], start_new_session=True)
    player = None
    try:
        deadline = time.monotonic() + 30
        while node.count_subscribers(args.input_topic) < 2:
            if launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Processing node did not become ready')
            rclpy.spin_once(node, timeout_sec=0.1)
        player = subprocess.Popen([
            'ros2', 'bag', 'play', args.bag,
        ], stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.monotonic() + 45
        while len(images) < 5:
            if launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Did not receive five nonempty depth images')
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            if player is not None:
                stop(player)
        finally:
            stop(launch)
            node.destroy_node()
            rclpy.shutdown()

    video = cv2.VideoCapture(args.video)
    readable, frame = video.read()
    video.release()
    if not readable or frame is None:
        raise RuntimeError('Saved video cannot be decoded after shutdown')
    print(f'PASS: {len(images)} images, size={images[0]}, frames={sorted(frames)}, '
          f'video={args.video}', flush=True)


if __name__ == '__main__':
    main()
