"""Inspect rosbag metadata and a bounded sample of PointCloud2 layouts."""
import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--sample-count', type=int, default=1)
    args = parser.parse_args()
    if args.sample_count < 1:
        parser.error('--sample-count must be positive')
    metadata = yaml.safe_load((args.bag / 'metadata.yaml').read_text())
    info = metadata['rosbag2_bagfile_information']
    topics = [item['topic_metadata']['name'] for item in info['topics_with_message_count']
              if item['topic_metadata']['type'] == 'sensor_msgs/msg/PointCloud2']
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(args.bag),
            storage_id=info['storage_identifier'],
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    if not topics:
        parser.error('No PointCloud2 topics')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    samples = {topic: [] for topic in topics}
    while reader.has_next() and any(len(rows) < args.sample_count for rows in samples.values()):
        topic, data, bag_stamp = reader.read_next()
        if len(samples[topic]) >= args.sample_count:
            continue
        cloud = deserialize_message(data, PointCloud2)
        samples[topic].append({
            'bag_stamp_ns': bag_stamp,
            'header_stamp_ns': cloud.header.stamp.sec * 10**9 + cloud.header.stamp.nanosec,
            'frame_id': cloud.header.frame_id, 'width': cloud.width, 'height': cloud.height,
            'point_step': cloud.point_step, 'row_step': cloud.row_step,
            'is_bigendian': cloud.is_bigendian, 'data_bytes': len(cloud.data),
            'fields': [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype,
                        'count': f.count} for f in cloud.fields],
        })
    result = {'schema_version': 1, 'bag': str(args.bag), 'metadata': info,
              'scope': 'metadata_and_first_samples_only', 'samples': samples}
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        with args.output.open('x') as output:
            output.write(text)
    else:
        print(text, end='')
