"""Check range visualization without modifying source point clouds."""

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from metro_perception_tools.depth_image import distance_cloud


@pytest.mark.parametrize('bigendian', [False, True])
def test_distance_cloud_with_row_padding(bigendian):
    data = bytearray(80)
    xyz = np.ndarray((2, 2, 3), dtype='>f4' if bigendian else '<f4',
                     buffer=data, strides=(40, 16, 4))
    xyz[:] = [[[3, 4, 0], [0, 0, -12]], [[1, 2, 2], [0, 0, 0]]]
    source = PointCloud2(
        height=2, width=2, point_step=16, row_step=40,
        is_bigendian=bigendian, data=bytes(data),
        fields=[PointField(name=name, offset=i * 4,
                           datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(('x', 'y', 'z'))],
    )
    source.header.frame_id = 'lidar'
    source.header.stamp.sec = 123
    result = distance_cloud(source)
    points = np.frombuffer(result.data, dtype='<f4').reshape(2, 2, 4)
    np.testing.assert_array_equal(points[..., :3], xyz)
    np.testing.assert_allclose(points[..., 3], [[5, 12], [3, 0]])
    assert result.header == source.header
    assert result.row_step == 32
    assert result.is_dense
    assert bytes(source.data) == bytes(data)
    assert [f.name for f in result.fields] == ['x', 'y', 'z', 'distance']


def test_empty_distance_cloud():
    source = PointCloud2(height=1, width=0, point_step=12, fields=[
        PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(('x', 'y', 'z'))
    ])
    assert not distance_cloud(source).data
