"""Launch perception and monitor; optional sensor TF publication belongs to bringup."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _enabled(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _sensor_tf(context):
    if not _enabled(LaunchConfiguration('publish_sensor_tf').perform(context)):
        return []

    profile = LaunchConfiguration('sensor_profile').perform(context)
    if not profile:
        profile = str(Path(get_package_share_directory('metro_perception_ros')) /
                      'config' / 'forward_sector_assumed.yaml')

    data = yaml.safe_load(Path(profile).read_text())
    translation = data.get('translation_m')
    rotation = data.get('rotation_rpy_rad')
    source = data.get('source_frame')
    target = data.get('target_frame')

    if translation is None and rotation is None:
        return []
    if translation is None or rotation is None:
        raise RuntimeError('sensor profile must define both translation_m and rotation_rpy_rad')
    if not source or not target or source == '*' or source == target:
        raise RuntimeError('publish_sensor_tf requires distinct explicit source_frame/target_frame')

    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_profile_static_tf',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '--x', str(translation[0]), '--y', str(translation[1]), '--z', str(translation[2]),
            '--roll', str(rotation[0]), '--pitch', str(rotation[1]), '--yaw', str(rotation[2]),
            '--frame-id', target, '--child-frame-id', source,
        ],
        output='screen',
    )]


def generate_launch_description():
    share = FindPackageShare('metro_perception_bringup')
    namespace = LaunchConfiguration('namespace')
    params = [LaunchConfiguration('algorithm_config'), LaunchConfiguration('runtime_config'),
              {'use_sim_time': LaunchConfiguration('use_sim_time'),
               'sensor_profile': LaunchConfiguration('sensor_profile')}]
    return LaunchDescription([
        DeclareLaunchArgument('sensor_profile', default_value=''),
        DeclareLaunchArgument('publish_sensor_tf', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='metro'),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('algorithm_config', default_value=PathJoinSubstitution(
            [share, 'config', 'algorithm.yaml'])),
        DeclareLaunchArgument('runtime_config', default_value=PathJoinSubstitution(
            [share, 'config', 'runtime.yaml'])),
        OpaqueFunction(function=_sensor_tf),
        Node(package='metro_perception_ros', executable='perception_node', name='perception',
             namespace=namespace, parameters=params, output='screen', remappings=[
                 ('~/input/points', LaunchConfiguration('input_topic')),
                 ('~/output/analysis', 'analysis')]),
        Node(package='metro_perception_ros', executable='obstacle_monitor_node',
             name='obstacle_monitor', namespace=namespace, parameters=params, output='screen',
             remappings=[('~/input/analysis', 'analysis'), ('~/output/assessment', 'assessment')]),
    ])
