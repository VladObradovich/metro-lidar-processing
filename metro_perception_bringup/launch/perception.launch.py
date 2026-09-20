"""Launch A02 and monitor. Default lidar-only profile allows assumed mounting."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('metro_perception_bringup')
    namespace = LaunchConfiguration('namespace')
    params = [LaunchConfiguration('algorithm_config'), LaunchConfiguration('runtime_config'),
              {'use_sim_time': LaunchConfiguration('use_sim_time'),
               'sensor_profile': LaunchConfiguration('sensor_profile')}]
    return LaunchDescription([
        DeclareLaunchArgument('sensor_profile', default_value=''),
        DeclareLaunchArgument('namespace', default_value='metro'),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('algorithm_config', default_value=PathJoinSubstitution(
            [share, 'config', 'algorithm.yaml'])),
        DeclareLaunchArgument('runtime_config', default_value=PathJoinSubstitution(
            [share, 'config', 'runtime.yaml'])),
        Node(package='metro_perception_ros', executable='perception_node', name='perception',
             namespace=namespace, parameters=params, output='screen', remappings=[
                 ('~/input/points', LaunchConfiguration('input_topic')),
                 ('~/output/analysis', 'analysis')]),
        Node(package='metro_perception_ros', executable='obstacle_monitor_node',
             name='obstacle_monitor', namespace=namespace, parameters=params, output='screen',
             remappings=[('~/input/analysis', 'analysis'), ('~/output/assessment', 'assessment')]),
    ])
