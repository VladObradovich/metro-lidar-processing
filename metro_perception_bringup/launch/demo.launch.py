"""Scaffold demo; start bag playback separately after launch is up."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('metro_perception_bringup')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='metro'),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('fixed_frame', default_value='hesai_lidar'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, 'launch', 'perception.launch.py'])), launch_arguments={
                'namespace': namespace, 'input_topic': LaunchConfiguration('input_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        Node(package='metro_perception_ros', executable='visualizer_node',
             name='visualizer', namespace=namespace, output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('~/input/assessment', 'assessment'), ('~/output/markers', 'markers')]),
        Node(package='rviz2', executable='rviz2', namespace=namespace,
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'perception.rviz']),
                        '-f', LaunchConfiguration('fixed_frame')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('/metro_rviz/input', LaunchConfiguration('input_topic')),
                         ('/metro_rviz/markers', 'markers')]),
    ])
