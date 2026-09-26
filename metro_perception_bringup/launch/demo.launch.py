"""Detector with the RViz display chain; start bag playback separately after launch is up."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('metro_perception_bringup')
    names = ('sensor_profile', 'sensor_frame_override', 'publish_sensor_tf', 'namespace',
             'input_topic', 'fixed_frame', 'rviz', 'use_sim_time')
    defaults = {'sensor_profile': '', 'sensor_frame_override': '', 'publish_sensor_tf': 'false',
                'namespace': 'metro', 'input_topic': '/lidar_points',
                'fixed_frame': 'lidar_assumed', 'rviz': 'true', 'use_sim_time': 'false'}
    arguments = {name: LaunchConfiguration(name) for name in names}
    arguments['publish_bound_transform'] = 'true'  # Other RViz displays may show the raw input.
    arguments['visualizer'] = 'true'  # Markers also without RViz, as the smokes expect.
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=defaults[name]) for name in names),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, 'launch', 'perception.launch.py'])), launch_arguments=arguments.items()),
    ])
