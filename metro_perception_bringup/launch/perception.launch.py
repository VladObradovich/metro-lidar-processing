"""
Launch perception and monitor; optional sensor TF publication belongs to bringup.

With rviz:=true (desktop image) it also starts the display chain: perception publishes the
analysed points labelled by corridor and obstacle, the depth_image node draws them as a grey
depth video with the corridor green and obstacles red, and RViz shows the video and the
assessment as text (metro_perception_rviz panel) on the left and the cloud with the corridor
lines on the right.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    source_mode = data.get('source_frame_mode', 'exact')
    target = data.get('target_frame')
    override = LaunchConfiguration('sensor_frame_override').perform(context).strip()

    if source == '*':
        source_mode = 'bind_first'
        source = None
    if source_mode == 'bind_first':
        if not override:
            raise RuntimeError(
                'publish_sensor_tf with bind_first profile requires sensor_frame_override'
            )
        source = override
    elif source_mode == 'exact':
        if override and override != source:
            raise RuntimeError('sensor_frame_override conflicts with exact source_frame')
    else:
        raise RuntimeError('source_frame_mode must be exact or bind_first')

    if translation is None and rotation is None:
        return []
    if translation is None or rotation is None:
        raise RuntimeError('sensor profile must define both translation_m and rotation_rpy_rad')
    if not source or not target or source == '*' or source == target:
        raise RuntimeError(
            'publish_sensor_tf requires distinct explicit source_frame/target_frame'
        )

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
    rviz = LaunchConfiguration('rviz')
    perception_params = params + [
        {'publish_bound_transform': LaunchConfiguration('publish_bound_transform'),
         'publish_labelled_cloud': rviz}]
    display = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    markers = PythonExpression(["'", rviz, "' == 'true' or '", LaunchConfiguration('visualizer'),
                                "' == 'true'"])
    return LaunchDescription([
        DeclareLaunchArgument('sensor_profile', default_value=''),
        DeclareLaunchArgument('publish_sensor_tf', default_value='false'),
        # Perception sends the profile transform of the bound input frame to /tf_static.
        DeclareLaunchArgument('publish_bound_transform', default_value='false'),
        DeclareLaunchArgument('sensor_frame_override', default_value=''),
        DeclareLaunchArgument('namespace', default_value='metro'),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('algorithm_config', default_value=PathJoinSubstitution(
            [share, 'config', 'algorithm.yaml'])),
        DeclareLaunchArgument('runtime_config', default_value=PathJoinSubstitution(
            [share, 'config', 'runtime.yaml'])),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start the depth video, markers and RViz'),
        DeclareLaunchArgument('visualizer', default_value='false',
                              description='Publish assessment markers without RViz'),
        DeclareLaunchArgument('fixed_frame', default_value='lidar_assumed',
                              description='RViz fixed frame: the profile target frame'),
        OpaqueFunction(function=_sensor_tf),
        Node(package='metro_perception_ros', executable='perception_node', name='perception',
             namespace=namespace, parameters=perception_params, output='screen', remappings=[
                 ('~/input/points', LaunchConfiguration('input_topic')),
                 ('~/output/analysis', 'analysis'),
                 ('~/output/labelled_points', 'labelled_points')]),
        Node(package='metro_perception_ros', executable='obstacle_monitor_node',
             name='obstacle_monitor', namespace=namespace, parameters=params, output='screen',
             remappings=[('~/input/analysis', 'analysis'), ('~/output/assessment', 'assessment')]),
        Node(package='metro_perception_ros', executable='visualizer_node', name='visualizer',
             namespace=namespace, output='screen', parameters=[display],
             condition=IfCondition(markers),
             remappings=[('~/input/assessment', 'assessment'), ('~/output/markers', 'markers'),
                         ('~/output/corridor_markers', 'corridor_markers')]),
        # The labelled cloud is in the target frame: forward is +x, the forward sector +-50 deg.
        Node(package='metro_perception_tools', executable='depth_image', name='depth_image',
             namespace=namespace, output='screen', condition=IfCondition(rviz),
             parameters=[display, {'input_topic': 'labelled_points',
                                   'output_topic': 'depth_image',
                                   'image_width': 480, 'image_height': 128,
                                   'min_depth': 1.0, 'max_depth': 150.0,
                                   'min_azimuth_deg': -50.0, 'max_azimuth_deg': 50.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz', namespace=namespace,
             condition=IfCondition(rviz), parameters=[display],
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'detector.rviz']),
                        '-f', LaunchConfiguration('fixed_frame')],
             remappings=[('/metro_rviz/cloud', 'labelled_points'),
                         ('/metro_rviz/depth', 'depth_image'),
                         ('/metro_rviz/assessment', 'assessment'),
                         ('/metro_rviz/corridor', 'corridor_markers'),
                         ('/metro_rviz/markers', 'markers')]),
    ])
