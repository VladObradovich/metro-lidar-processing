from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('fixed_frame', default_value='hesai_lidar'),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('output_topic', default_value='/lidar/depth_image'),
        DeclareLaunchArgument('distance_topic', default_value='/lidar/distance_points'),
        DeclareLaunchArgument('image_width', default_value='320'),
        DeclareLaunchArgument('image_height', default_value='128'),
        DeclareLaunchArgument('min_depth', default_value='1.0'),
        DeclareLaunchArgument('max_depth', default_value='300.0'),
        DeclareLaunchArgument('min_azimuth_deg', default_value='-140.0'),
        DeclareLaunchArgument('max_azimuth_deg', default_value='-40.0'),
        DeclareLaunchArgument('histogram_equalization', default_value='true'),
        DeclareLaunchArgument('point_stride', default_value='1'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('video_fps', default_value='10.0'),
    ]
    node = Node(
        package='metro_perception_tools',
        executable='depth_image',
        name='lidar_depth_image',
        output='screen',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'distance_topic': LaunchConfiguration('distance_topic'),
                'image_width': ParameterValue(
                    LaunchConfiguration('image_width'), value_type=int
                ),
                'image_height': ParameterValue(
                    LaunchConfiguration('image_height'), value_type=int
                ),
                'min_depth': ParameterValue(
                    LaunchConfiguration('min_depth'), value_type=float
                ),
                'max_depth': ParameterValue(
                    LaunchConfiguration('max_depth'), value_type=float
                ),
                'min_azimuth_deg': ParameterValue(
                    LaunchConfiguration('min_azimuth_deg'), value_type=float
                ),
                'max_azimuth_deg': ParameterValue(
                    LaunchConfiguration('max_azimuth_deg'), value_type=float
                ),
                'histogram_equalization': ParameterValue(
                    LaunchConfiguration('histogram_equalization'), value_type=bool
                ),
                'point_stride': ParameterValue(
                    LaunchConfiguration('point_stride'), value_type=int
                ),
                'video_path': LaunchConfiguration('video_path'),
                'video_fps': ParameterValue(
                    LaunchConfiguration('video_fps'), value_type=float
                ),
            }
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('metro_perception_bringup'), 'rviz', 'depth_image.rviz'
            ]),
            '-f', LaunchConfiguration('fixed_frame'),
        ],
        remappings=[
            ('/metro_rviz/input', LaunchConfiguration('distance_topic')),
            ('/metro_rviz/image', LaunchConfiguration('output_topic')),
        ],
        output='screen',
    )
    return LaunchDescription(arguments + [node, rviz])
