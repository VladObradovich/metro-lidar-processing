from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('output_topic', default_value='/lidar/depth_image'),
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
        package='metro_lidar_processing',
        executable='depth_image',
        name='lidar_depth_image',
        output='screen',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
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
    return LaunchDescription(arguments + [node])
