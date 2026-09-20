from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Path absolut ke model TANPA ekstensi, mis. /home/user/models/best'
        ),
        DeclareLaunchArgument('class_mapping_path', default_value=''),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('debug_view', default_value='false'),
        DeclareLaunchArgument('telemetry_enabled', default_value='false'),
        DeclareLaunchArgument('capture_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('frame_buffer_size', default_value='2'),

        Node(
            package='vision_avoidance_ros2',
            executable='vision_node',
            name='vision_avoidance_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'class_mapping_path': LaunchConfiguration('class_mapping_path'),
                'camera_index': LaunchConfiguration('camera_index'),
                'debug_view': LaunchConfiguration('debug_view'),
                'telemetry_enabled': LaunchConfiguration('telemetry_enabled'),
                'capture_rate_hz': LaunchConfiguration('capture_rate_hz'),
                'frame_buffer_size': LaunchConfiguration('frame_buffer_size'),
            }],
        ),
    ])
