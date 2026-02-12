from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch VLM control with higher speeds for open areas/simulation."""
    model = LaunchConfiguration('model', default='qwen2.5vl:latest')
    vlm_control = Node(
        package='robot',
        executable='vlm_control',
        name='vlm_control_fast',
        output='screen',
        parameters=[{
            'model': model,
            'linear_speed_constant': 1.0,
            'angular_speed_constant': 0.3,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='qwen2.5vl:latest', description='VLM model name (e.g. qwen2.5vl:latest)'),
        vlm_control,
    ])

