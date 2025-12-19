from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch VLM control with higher speeds for open areas/simulation."""
    vlm_control = Node(
        package='robot',
        executable='vlm_control',
        name='vlm_control_fast',
        output='screen',
        parameters=[{
            'linear_speed_constant': 1.0,
            'angular_speed_constant': 0.3,
        }],
    )

    return LaunchDescription([vlm_control])

