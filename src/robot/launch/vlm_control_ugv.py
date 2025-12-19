from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch VLM control with conservative speeds for tight spaces/indoor runs."""
    vlm_control = Node(
        package='robot',
        executable='vlm_control',
        name='vlm_control_safe',
        output='screen',
        parameters=[{
            'linear_speed_constant': 0.15,
            'angular_speed_constant': 0.2,
        }],
    )

    return LaunchDescription([vlm_control])

