import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch VLM control with conservative speeds for tight spaces/indoor runs."""
    package_dir = get_package_share_directory('robot')
    monitor = LaunchConfiguration('monitor', default='true')

    vlm_control = Node(
        package='robot',
        executable='vlm_control',
        name='vlm_control_ugv',
        output='screen',
        parameters=[{
            'linear_speed_constant': 0.15,
            'angular_speed_constant': 0.2,
        }],
    )

    vlm_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'vlm_monitor.py'),
        ),
        condition=IfCondition(monitor),
    )

    return LaunchDescription([
        DeclareLaunchArgument('monitor', default_value='true', description='Start the VLM rqt monitor (true/false)'),
        vlm_control,
        vlm_monitor,
    ])

