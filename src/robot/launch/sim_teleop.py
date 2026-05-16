import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    linear_speed = ParameterValue(
        LaunchConfiguration('linear_speed', default='0.1'), value_type=float
    )
    angular_speed = ParameterValue(
        LaunchConfiguration('angular_speed', default='0.1'), value_type=float
    )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    webots_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    keyboard_teleop = Node(
        package='robot',
        executable='keyboard_teleop',
        parameters=[{
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.1',
            description='Keyboard teleop linear speed step (m/s).',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.1',
            description='Keyboard teleop angular speed step (rad/s).',
        ),
        webots,
        webots_driver,
        keyboard_teleop,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
