import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    default_world = os.path.join(package_dir, 'worlds', 'track.wbt')

    def launch_setup(context, *args, **kwargs):
        robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

        world = context.perform_substitution(LaunchConfiguration('world'))
        if not os.path.isabs(world):
            world = os.path.join(package_dir, 'worlds', world)

        linear_speed = ParameterValue(
            LaunchConfiguration('linear_speed', default='0.1'), value_type=float
        )
        angular_speed = ParameterValue(
            LaunchConfiguration('angular_speed', default='0.1'), value_type=float
        )
        max_speed = ParameterValue(
            LaunchConfiguration('max_speed', default='0.5'), value_type=float
        )

        webots = WebotsLauncher(world=world)

        webots_driver = WebotsController(
            robot_name='my_robot',
            parameters=[
                {'robot_description': robot_description_path},
            ],
        )

        keyboard_teleop = Node(
            package='robot',
            executable='keyboard_teleop',
            parameters=[
                {
                    'linear_speed': linear_speed,
                    'angular_speed': angular_speed,
                    'max_speed': max_speed,
                }
            ],
        )

        return [
            webots,
            webots_driver,
            keyboard_teleop,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=default_world,
                description=(
                    'Webots world: absolute path to a .wbt file, or a filename '
                    "under this package's share/robot/worlds/ (e.g. track.wbt)."
                ),
            ),
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
            DeclareLaunchArgument(
                'max_speed',
                default_value='0.5',
                description='Keyboard teleop maximum speed.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
