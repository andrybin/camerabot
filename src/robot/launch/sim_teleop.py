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
            LaunchConfiguration('linear_speed', default='2'), value_type=float
        )
        angular_speed = ParameterValue(
            LaunchConfiguration('angular_speed', default='2'), value_type=float
        )
        max_lin_speed = ParameterValue(
            LaunchConfiguration('max_lin_speed', default='2'), value_type=float
        )
        max_ang_speed = ParameterValue(
            LaunchConfiguration('max_ang_speed', default='2'), value_type=float
        )
        command_stamp_offset_ms = ParameterValue(
            LaunchConfiguration('command_stamp_offset_ms', default='0'), value_type=float
        )
        teleop_cmd_vel_topic = context.perform_substitution(
            LaunchConfiguration('teleop_cmd_vel_topic')
        )
        cmd_vel_topic = context.perform_substitution(
            LaunchConfiguration('cmd_vel_topic')
        )

        webots = WebotsLauncher(world=world)

        webots_driver = WebotsController(
            robot_name='my_robot',
            parameters=[
                {'robot_description': robot_description_path},
            ],
            remappings=[
                ('cmd_vel', cmd_vel_topic),
            ],
        )

        keyboard_teleop = Node(
            package='robot',
            executable='keyboard_teleop',
            parameters=[
                {
                    'linear_speed': linear_speed,
                    'angular_speed': angular_speed,
                    'max_lin_speed': max_lin_speed,
                    'max_ang_speed': max_ang_speed,
                    'cmd_vel_topic': teleop_cmd_vel_topic,
                    'command_stamp_offset_ms': command_stamp_offset_ms,
                }
            ],
        )

        cmd_vel_relay = Node(
            package='robot',
            executable='vel_augmenter',
            parameters=[
                {
                    'input_cmd_vel_topic': teleop_cmd_vel_topic,
                    'output_cmd_vel_topic': cmd_vel_topic,
                    'period': 0.0,
                    'turn_value': 0.0,
                    'duration': 0.0,
                }
            ],
        )

        return [
            webots,
            webots_driver,
            keyboard_teleop,
            cmd_vel_relay,
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
                default_value='2',
                description='Keyboard teleop linear speed step (m/s).',
            ),
            DeclareLaunchArgument(
                'angular_speed',
                default_value='2',
                description='Keyboard teleop angular speed step (rad/s).',
            ),
            DeclareLaunchArgument(
                'max_lin_speed',
                default_value='2',
                description='Keyboard teleop maximum linear speed (m/s).',
            ),
            DeclareLaunchArgument(
                'max_ang_speed',
                default_value='2',
                description='Keyboard teleop maximum angular speed (rad/s).',
            ),
            DeclareLaunchArgument(
                'teleop_cmd_vel_topic',
                default_value='cmd_vel_teleop',
                description='Keyboard teleop Twist topic (subscribe here for behaviour_recorder).',
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='cmd_vel',
                description='Twist topic consumed by the Webots driver.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
