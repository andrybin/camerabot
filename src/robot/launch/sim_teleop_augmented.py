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
        max_speed = ParameterValue(
            LaunchConfiguration('max_speed', default='2'), value_type=float
        )
        period = ParameterValue(
            LaunchConfiguration('period', default='3.0'), value_type=float
        )
        turn_value = ParameterValue(
            LaunchConfiguration('turn_value', default='1.0'), value_type=float
        )
        duration = ParameterValue(
            LaunchConfiguration('duration', default='1.0'), value_type=float
        )
        input_cmd_vel_topic = context.perform_substitution(
            LaunchConfiguration('input_cmd_vel_topic')
        )
        output_cmd_vel_topic = context.perform_substitution(
            LaunchConfiguration('output_cmd_vel_topic')
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
            remappings=[
                ('cmd_vel', input_cmd_vel_topic),
            ],
        )

        vel_augmenter = Node(
            package='robot',
            executable='vel_augmenter',
            parameters=[
                {
                    'input_cmd_vel_topic': input_cmd_vel_topic,
                    'output_cmd_vel_topic': output_cmd_vel_topic,
                    'period': period,
                    'turn_value': turn_value,
                    'duration': duration,
                }
            ],
        )

        return [
            webots,
            webots_driver,
            keyboard_teleop,
            vel_augmenter,
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
                'max_speed',
                default_value='2',
                description='Keyboard teleop maximum speed.',
            ),
            DeclareLaunchArgument(
                'input_cmd_vel_topic',
                default_value='cmd_vel_teleop',
                description='Raw keyboard teleop Twist topic (input to vel_augmenter).',
            ),
            DeclareLaunchArgument(
                'output_cmd_vel_topic',
                default_value='cmd_vel',
                description='Augmented Twist topic consumed by the Webots driver.',
            ),
            DeclareLaunchArgument(
                'period',
                default_value='3.0',
                description='Interval between random turn injections (seconds).',
            ),
            DeclareLaunchArgument(
                'turn_value',
                default_value='1',
                description='Random turn magnitude added to angular.z (rad/s).',
            ),
            DeclareLaunchArgument(
                'duration',
                default_value='1.0',
                description='How long each random turn lasts (seconds).',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
