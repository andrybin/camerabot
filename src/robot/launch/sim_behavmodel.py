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

        weights_path = context.perform_substitution(LaunchConfiguration('weights_path'))
        camera_topic = context.perform_substitution(LaunchConfiguration('camera_topic'))
        cmd_vel_topic = context.perform_substitution(LaunchConfiguration('cmd_vel_topic'))
        device = context.perform_substitution(LaunchConfiguration('device'))

        max_lin_speed = ParameterValue(
            LaunchConfiguration('max_lin_speed', default='0.5'), value_type=float
        )
        max_ang_speed = ParameterValue(
            LaunchConfiguration('max_ang_speed', default='0.5'), value_type=float
        )
        num_past = ParameterValue(LaunchConfiguration('num_past', default='4'), value_type=int)
        img_width = ParameterValue(LaunchConfiguration('img_width', default='128'), value_type=int)
        img_height = ParameterValue(LaunchConfiguration('img_height', default='64'), value_type=int)
        publish_rate_hz = ParameterValue(
            LaunchConfiguration('publish_rate_hz', default='0.0'), value_type=float
        )

        webots = WebotsLauncher(world=world)

        webots_driver = WebotsController(
            robot_name='my_robot',
            parameters=[
                {'robot_description': robot_description_path},
            ],
        )

        behaviour_control = Node(
            package='robot',
            executable='behaviour_control',
            output='screen',
            parameters=[
                {
                    'weights_path': weights_path,
                    'camera_topic': camera_topic,
                    'cmd_vel_topic': cmd_vel_topic,
                    'max_lin_speed': max_lin_speed,
                    'max_ang_speed': max_ang_speed,
                    'device': device,
                    'num_past': num_past,
                    'img_width': img_width,
                    'img_height': img_height,
                    'publish_rate_hz': publish_rate_hz,
                }
            ],
        )

        return [
            webots,
            webots_driver,
            behaviour_control,
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
                'weights_path',
                default_value='',
                description=(
                    'Path to behaviour clone ONNX model (export with behavclon/export.py). Required.'
                ),
            ),
            DeclareLaunchArgument(
                'camera_topic',
                default_value='/camera/image_color',
                description='Camera image topic (override if Webots uses another name).',
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='cmd_vel',
                description='Twist topic for the policy output.',
            ),
            DeclareLaunchArgument(
                'max_lin_speed',
                default_value='0.5',
                description='Max linear speed (m/s); must match training/recorder scaling.',
            ),
            DeclareLaunchArgument(
                'max_ang_speed',
                default_value='0.5',
                description='Max angular speed (rad/s); must match training/recorder scaling.',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='cpu',
                description='Torch device: cpu or cuda.',
            ),
            DeclareLaunchArgument(
                'num_past',
                default_value='4',
                description='Past-frame stack size (checkpoint usually overrides).',
            ),
            DeclareLaunchArgument(
                'img_width',
                default_value='128',
                description='Model input width in pixels (ONNX usually overrides).',
            ),
            DeclareLaunchArgument(
                'img_height',
                default_value='64',
                description='Model input height in pixels (ONNX usually overrides).',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value='0.0',
                description='If >0, publish cmd_vel at this rate; 0 = publish every camera frame.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
