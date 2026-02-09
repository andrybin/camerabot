import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    model = LaunchConfiguration('model', default='qwen2.5vl:latest')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    webots_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    vlm_control_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'vlm_control_sim.py'),
        ),
        launch_arguments=[('model', model)],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='qwen2.5vl:latest', description='VLM model name (e.g. qwen2.5vl:latest)'),
        webots,
        webots_driver,
        vlm_control_sim,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ]) 