import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    rviz_config_path = os.path.join(package_dir, 'resource', 'default.rviz')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    simple_control = Node(
        package='robot',
        executable='simple_control',
    )

    vlm_control = Node(
        package='robot',
        executable='vlm_control',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        rviz,
        # simple_control,
        vlm_control,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ]) 