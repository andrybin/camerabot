from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch keyboard teleop and rqt for real robot control."""
    keyboard_teleop = Node(
        package='robot',
        executable='keyboard_teleop',
        output='screen',
    )

    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        remappings=[('image', '/camera/image_color')],
        output='screen',
    )

    return LaunchDescription([
        keyboard_teleop,
        rqt_image_view,
    ])
