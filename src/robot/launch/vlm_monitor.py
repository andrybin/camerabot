import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    perspective_path = os.path.join(package_dir, 'resource', 'vlm_monitor.perspective')

    rqt = ExecuteProcess(
        cmd=['rqt', '--perspective-file', perspective_path],
        output='screen',
    )

    return LaunchDescription([
        rqt,
    ])
