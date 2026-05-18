from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_dir = LaunchConfiguration('output_dir')
    image_topic = LaunchConfiguration('image_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    jpeg_quality = ParameterValue(
        LaunchConfiguration('jpeg_quality', default='92'), value_type=int
    )
    max_lin_speed = ParameterValue(
        LaunchConfiguration('max_lin_speed', default='0.5'), value_type=float
    )
    max_ang_speed = ParameterValue(
        LaunchConfiguration('max_ang_speed', default='0.5'), value_type=float
    )
    past_frames = ParameterValue(
        LaunchConfiguration('past_frames', default='0'), value_type=int
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'output_dir',
                default_value='behaviour_recordings',
                description='Directory for saved JPEGs (relative to cwd if not absolute).',
            ),
            DeclareLaunchArgument(
                'image_topic',
                default_value='/camera/image_color',
                description=(
                    'Camera image topic. Override if Webots uses a different name '
                    '(see ros2 topic list).'
                ),
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='cmd_vel',
                description='Twist command topic to pair with images.',
            ),
            DeclareLaunchArgument(
                'jpeg_quality',
                default_value='92',
                description='JPEG quality 0–100 for saved images.',
            ),
            DeclareLaunchArgument(
                'max_lin_speed',
                default_value='0.5',
                description='Maximum linear speed for filename scaling.',
            ),
            DeclareLaunchArgument(
                'max_ang_speed',
                default_value='0.5',
                description='Maximum angular speed for filename scaling.',
            ),
            DeclareLaunchArgument(
                'past_frames',
                default_value='0',
                description=(
                    'Ring buffer size: on each save, zip this many prior frames into '
                    '<timestamp>.zip beside the JPEG (0 disables).'
                ),
            ),
            Node(
                package='robot',
                executable='behaviour_recorder',
                output='screen',
                parameters=[
                    {
                        'output_dir': output_dir,
                        'image_topic': image_topic,
                        'cmd_vel_topic': cmd_vel_topic,
                        'jpeg_quality': jpeg_quality,
                        'max_lin_speed': max_lin_speed,
                        'max_ang_speed': max_ang_speed,
                        'past_frames': past_frames,
                    }
                ],
            ),
        ]
    )
