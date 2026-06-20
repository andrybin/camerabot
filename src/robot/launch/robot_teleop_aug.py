from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch UGV driver, camera, keyboard teleop, and velocity augmenter for real robot."""
    camera_source = LaunchConfiguration('camera_source')
    camera_width = ParameterValue(LaunchConfiguration('camera_width'), value_type=int)
    camera_height = ParameterValue(LaunchConfiguration('camera_height'), value_type=int)
    camera_fps = ParameterValue(LaunchConfiguration('camera_fps'), value_type=int)
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    camera_encoding = LaunchConfiguration('camera_encoding')
    camera_show_image = ParameterValue(LaunchConfiguration('camera_show_image'), value_type=bool)
    camera_topic = LaunchConfiguration('camera_topic')

    ugv_uart = LaunchConfiguration('ugv_uart')
    ugv_baud = LaunchConfiguration('ugv_baud')

    linear_speed = ParameterValue(
        LaunchConfiguration('linear_speed'), value_type=float
    )
    angular_speed = ParameterValue(
        LaunchConfiguration('angular_speed'), value_type=float
    )
    max_lin_speed = ParameterValue(
        LaunchConfiguration('max_lin_speed'), value_type=float
    )
    max_ang_speed = ParameterValue(
        LaunchConfiguration('max_ang_speed'), value_type=float
    )
    command_stamp_offset_ms = ParameterValue(
        LaunchConfiguration('command_stamp_offset_ms'), value_type=float
    )
    period = ParameterValue(
        LaunchConfiguration('period'), value_type=float
    )
    turn_value = ParameterValue(
        LaunchConfiguration('turn_value'), value_type=float
    )
    duration = ParameterValue(
        LaunchConfiguration('duration'), value_type=float
    )
    teleop_cmd_vel_topic = LaunchConfiguration('teleop_cmd_vel_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    camera_node = Node(
        package='robot',
        executable='camera',
        name='camera',
        output='screen',
        parameters=[{
            'source': camera_source,
            'width': camera_width,
            'height': camera_height,
            'fps': camera_fps,
            'frame_id': camera_frame_id,
            'encoding': camera_encoding,
            'show_image': camera_show_image,
            'topic': camera_topic,
        }],
    )

    ugv_driver = Node(
        package='robot',
        executable='ugv_driver',
        name='ugv_driver',
        output='screen',
        additional_env={
            'UGV_UART': ugv_uart,
            'UGV_BAUD': ugv_baud,
        },
        remappings=[
            ('cmd_vel', cmd_vel_topic),
        ],
    )

    keyboard_teleop = Node(
        package='robot',
        executable='keyboard_teleop',
        output='screen',
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

    vel_augmenter = Node(
        package='robot',
        executable='vel_augmenter',
        parameters=[
            {
                'input_cmd_vel_topic': teleop_cmd_vel_topic,
                'output_cmd_vel_topic': cmd_vel_topic,
                'period': period,
                'turn_value': turn_value,
                'duration': duration,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_source', default_value='picam2'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='5'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('camera_encoding', default_value='rgb8'),
        DeclareLaunchArgument('camera_show_image', default_value='false'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_color'),
        DeclareLaunchArgument('ugv_uart', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('ugv_baud', default_value='115200'),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.15',
            description='Keyboard teleop linear speed step (m/s).',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.2',
            description='Keyboard teleop angular speed step (rad/s).',
        ),
        DeclareLaunchArgument(
            'max_lin_speed',
            default_value='0.15',
            description='Keyboard teleop maximum linear speed (m/s).',
        ),
        DeclareLaunchArgument(
            'max_ang_speed',
            default_value='0.2',
            description='Keyboard teleop maximum angular speed (rad/s).',
        ),
        DeclareLaunchArgument(
            'command_stamp_offset_ms',
            default_value='-100',
            description=(
                'Added to cmd header.stamp (ms) to compensate SSH/network delay '
                'when teleop runs off-robot. Use 0 for local/sim teleop.'
            ),
        ),
        DeclareLaunchArgument(
            'teleop_cmd_vel_topic',
            default_value='cmd_vel_teleop',
            description='Keyboard teleop Twist topic (subscribe here for behaviour_recorder).',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='Augmented Twist topic consumed by the UGV driver.',
        ),
        DeclareLaunchArgument(
            'period',
            default_value='4.0',
            description='Interval between random turn injections (seconds).',
        ),
        DeclareLaunchArgument(
            'turn_value',
            default_value='0.2',
            description='Random turn magnitude added to angular.z (rad/s).',
        ),
        DeclareLaunchArgument(
            'duration',
            default_value='0.5',
            description='How long each random turn lasts (seconds).',
        ),
        camera_node,
        ugv_driver,
        keyboard_teleop,
        vel_augmenter,
    ])
