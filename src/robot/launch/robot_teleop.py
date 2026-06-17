from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch UGV driver, camera, and keyboard teleop for real robot control."""
    camera_source = LaunchConfiguration('camera_source', default='picam2')
    camera_width = ParameterValue(LaunchConfiguration('camera_width', default='640'), value_type=int)
    camera_height = ParameterValue(LaunchConfiguration('camera_height', default='480'), value_type=int)
    camera_fps = ParameterValue(LaunchConfiguration('camera_fps', default='5'), value_type=int)
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera')
    camera_encoding = LaunchConfiguration('camera_encoding', default='rgb8')
    camera_show_image = ParameterValue(LaunchConfiguration('camera_show_image', default='false'), value_type=bool)
    camera_topic = LaunchConfiguration('camera_topic', default='/camera/image_color')

    ugv_uart = LaunchConfiguration('ugv_uart', default='/dev/ttyAMA0')
    ugv_baud = LaunchConfiguration('ugv_baud', default='115200')

    linear_speed = ParameterValue(
        LaunchConfiguration('linear_speed', default='0.1'), value_type=float
    )
    angular_speed = ParameterValue(
        LaunchConfiguration('angular_speed', default='0.1'), value_type=float
    )
    max_speed = ParameterValue(
        LaunchConfiguration('max_speed', default='0.5'), value_type=float
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='cmd_vel_teleop')

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
                'max_speed': max_speed,
                'cmd_vel_topic': cmd_vel_topic,
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
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel_teleop'),
        camera_node,
        ugv_driver,
        keyboard_teleop,
    ])
