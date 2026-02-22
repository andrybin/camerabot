from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_source = LaunchConfiguration('camera_source', default='picam2')
    camera_width = ParameterValue(LaunchConfiguration('camera_width', default='640'), value_type=int)
    camera_height = ParameterValue(LaunchConfiguration('camera_height', default='480'), value_type=int)
    camera_fps = ParameterValue(LaunchConfiguration('camera_fps', default='30'), value_type=int)
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera')
    camera_encoding = LaunchConfiguration('camera_encoding', default='rgb8')
    camera_show_image = ParameterValue(LaunchConfiguration('camera_show_image', default='false'), value_type=bool)
    camera_topic = LaunchConfiguration('camera_topic', default='/camera/image_color')

    ugv_uart = LaunchConfiguration('ugv_uart', default='/dev/ttyAMA0')
    ugv_baud = LaunchConfiguration('ugv_baud', default='115200')

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
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_source', default_value='picam2'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('camera_encoding', default_value='rgb8'),
        DeclareLaunchArgument('camera_show_image', default_value='false'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_color'),
        DeclareLaunchArgument('ugv_uart', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('ugv_baud', default_value='115200'),
        camera_node,
        ugv_driver,
    ])
