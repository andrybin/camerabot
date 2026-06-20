from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch UGV driver, camera, and behaviour-clone policy for real robot navigation."""
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

    weights_path = LaunchConfiguration('weights_path')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    device = LaunchConfiguration('device')

    max_lin_speed = ParameterValue(
        LaunchConfiguration('max_lin_speed'), value_type=float
    )
    max_ang_speed = ParameterValue(
        LaunchConfiguration('max_ang_speed'), value_type=float
    )
    num_past = ParameterValue(LaunchConfiguration('num_past'), value_type=int)
    img_width = ParameterValue(LaunchConfiguration('img_width'), value_type=int)
    img_height = ParameterValue(LaunchConfiguration('img_height'), value_type=int)
    publish_rate_hz = ParameterValue(
        LaunchConfiguration('publish_rate_hz'), value_type=float
    )

    camera_node = Node(
        package='robot',
        executable='camera',
        name='camera',
        output='screen',
        parameters=[
            {
                'source': camera_source,
                'width': camera_width,
                'height': camera_height,
                'fps': camera_fps,
                'frame_id': camera_frame_id,
                'encoding': camera_encoding,
                'show_image': camera_show_image,
                'topic': camera_topic,
            }
        ],
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

    return LaunchDescription(
        [
            DeclareLaunchArgument('camera_source', default_value='picam2'),
            DeclareLaunchArgument('camera_width', default_value='640'),
            DeclareLaunchArgument('camera_height', default_value='480'),
            DeclareLaunchArgument('camera_fps', default_value='5'),
            DeclareLaunchArgument('camera_frame_id', default_value='camera'),
            DeclareLaunchArgument('camera_encoding', default_value='rgb8'),
            DeclareLaunchArgument('camera_show_image', default_value='false'),
            DeclareLaunchArgument('ugv_uart', default_value='/dev/ttyAMA0'),
            DeclareLaunchArgument('ugv_baud', default_value='115200'),
            DeclareLaunchArgument(
                'weights_path',
                default_value='behavclon/model.onnx',
                description=(
                    'Path to behaviour clone ONNX model (export with behavclon/export.py). Required.'
                ),
            ),
            DeclareLaunchArgument(
                'camera_topic',
                default_value='/camera/image_color',
                description='Camera image topic from the robot camera node.',
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='cmd_vel',
                description='Twist topic for the policy output (subscribed by ugv_driver).',
            ),
            DeclareLaunchArgument(
                'max_lin_speed',
                default_value='0.15',
                description='Max linear speed (m/s); must match training/recorder scaling.',
            ),
            DeclareLaunchArgument(
                'max_ang_speed',
                default_value='0.3',
                description='Max angular speed (rad/s); must match training/recorder scaling.',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='cpu',
                description='Torch device hint (ONNX runtime uses cpu/cuda providers).',
            ),
            DeclareLaunchArgument(
                'num_past',
                default_value='0',
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
            camera_node,
            ugv_driver,
            behaviour_control,
        ]
    )
