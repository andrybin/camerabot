import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
# Python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy


# QoS Profile
qos = QoSProfile(
    depth=1,  # Keep only the last message
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
)


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters
        self.declare_parameter('source', '0')  # camera index or URL
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 3)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('encoding', 'bgr8')
        self.declare_parameter('show_image', False)
        self.declare_parameter('topic', '/camera/image_color')

        source_param = self.get_parameter('source').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, topic, qos)
        self.bridge = CvBridge()

        # Parse source as int if possible
        self.capture_source = self._parse_source(source_param)

        self.cap = cv2.VideoCapture(self.capture_source)

        # Apply capture properties when possible
        if width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        if height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        if fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, float(fps))

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera source: {source_param}')

        period = 1.0 / float(max(1, fps))
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'CameraPublisher started: source={source_param}, size={width}x{height}, '
            f'fps={fps}, topic={topic}, encoding={self.encoding}, show_image={self.show_image}'
        )

    def _parse_source(self, source: str):
        # If numeric string, return int index; otherwise assume URL/path
        try:
            return int(source)
        except ValueError:
            return source

    def timer_callback(self):
        if not self.cap or not self.cap.isOpened():
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Failed to grab frame')
            return
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.get_logger().info(f'Publishing image')
        self.publisher_.publish(msg)

        if self.show_image:
            try:
                cv2.imshow('camera', frame)
                cv2.waitKey(1)
            except Exception:
                # Ignore display errors on headless systems
                pass

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
