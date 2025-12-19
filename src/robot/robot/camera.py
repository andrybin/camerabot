import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
# Python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy


# QoS Profile
qos = QoSProfile(
    depth=1,  # Keep only the last message
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
)


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters
        self.declare_parameter('source', 'picam2')  # camera index or URL or [picam2 libcamera rpicam]
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('encoding', 'rgb8')
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

        # Backend selection: Picamera2 or OpenCV capture
        self.use_picam2 = False
        self.picam2 = None
        self.cap = None

        src_str = str(self.capture_source).lower() if not isinstance(self.capture_source, int) else ''
        print(f"src_str: {self.capture_source}")
        if src_str in ('picam2', 'libcamera', 'rpicam'):
            # Picamera2 backend (Raspberry Pi)
            try:
                from picamera2 import Picamera2
                self.picam2 = Picamera2()
                config = self.picam2.create_video_configuration(
                    main={
                        "size": (int(width), int(height)),
                        "format": "RGB888",
                    },
                    controls={"FrameRate": int(fps)}
                )
                self.picam2.configure(config)
                try:
                    self.picam2.set_controls({"FrameRate": int(fps)})
                except Exception:
                    pass
                self.picam2.start()
                self.use_picam2 = True
            except Exception as ex:
                self.get_logger().error(f"Picamera2 initialization failed: {ex}")
                self.use_picam2 = False

        if not self.use_picam2:
            # Choose capture backend based on source
            if isinstance(self.capture_source, int):
                self.cap = cv2.VideoCapture(self.capture_source)
            else:
                src = str(self.capture_source)
                if src.startswith('http'):
                    # HTTP MJPEG via FFmpeg backend with tiny buffer
                    self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
                    try:
                        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                elif src.startswith('udp://'):
                    # Raw H.264 over UDP from rpicam-vid (not RTP)
                    try:
                        port = int(src.rsplit(':', 1)[-1])
                    except Exception:
                        port = 8554
                    pipeline = (
                        f"udpsrc port={port} caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
                        "queue max-size-buffers=1 leaky=downstream ! "
                        "h264parse disable-passthrough=true ! "
                        "avdec_h264 skip-frame=nonref ! "
                        "videoconvert ! appsink drop=true max-buffers=1 sync=false"
                    )
                    self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                else:
                    # Fallback to FFmpeg with small buffer
                    self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
                    try:
                        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass

            # Apply capture properties when possible
            if self.cap is not None:
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

        backend = 'picam2' if self.use_picam2 else ('opencv' if self.cap is not None else 'none')
        self.get_logger().info(
            f'CameraPublisher started: backend={backend}, source={source_param}, size={width}x{height}, '
            f'fps={fps}, topic={topic}, encoding={self.encoding}, show_image={self.show_image}'
        )

    def _parse_source(self, source: str):
        # If numeric string, return int index; otherwise assume URL/path
        try:
            return int(source)
        except ValueError:
            return source

    def timer_callback(self):
        frame = None
        if self.use_picam2 and self.picam2 is not None:
            try:
                frame = self.picam2.capture_array()
                ok = frame is not None
                if ok:
                    self.get_logger().debug(f'Got frame {frame.shape} from picamera')
            except Exception:
                ok = False
        else:
            if not self.cap or not self.cap.isOpened():
                return
            # Drop backlog then retrieve freshest frame
            try:
                for _ in range(2):
                    self.cap.grab()
                ok, frame = self.cap.retrieve()
            except Exception:
                ok, frame = self.cap.read()

        if not ok or frame is None:
            self.get_logger().warn('Failed to grab frame')
            return

        if self.use_picam2:
            try:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except Exception:
                pass

        msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher_.publish(msg)

        if self.show_image:
            try:
                cv2.imshow('camera', frame)
                cv2.waitKey(1)
            except Exception:
                pass

    def destroy_node(self):
        if getattr(self, 'use_picam2', False) and getattr(self, 'picam2', None) is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
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
