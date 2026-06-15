import os
import zipfile
from collections import deque
from copy import deepcopy
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image as RosImage

from behavclon.control_codec import encode_control_code

_IMAGE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
)


def _twist_components(twist: Twist):
    return (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


def _twists_close(a: Twist, b: Twist, eps: float) -> bool:
    for x, y in zip(_twist_components(a), _twist_components(b)):
        if abs(x - y) > eps:
            return False
    return True


def _image_stamp_ns(msg: RosImage, fallback_ns: int) -> int:
    """Nanoseconds from message header stamp, or fallback if stamp is unset."""
    stamp = msg.header.stamp
    if stamp.sec != 0 or stamp.nanosec != 0:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return int(fallback_ns)


class BehaviourRecorderNode(Node):
    def __init__(self):
        super().__init__('behaviour_recorder')

        self.declare_parameter('image_topic', '/camera/image_color')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('output_dir', 'behavclon/records')
        self.declare_parameter('twist_epsilon', 1e-4)
        self.declare_parameter('jpeg_quality', 92)
        self.declare_parameter('max_lin_speed', 0.5)
        self.declare_parameter('max_ang_speed', 0.5)
        self.declare_parameter('past_frames', 3)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        if not os.path.isabs(output_dir):
            output_dir = os.path.abspath(os.path.join(os.getcwd(), output_dir))
        else:
            output_dir = os.path.abspath(output_dir)
        self._twist_epsilon = float(
            self.get_parameter('twist_epsilon').get_parameter_value().double_value
        )
        self._jpeg_quality = int(
            self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        )
        self._max_lin_speed = float(
            self.get_parameter('max_lin_speed').get_parameter_value().double_value
        )
        self._max_ang_speed = float(
            self.get_parameter('max_ang_speed').get_parameter_value().double_value
        )
        self._past_frames = int(
            self.get_parameter('past_frames').get_parameter_value().integer_value
        )
        if self._past_frames < 0:
            self.get_logger().warn('past_frames < 0; using 0')
            self._past_frames = 0

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        self._output_dir = str(output_path)
        self._bridge = CvBridge()
        self._latest_cv_image = None
        self._last_recorded_twist: Twist | None = None
        self._pending_save = False
        self._pending_twist: Twist | None = None
        self._frame_buffer: deque | None = (
            deque(maxlen=self._past_frames) if self._past_frames > 0 else None
        )

        self.create_subscription(RosImage, image_topic, self._image_callback, _IMAGE_QOS)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 1)

        self.get_logger().info(
            f'behaviour_recorder: writing to {self._output_dir!r} '
            f'(image={image_topic!r}, cmd_vel={cmd_vel_topic!r}, past_frames={self._past_frames})'
        )

    def _cmd_vel_callback(self, msg: Twist):
        if self._last_recorded_twist is not None and _twists_close(
            msg, self._last_recorded_twist, self._twist_epsilon
        ):
            return
        self._pending_twist = deepcopy(msg)
        self._pending_save = True

    def _recording_filename(self, stamp_ns: int, twist: Twist) -> str:
        """{stamp_ns}_{propagation}{turn}.jpg — e.g. FN, FL, FR, BN, BL, BR, NN."""
        code = encode_control_code(
            twist.linear.x,
            twist.angular.z,
            max_lin_speed=self._max_lin_speed,
            max_ang_speed=self._max_ang_speed,
        )
        return f'{stamp_ns}_{code}.jpg'

    def _write_history_zip(
        self, zip_stamp_ns: int, past_entries: list, encode_params: list
    ) -> None:
        """past_entries: list of (image_stamp_ns, bgr_frame) oldest first."""
        zip_path = os.path.join(self._output_dir, f'{zip_stamp_ns}.zip')
        used_names: set[str] = set()
        with zipfile.ZipFile(zip_path, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
            for image_stamp_ns, past_frame in past_entries:
                base = f'{image_stamp_ns}.jpg'
                name = base
                n = 0
                while name in used_names:
                    n += 1
                    name = f'{image_stamp_ns}_{n}.jpg'
                used_names.add(name)
                ok_enc, buf = cv2.imencode('.jpg', past_frame, encode_params)
                if not ok_enc:
                    raise RuntimeError(f'cv2.imencode failed for {name!r}')
                zf.writestr(name, buf.tobytes())
        self.get_logger().info(
            f'Saved history {os.path.basename(zip_path)} ({len(past_entries)} frames)'
        )

    def _image_callback(self, msg: RosImage):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        self._latest_cv_image = cv_image
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        recv_ns = self.get_clock().now().nanoseconds
        frame_stamp_ns = _image_stamp_ns(msg, recv_ns)

        if self._pending_save and self._pending_twist is not None:
            past_snapshot = list(self._frame_buffer) if self._frame_buffer is not None else []

            stamp_ns = recv_ns
            filename = self._recording_filename(stamp_ns, self._pending_twist)
            path = os.path.join(self._output_dir, filename)

            try:
                frame = cv_image.copy() if not cv_image.flags['C_CONTIGUOUS'] else cv_image
                ok = cv2.imwrite(path, frame, encode_params)
                if not ok:
                    self.get_logger().error(f'cv2.imwrite failed for {path!r}')
                    return
            except Exception as exc:
                self.get_logger().error(f'Failed to write {path!r}: {exc}')
                return

            if past_snapshot:
                try:
                    self._write_history_zip(stamp_ns, past_snapshot, encode_params)
                except Exception as exc:
                    self.get_logger().error(f'Failed to write history zip: {exc}')

            self._last_recorded_twist = deepcopy(self._pending_twist)
            self._pending_save = False
            self._pending_twist = None
            self.get_logger().info(f'Saved {filename}')

        if self._frame_buffer is not None:
            self._frame_buffer.append((frame_stamp_ns, cv_image.copy()))


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
