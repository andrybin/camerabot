import os
import zipfile
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image as RosImage

from behavclon.common import encode_control_code

_IMAGE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
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


def _header_stamp_ns(header, fallback_ns: int) -> int:
    """Nanoseconds from message header stamp, or fallback if stamp is unset."""
    stamp = header.stamp
    if stamp.sec != 0 or stamp.nanosec != 0:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return int(fallback_ns)


def _image_stamp_ns(msg: RosImage, fallback_ns: int) -> int:
    return _header_stamp_ns(msg.header, fallback_ns)


def _format_time_ns(stamp_ns: int) -> str:
    return datetime.fromtimestamp(stamp_ns / 1_000_000_000).strftime('%H:%M:%S.%f')[:-3]


@dataclass
class _PendingRecord:
    cmd_stamp_ns: int
    twist: Twist
    image_entry: tuple[int, object] | None = None


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

        output_path = Path(output_dir) / datetime.now().strftime('%y:%m:%d:%H:%M:%S')
        output_path.mkdir(parents=True, exist_ok=True)
        self._output_dir = str(output_path)
        self._bridge = CvBridge()
        self._last_recorded_twist: Twist = Twist()
        self._pending_record: _PendingRecord | None = None
        pool_size = max(self._past_frames + 2, 10)
        self._frame_buffer: deque = deque(maxlen=pool_size)

        self.create_subscription(RosImage, image_topic, self._image_callback, _IMAGE_QOS)
        self.create_subscription(TwistStamped, cmd_vel_topic, self._cmd_vel_callback, 1)

        self.get_logger().info(
            f'behaviour_recorder: writing to {self._output_dir!r} '
            f'(image={image_topic!r}, cmd_vel={cmd_vel_topic!r}, past_frames={self._past_frames})'
        )

    def _cmd_vel_callback(self, msg: TwistStamped):
        twist = msg.twist
        if self._pending_record is not None and _twists_close(
            twist, self._pending_record.twist, self._twist_epsilon
        ):
            return

        cmd_stamp_ns = _header_stamp_ns(msg.header, self.get_clock().now().nanoseconds)
        if self._pending_record is not None:
            self._save_pending_record('new command received')
        if self._last_recorded_twist is not None and _twists_close(
            twist, self._last_recorded_twist, self._twist_epsilon
        ):
            return

        self._pending_record = _PendingRecord(
            cmd_stamp_ns, deepcopy(twist), self._latest_image_before(cmd_stamp_ns)
        )
        if self._has_image_at_or_after(cmd_stamp_ns):
            self._save_pending_record('image at/after cmd already in buffer')

    def _latest_image_before(self, cmd_stamp_ns: int) -> tuple[int, object] | None:
        """Most recent buffered frame strictly before cmd_stamp_ns."""
        before = [entry for entry in self._frame_buffer if entry[0] < cmd_stamp_ns]
        if not before:
            return None
        return max(before, key=lambda entry: entry[0])

    def _has_image_at_or_after(self, cmd_stamp_ns: int) -> bool:
        return any(entry[0] >= cmd_stamp_ns for entry in self._frame_buffer)

    def _update_pending_image(self, entry: tuple[int, object]) -> None:
        if self._pending_record is None:
            return

        image_stamp_ns, _ = entry
        cmd_stamp_ns = self._pending_record.cmd_stamp_ns
        if image_stamp_ns < cmd_stamp_ns:
            current = self._pending_record.image_entry
            if current is None or image_stamp_ns > current[0]:
                self._pending_record.image_entry = entry
            return

        self._save_pending_record('first image at/after cmd')

    def _save_pending_record(self, reason: str) -> bool:
        pending_record = self._pending_record
        if pending_record is None:
            return False
        if pending_record.image_entry is None:
            self.get_logger().warn(
                f'No image received for cmd at {_format_time_ns(pending_record.cmd_stamp_ns)}; '
                f'skipping behaviour record'
            )
            self._pending_record = None
            return False

        image_stamp_ns, cv_image = pending_record.image_entry
        saved = self._save_record(
            pending_record.cmd_stamp_ns,
            pending_record.twist,
            image_stamp_ns,
            cv_image,
            reason,
        )
        if saved:
            self._pending_record = None
        return saved

    def _save_record(
        self,
        cmd_stamp_ns: int,
        twist: Twist,
        image_stamp_ns: int,
        cv_image,
        reason: str,
    ) -> bool:
        delta_ms = (image_stamp_ns - cmd_stamp_ns) / 1_000_000.0
        past_snapshot = self._past_frames_before(image_stamp_ns)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        filename = self._recording_filename(cmd_stamp_ns, twist)
        path = os.path.join(self._output_dir, filename)

        try:
            frame = cv_image.copy() if not cv_image.flags['C_CONTIGUOUS'] else cv_image
            ok = cv2.imwrite(path, frame, encode_params)
            if not ok:
                self.get_logger().error(f'cv2.imwrite failed for {path!r}')
                return False
        except Exception as exc:
            self.get_logger().error(f'Failed to write {path!r}: {exc}')
            return False

        if past_snapshot:
            try:
                self._write_history_zip(cmd_stamp_ns, past_snapshot, encode_params)
            except Exception as exc:
                self.get_logger().error(f'Failed to write history zip: {exc}')

        self._last_recorded_twist = twist
        self.get_logger().info(
            f'Saved {filename} ({reason}, '
            # f'cmd={_format_time_ns(cmd_stamp_ns)}, image={_format_time_ns(image_stamp_ns)}, '
            f'cmd-image Δ={delta_ms:+.1f} ms, '
            f'image {"before" if delta_ms < 0 else "after"} cmd)'
        )
        return True

    def _past_frames_before(self, anchor_stamp_ns: int) -> list:
        """Up to past_frames entries strictly before anchor_stamp_ns, oldest first."""
        if self._past_frames <= 0:
            return []
        past = [(s, img) for s, img in self._frame_buffer if s < anchor_stamp_ns]
        return past[-self._past_frames :]

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

        recv_ns = self.get_clock().now().nanoseconds
        frame_stamp_ns = _image_stamp_ns(msg, recv_ns)
        frame_entry = (frame_stamp_ns, cv_image.copy())
        self._frame_buffer.append(frame_entry)
        self._update_pending_image(frame_entry)


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
