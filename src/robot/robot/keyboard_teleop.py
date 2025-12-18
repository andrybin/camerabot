import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Parameters for movement speeds
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.2)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # Current command to publish continuously
        self.current_twist = Twist()

        # Publish at a steady rate so the robot continues moving until changed
        self.publish_timer = self.create_timer(0.1, self._publish_current_command)

        # Prepare TTY for key input
        self._tty_file = None
        self._tty_fd = None
        self._old_term_settings = None
        self._keep_running = True
        try:
            # Prefer controlling terminal directly (binary mode)
            self._tty_file = open('/dev/tty', 'rb', buffering=0)
            self._tty_fd = self._tty_file.fileno()
            self._old_term_settings = termios.tcgetattr(self._tty_fd)
            tty.setcbreak(self._tty_fd)
            self.get_logger().info('Keyboard teleop connected to /dev/tty for input.')
        except (OSError, ValueError):
            # Fallback to stdin if it is a TTY
            if sys.stdin.isatty():
                # Use buffer to read raw bytes
                self._tty_file = sys.stdin.buffer
                self._tty_fd = self._tty_file.fileno()
                try:
                    self._old_term_settings = termios.tcgetattr(self._tty_fd)
                    tty.setcbreak(self._tty_fd)
                    self.get_logger().info('Keyboard teleop using stdin for input.')
                except (OSError, ValueError) as exc:
                    self.get_logger().warn(f'Cannot set TTY mode on stdin: {exc}. Keyboard input disabled.')
                    self._keep_running = False
            else:
                self.get_logger().warn('No TTY available (/dev/tty and stdin unusable). Keyboard input disabled.')
                self._keep_running = False

        # Thread to capture keyboard input without blocking ROS spinning
        if self._keep_running:
            self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._keyboard_thread.start()

        self.get_logger().info('Keyboard teleop started. Use arrow keys to move, space to stop.')

    def _publish_current_command(self):
        self.cmd_vel_publisher.publish(self.current_twist)

    def _keyboard_loop(self):
        if self._tty_file is None or self._tty_fd is None:
            return
        try:
            while self._keep_running:
                if self._stdin_has_data(timeout=0.1):
                    key = os.read(self._tty_fd, 1)
                    if key == b'\x1b':
                        seq = self._read_escape_sequence()
                        self.get_logger().debug(f'Key sequence: {repr(seq)}')
                        self._handle_arrow_key(seq)
                    elif key == b' ':  # Space bar
                        self._stop_robot()
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Keyboard loop error: {exc}')
        finally:
            if self._old_term_settings is not None and self._tty_fd is not None:
                try:
                    termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._old_term_settings)
                except (OSError, ValueError):
                    pass

    def _stdin_has_data(self, timeout: float) -> bool:
        if self._tty_file is None:
            return False
        readable, _, _ = select.select([self._tty_fd], [], [], timeout)
        return bool(readable)

    def _read_escape_sequence(self) -> bytes:
        # Read remaining bytes of escape sequence (e.g., ESC [ A or ESC O A)
        seq = b'\x1b'
        deadline = time.monotonic() + 0.25
        # Collect up to 8 bytes or until timeout between bytes
        while time.monotonic() < deadline and len(seq) < 8:
            if self._stdin_has_data(timeout=0.03):
                try:
                    seq += os.read(self._tty_fd, 1)
                except (BlockingIOError, OSError):
                    break
            else:
                break
        return seq

    def _handle_arrow_key(self, seq: bytes):
        # Support ESC [ A/B/C/D and ESC O A/B/C/D and variants with modifiers (e.g., [1;5A)
        if not seq.startswith(b'\x1b'):
            return
        if seq.endswith(b'A') or b'[A' in seq or b'OA' in seq:
            self._move_forward()
        elif seq.endswith(b'B') or b'[B' in seq or b'OB' in seq:
            self._move_backward()
        elif seq.endswith(b'D') or b'[D' in seq or b'OD' in seq:
            self._rotate_counterclockwise()
        elif seq.endswith(b'C') or b'[C' in seq or b'OC' in seq:
            self._rotate_clockwise()

    def _move_forward(self):
        if self.current_twist.linear.x > 0:
            # 🚀 Increment speed if already moving forward
            self.current_twist.linear.x += self.linear_speed
        else:
            # 🚗 Set speed to default if not moving forward
            self.current_twist.linear.x = self.linear_speed
        self.current_twist.angular.z = 0.0
        self.get_logger().info(f'🟢 Move forward: {self.current_twist.linear.x:.2f}')

    def _move_backward(self):
        self.current_twist.linear.x = -self.linear_speed
        self.current_twist.angular.z = 0.0
        self.get_logger().info(f'🔴 Move backward: {self.current_twist.linear.x:.2f}')

    def _rotate_counterclockwise(self):
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -self.angular_speed
        self.get_logger().info(f'🔄 Rotate counterclockwise: {self.current_twist.angular.z:.2f}')

    def _rotate_clockwise(self):
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = self.angular_speed
        self.get_logger().info(f'🔁 Rotate clockwise: {self.current_twist.angular.z:.2f}')

    def _stop_robot(self):
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.get_logger().info('⛔ Stop')


    def destroy_node(self):
        self._keep_running = False
        # Close and restore TTY
        if self._old_term_settings is not None and self._tty_fd is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._old_term_settings)
            except (OSError, ValueError):
                pass
        if self._tty_file not in (None, sys.stdin, getattr(sys.stdin, 'buffer', None)):
            try:
                self._tty_file.close()
            except (OSError, ValueError):
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


