import random
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdRedirectNode(Node):
    def __init__(self):
        super().__init__('vel_augmenter')

        self.declare_parameter('input_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('period', 5.0)
        self.declare_parameter('turn_value', 0.5)
        self.declare_parameter('duration', 1.0)

        input_topic = self.get_parameter('input_cmd_vel_topic').get_parameter_value().string_value
        output_topic = (
            self.get_parameter('output_cmd_vel_topic').get_parameter_value().string_value
        )
        self._period = float(self.get_parameter('period').value)
        self._turn_value = float(self.get_parameter('turn_value').value)
        self._duration = float(self.get_parameter('duration').value)

        self._augment_angular_z = 0.0
        self._augment_until = 0.0

        self._publisher = self.create_publisher(Twist, output_topic, 1)
        self.create_subscription(TwistStamped, input_topic, self._cmd_vel_callback, 1)

        if self._period > 0.0 and self._turn_value != 0.0 and self._duration > 0.0:
            self._trigger_random_turn()
            self.create_timer(self._period, self._trigger_random_turn)

        self.get_logger().info(
            f'vel_augmenter: {input_topic!r} -> {output_topic!r}, '
            f'random turn every {self._period}s: ±{self._turn_value} for {self._duration}s'
        )

    def _trigger_random_turn(self):
        sign = random.choice([-1.0, 1.0])
        self._augment_angular_z = sign * self._turn_value
        self._augment_until = time.monotonic() + self._duration
        direction = 'left' if sign > 0 else 'right'
        self.get_logger().info(
            f'Random turn {direction}: angular.z += {self._augment_angular_z:.2f} '
            f'for {self._duration:.2f}s'
        )

    def _active_augment_angular_z(self) -> float:
        if time.monotonic() < self._augment_until:
            return self._augment_angular_z
        self._augment_angular_z = 0.0
        return 0.0

    def _cmd_vel_callback(self, msg: TwistStamped):
        out = Twist()
        out.linear = msg.twist.linear
        out.angular = msg.twist.angular
        out.angular.z += self._active_augment_angular_z()
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdRedirectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
