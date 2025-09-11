import os
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist

from .ugv_base_ctrl import BaseController


class UGVDriver:
    def __init__(self, robot_node: Any, half_wheel_separation=1, max_linear_speed=255, steps_without_command_threshold=30) -> None:
        self.__node = None  # type: ignore[assignment]
        self.__base = None  # type: ignore[assignment]
        self._target_twist = Twist()
        self.__half_wheel_separation = half_wheel_separation
        self.__max_linear_speed = max_linear_speed

        # robot_node is accepted for interface compatibility with Webots but is unused here
        del robot_node

        # Serial connection parameters (overridable via properties or env)
        uart_dev = os.environ.get("UGV_UART", "/dev/ttyAMA0")
        baud_rate = int(os.environ.get("UGV_BAUD", 115200))

        # Hardware controller
        self._base = BaseController(uart_dev, baud_rate)

        # ROS 2 node and subscription
        rclpy.init(args=None)
        self.__node = rclpy.create_node("ugv_driver")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)

        self.steps_without_command = 0
        self.steps_without_command_threshold = steps_without_command_threshold

    def __cmd_vel_callback(self, twist: Twist) -> None:
        self._target_twist = twist
        self.steps_without_command = 0

    def step(self) -> None:
        # Process a single ROS 2 cycle
        rclpy.spin_once(self.__node, timeout_sec=0)

        # If no command has been received for 50 steps, stop the robot
        self.steps_without_command += 1
        if self.steps_without_command > self.steps_without_command_threshold:
            self._target_twist = Twist()

        # Convert Twist to differential wheel linear speeds (m/s)
        forward_speed = float(self._target_twist.linear.x)
        angular_speed = float(self._target_twist.angular.z)

        left_linear_speed = forward_speed - angular_speed * self.__half_wheel_separation
        right_linear_speed = forward_speed + angular_speed * self.__half_wheel_separation

        # Scale to controller input range [-255, 255] using configured max linear speed
        scale = 255.0 / self.__max_linear_speed if self.__max_linear_speed > 0 else 0.0
        left_cmd = left_linear_speed#int(max(-255, min(255, left_linear_speed * scale)))
        right_cmd = right_linear_speed#int(max(-255, min(255, right_linear_speed * scale)))

        # Send to base
        self._base.base_speed_ctrl(left_cmd, right_cmd)


def main(args: Optional[list[str]] = None) -> None:
    del args
    driver = UGVDriver(robot_node=None, half_wheel_separation=1, max_linear_speed=255)
    steps_in_motion = 0
    try:
        while True:
            driver.step()
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        pass
