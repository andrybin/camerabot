from abc import abstractmethod

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HALF_DISTANCE_BETWEEN_WHEELS = 1.
WHEEL_RADIUS = 1.
DEFAULT_STEPS_WITHOUT_COMMAND_THRESHOLD = 30

class BaseDriver(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__(self.__class__.__name__)
        self._target_twist = Twist()
        self.steps_without_command = 0
        self.steps_without_command_threshold = DEFAULT_STEPS_WITHOUT_COMMAND_THRESHOLD
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 1)

    def _cmd_vel_callback(self, twist):
        self._target_twist = twist
        self.steps_without_command = 0

    def set_left_right_wheels_velocity(self):
        forward_speed = self._target_twist.linear.x
        angular_speed = self._target_twist.angular.z

        # Minus for correct rotation direction
        command_motor_left = -(forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = -(forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        return command_motor_left, command_motor_right

    @abstractmethod
    def velocity_to_motors(self, command_motor_left, command_motor_right):
        pass

    def step(self):
        rclpy.spin_once(self, timeout_sec=0)

        # If no command has been received for N steps, stop the robot
        self.steps_without_command += 1
        if self.steps_without_command > self.steps_without_command_threshold:
            if self._target_twist.linear.x != 0 or self._target_twist.angular.z != 0:
                self.get_logger().warn(f"No command received for {self.steps_without_command} steps, stopping the robot")
                self._target_twist = Twist()

        self.velocity_to_motors(*self.set_left_right_wheels_velocity())