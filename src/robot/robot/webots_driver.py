from .base_driver import BaseDriver


class WebotsDriver(BaseDriver):
    def init(self, robot_node, properties):
        self.__robot = robot_node.robot

        self._left_motor = self.__robot.getDevice('left wheel motor')
        self._right_motor = self.__robot.getDevice('right wheel motor')

        self._left_motor.setPosition(float('inf'))
        self._left_motor.setVelocity(0)

        self._right_motor.setPosition(float('inf'))
        self._right_motor.setVelocity(0)

    def velocity_to_motors(self, command_motor_left, command_motor_right):
        # Check if motors are initialized before using them
        if hasattr(self, '_left_motor') and hasattr(self, '_right_motor'):
            self._left_motor.setVelocity(-command_motor_left)
            self._right_motor.setVelocity(-command_motor_right)