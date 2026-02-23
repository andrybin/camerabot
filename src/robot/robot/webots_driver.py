from .base_driver import BaseDriver


class WebotsDriver(BaseDriver):
    def __init__(self, max_steps_without_command: int = 50):
        # The Webots ROS 2 driver instantiates plugins with no constructor args.
        # Keep constructor arg-free (with defaults) and do ROS init here.
        super().__init__(max_steps_without_command)

    def init(self, robot_node, properties, max_steps_without_command=60):
        
        self.__robot = robot_node.robot

        self._left_motor = self.__robot.getDevice('left wheel motor')
        self._right_motor = self.__robot.getDevice('right wheel motor')

        self._left_motor.setPosition(float('inf'))
        self._left_motor.setVelocity(0)

        self._right_motor.setPosition(float('inf'))
        self._right_motor.setVelocity(0)
        super().__init__(max_steps_without_command)

    def velocity_to_motors(self, command_motor_left, command_motor_right):
        # Minus for correct stright direction (fix it in future)
        if command_motor_left * command_motor_right > 0:
            command_motor_left *= -1
            command_motor_right *= -1
        self._left_motor.setVelocity(command_motor_left)
        self._right_motor.setVelocity(command_motor_right)
