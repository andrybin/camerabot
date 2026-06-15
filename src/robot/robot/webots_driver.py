from .base_driver import BaseDriver
from .base_driver import log_command_if_changed


class WebotsDriver(BaseDriver):
    def __init__(self, max_steps_without_command: int = 50):
        # The Webots ROS 2 driver instantiates plugins with no constructor args.
        # Keep constructor arg-free (with defaults) and do ROS init here.
        super().__init__(max_steps_without_command)

    def init(self, robot_node, properties, max_steps_without_command=60):
        
        self.__robot = robot_node.robot

        self._left_motor = self.__robot.getDevice('left wheel motor')
        self._left_motor.setPosition(float('inf'))
        self._left_motor.setVelocity(0)

        self._right_motor = self.__robot.getDevice('right wheel motor')
        self._right_motor.setPosition(float('inf'))
        self._right_motor.setVelocity(0)
        super().__init__(max_steps_without_command)

    @log_command_if_changed
    def velocity_to_motors(self, command_motor_left, command_motor_right):
        # Webots wheels share the same joint axis (0 1 0) on mirrored mounts; equal
        # commands drive straight, but diff-drive turn sense is inverted vs base
        # kinematics. Swapping preserves forward/back and fixes left/right turns.
        self._left_motor.setVelocity(command_motor_right)
        self._right_motor.setVelocity(command_motor_left)
