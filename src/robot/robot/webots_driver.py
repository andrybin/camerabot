from .base_driver import BaseDriver
from .base_driver import log_command_if_changed

# Motor sign hack: flip only when both sides are strictly the same nonzero sign,
# or when the right command is ~0 and the left is clearly negative (v = -ω in
# diff kinematics). Do not flip when the left is ~0 and the right is negative
# (v = ω → left wheel kinematically stops); treating that as "both backward"
# inverts the turn (see cmd_vel with linear == angular).
_MOTOR_FLIP_EPS = 1e-9


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
        # Minus for correct straight direction (fix it in future)
        # Use same-hemisphere check, not (left * right > 0): when one side is
        # exactly 0.0 / -0.0 the product is 0 and the flip is skipped, which
        # inverts turning vs slightly nonzero commands (e.g. -1.0 and -0.0).
        both_nonpositive = command_motor_left <= 0.0 and command_motor_right <= 0.0
        both_nonnegative = command_motor_left >= 0.0 and command_motor_right >= 0.0
        if both_nonpositive or both_nonnegative:
            command_motor_left *= -1
            command_motor_right *= -1
        self._left_motor.setVelocity(command_motor_left)
        self._right_motor.setVelocity(command_motor_right)
