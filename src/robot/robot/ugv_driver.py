import os
import time
from typing import Optional

from .base_driver import BaseDriver
from .ugv_base_ctrl import BaseController

UART_DEV = "UGV_UART"
BAUD_RATE = "UGV_BAUD"


class UGVDriver(BaseDriver):
    def __init__(self, half_wheel_separation=1, max_linear_speed=255, steps_without_command_threshold=30) -> None:
        super().__init__()
        self.__half_wheel_separation = half_wheel_separation
        self.__max_linear_speed = max_linear_speed

        # Serial connection parameters (overridable via properties or env)
        uart_dev = os.environ.get(UART_DEV, "/dev/ttyAMA0")
        baud_rate = int(os.environ.get(BAUD_RATE, 115200))

        # Hardware controller
        self._hardware_controller = BaseController(uart_dev, baud_rate)

    def velocity_to_motors(self, command_motor_left, command_motor_right):
        self._hardware_controller.base_speed_ctrl(command_motor_left, command_motor_right)


def main(args: Optional[list[str]] = None) -> None:
    del args
    driver = UGVDriver(half_wheel_separation=1, max_linear_speed=255)
    try:
        while True:
            driver.step()
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        pass
