import os
import time
from typing import Optional

from .base_driver import BaseDriver
from .ugv_base_ctrl import BaseController

UART_DEV = "UGV_UART"
BAUD_RATE = "UGV_BAUD"


class UGVDriver(BaseDriver):
    def __init__(self) -> None:
        super().__init__()

        # Serial connection parameters (overridable via properties or env)
        uart_dev = os.environ.get(UART_DEV, "/dev/ttyAMA0")
        baud_rate = int(os.environ.get(BAUD_RATE, 115200))

        # Hardware controller
        self._hardware_controller = BaseController(uart_dev, baud_rate)

    def velocity_to_motors(self, command_motor_left, command_motor_right):
        self._hardware_controller.base_speed_ctrl(command_motor_left, command_motor_right)


def main(args: Optional[list[str]] = None) -> None:
    del args
    driver = UGVDriver()
    try:
        while True:
            driver.step()
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        pass
