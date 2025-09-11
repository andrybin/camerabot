from ugv_base_ctrl import BaseController
import time

# Function for Detecting Raspberry Pi
def is_raspberry_pi5():
    with open('/proc/cpuinfo', 'r') as file:
        for line in file:
            if 'Model' in line:
                if 'Raspberry Pi 5' in line:
                    return True
                else:
                    return False

# Determine the GPIO Serial Device Name Based on the Raspberry Pi Model
if is_raspberry_pi5():
    base = BaseController('/dev/ttyAMA0', 115200)
else:
    base = BaseController('/dev/serial0', 115200)

for i in range(10):
    base.send_command({"T":1,"L":0.2,"R":0.2})
    time.sleep(1)
    base.send_command({"T":1,"L":-0.2,"R":-0.2})
    time.sleep(2)
    base.send_command({"T":1,"L":0,"R":0})
    time.sleep(1)
    base.send_command({"T":1,"L":0.4,"R":-0.4})
    time.sleep(2)