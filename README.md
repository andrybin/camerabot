# CAMERABOT
![jjj](assets/real_drive_1.gif)

## Install on host-mashine
### 1. Setup container
If docker is not installed
```
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and log back in for the group changes to take effect
# Or run: newgrp docker
```
Use container: enter the ROS2 container (builds and starts it if needed)
```
./host.sh
```

Build packages (only for first time). Use **symlink install** so Python changes in `src/` take effect without rebuilding:
```
colcon build --symlink-install
```
(Without `--symlink-install`, you must run `colcon build` after every Python change.)

Or use fast script that do all the staff:
```
./b
```

Source environment (every time in new terminal connected to container). Run this **in the same terminal** before any `ros2` command:
```
. e
```

### 2. Run Webots (world only) or simulation
All commands below are run **inside the container** (after `./host.sh`). Ensure you ran `. e` in that terminal first.

Webots world only (no robot driver, no VLM):
```
ros2 launch robot webots_world.py
```

Webots simulation with teleop (keyboard: arrow keys; click terminal first):
```
ros2 launch robot sim_teleop.py
```

### 3. Install Ollama with vlm-model
Run outside the host container
```
curl -fsSL https://ollama.com/install.sh | sh
```
Optionally verify run the model
```
ollama run qwen2.5vl:latest
```

### 4. Run simulation with VLM
Inside the container (after `./host.sh`):
```
ros2 launch robot sim.py
```

## Installation on RaspberryPI5
### 1. Setup Docker
```
curl -fsSL https://get.docker.com -o get-docker.sh &&\
sudo sh get-docker.sh &&\
sudo usermod -aG docker $USER
```
Then relogin!

### 2. Clone this repo
```
git clone https://github.com/andrybin/camerabot.git
```
### 3. Setup ROS2-docker on robot
```
cd camerabot
chmod +x robot.sh
./robot.sh
```
### 4. Test camera
Continue from step 3 inside cmerabot container

Test camera (if you physically install camera only now - restart RPI5 after installation and run robot.sh):
```
python3 tools/picamera_test.py
```
Check frames on GUI-window or test_frame.jpg file

### 5. Configure connection
Enable I2C and SPI alowing to see UART device (in my case /dev/ttyAMA0)
```
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
```
Optionally use GUI menu
```
sudo raspi-config
```

## Set up both
### 1. Test communication between robot and host
On robot:
```
ros2 run robot talker
```
On host:
```
ros2 run robot listener
```

### 2. Test camera streem and control
On robot:
```
# first terminal
ros2 run robot camera

# second terminal
ros2 run robot ugv_contol
```

On host:
```
# first terminal to show image
rqt

# second terminal
ros2 run robot keyboard_teleop
```


ros2 topic echo /vlm/result --full-length