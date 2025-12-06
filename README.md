## Install on host-mashine
### 1. Run container and enter
```
./host.sh
```

### 2. Build packages (only for first time)
```
colcon build
. e
```

### 3. Simple simulation test
```
ros2 launch robot sim.py simple_control:=true vlm_control:=false
```

### 4. Install Ollama and the vlm-model
```
curl -fsSL https://ollama.com/install.sh | sh
ollama run qwen2.5vl:3b
```

### 5. Run simulation with VLM
```
ros2 launch robot sim.py simple_control:=false vlm_control:=true
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
```
Test camera (if you physically install camera only now - restart RPI5 after installation and run robot.sh):
```
python3 tools/picamera_test.py
```
Check frames on GUI-window or test_frame.jpg file
