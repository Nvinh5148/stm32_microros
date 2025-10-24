
## ⚙️ Setup Instructions

### 1️⃣ Clone this repository (with submodules)
```bash
git clone --recurse-submodules https://github.com/Nvinh5148/stm32_microros.git
cd stm32_microros
If you already cloned without submodules:
git submodule update --init --recursive
### 
2️⃣ Build the firmware using STM32CubeIDE
- run docker first in terminal 1 
sudo chmod 666 /var/run/docker.sock
3️⃣ Run the micro-ROS Agent on your PC
in terminal 2
source /home/vinh/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0


🔗 References
micro.ros.org
micro-ROS STM32CubeMX Utils GitHub
micro-ROS Tutorials
