# kachaka_navyu

This is a launch file to launch navigation module for kachaka using navyu.  
https://github.com/RyuYamamoto/navyu

## How to use
### 1. Build navyu
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RyuYamamoto/navyu.git
git clone https://github.com/RyuYamamoto/kachaka_navyu.git
cd ../
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to navyu kachaka_navyu
```

### 2. Setup Kachaka ROS 2 Bridge
Clone [kachaka-api package](https://github.com/pf-robotics/kachaka-api).  
See repository for details.
```bash
git clone https://github.com/pf-robotics/kachaka-api
``` 

Build depend packages.
```bash
cd ~/ros2_ws/src
ln -s ~/kachaka-api/ros2/kachaka_interfaces/ kachaka_interfaces
ln -s ~/kachaka-api/ros2/kachaka_description/ kachaka_description
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to kachaka_interfaces kachaka_description
```

### 3. Launch navigation
Launch ROS 2 Bridge.
```bash
cd kachaka-api/
./start_bridge.sh <kachaka IP Adress>
```

Launch navyu.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch kachaka_navyu navyu_bringup.launch.py
```
