# ROS Workspace for SUAVE

## Initialize your workspace and environment
```
mkdir ~/Dev
cd ~/Dev
git clone https://github.com/patrick4k/suavecpp-ros2_ws.git
cd suavecpp-ros2-ws/setup
source setup.bash
```

## To build
Dependencies:
- ROS Humble (Including devlopment tools)
- MAVSDK
- PCL

```
colcon build
source ./install setup.bash
```

## To run
Launch PX4 SITL
```
cd PX4-Autopilot
make px4_sitl gz_x500
```
Launch suave_main
```
ros2 run suave_launch suave_main
```
