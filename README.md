# ROS Workspace for SUAVE

## To build
Dependencies:
- ROS Humble (Including devlopment tools)
- MAVSDK
- PCL

```
cd suavecpp-ros2_ws/setup
source install_deps.bash
cd ..
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
