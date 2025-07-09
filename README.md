# ADA Demo Tutorial

TODO

- [x] System Setup
- [x] Playground Setting
- [x] Localization choosing: Lidar vs. Camera
- [x] Map Building
- [x] Pure Pusit ROS2 Implementation
- [x] Stop sign and pedestrian sign detection
- [x] Final Integration
- [ ] Code clean
- [ ] Tutorial write
- [ ] Other vehicle test

## Launch System and Sensors

1. Start F1tenth System in the container
```
/home/ada1/f1tenth_system/scripts/run_container.sh
```
  In the container, do:
```
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```
2. Launch LiDAR sensor (local)
```
ros2 launch sllidar_ros2 sllidar_s1_launch.py
```
3. Launch Camera (local)
```
ros2 run realsense2_camera realsense2_camera_node
```
  More launch options for camera with IMU.
```
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
```
4. Launch Perception Yolo in the container
```
cd perception_ws/python_examples
./run_container.sh
```
In the container, do:
```
source install/setup.bash
ros2 run ada_perception perception_node
```

## NavOS Algorithm Launch
1. Control and Perception Container Launch
```
cd ros2_ws/src/ada/launch
./start.sh
```

2. Localization, pure pusit, and stop sign control launch
```
cd ros2_ws/
source install/setup.bash
ros2 run ada ada_system.launch.py
```
