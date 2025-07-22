## Installing on Jetson Orin Nano

1. Allow .sh files to be executable.
```
cd ~/CARKit
chmod +x ./install_env.sh
chmod +x ./build_ws.sh
```

2. Local environment setup, 20 mins expected.
```
sudo ./install_env.sh
```

3. Workspace build and installation, 30 mins expected.
```
./build_ws.sh
```

4. Docker pull all needed containers.
```
docker pull williamhecoin/ada_academy:perception2025v1
docker pull ariiees/ada:foxy-f1tenth
```

6. Install modified f1tenth workspace.
```
git clone https://github.com/thecarlab/ada_system.git
cd ~/ada_system
chmod +x ./run_container.sh
./run_container.sh
```

## CARKit System Launch
1. Control and Perception Container Launch

[Terminal 1]
```
cd ~/CARKit/src/ada/launch
./start.sh
```

2. Whole system launch

[Terminal 2]
```
sudo -i
source /opt/ros/humble/setup.bash
source /home/YOUR_USR_NAME/CARKit/install/setup.bash
source /home/YOUR_USR_NAME/sensor_ws/install/setup.bash
ros2 launch ada ada_system.launch.py
```

## Separate Modules and Sensors Launch

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
cd ~/CARKit/src/perception/util
./run_container.sh
```
In the container, do:
```
cd perception_ws/util
.setup.bash
cd perception_ws
colcon build --symlink-install
source install/setup.bash
ros2 run perception perception_node
```

## Reference
This repository includes code from several third-party repositories. To simplify cloning, building, and making our own modifications, we have integrated them directly as local files.

Below is a list of the original upstream repositories. We highly recommend visiting them to learn more, follow their development, and read their documentation:

- lidarslam_ros2: [original here](https://github.com/rsasaki0109/lidarslam_ros2)

- lidar_localization_ros2: [original here](https://github.com/rsasaki0109/lidar_localization_ros2)
