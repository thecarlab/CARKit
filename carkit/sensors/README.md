# Sensors

The sensors module contains CARKit sensor transform nodes and clones external sensor drivers into this folder.

## Fetch Drivers

Inside Docker, from the repo root:

```bash
./carkit/setup_vendor_repos.sh
```

This creates:

- `carkit/sensors/realsense-ros`
- `carkit/sensors/sllidar_ros2`

## SLLiDAR Driver

Package after fetch: `sllidar_ros2`

Launch:

```bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

Test:

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

Expected output topic:

- `/scan` (`sensor_msgs/LaserScan`)

## RealSense Driver

Package after fetch: `realsense2_camera`

Launch:

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

Test:

```bash
ros2 topic echo /camera/camera/color/image_raw --once
ros2 topic echo /camera/camera/depth/image_rect_raw --once
ros2 topic echo /camera/camera/imu --once
```

Expected output topics:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/depth/image_rect_raw` (`sensor_msgs/Image`)
- `/camera/camera/imu` (`sensor_msgs/Imu`)

## CARKit Sensor Transforms

Package: `carkit_sensor_transforms`

Launch:

```bash
ros2 run carkit_sensor_transforms lidar_transformer_node
ros2 run carkit_sensor_transforms imu_transformer_node
```

Test:

```bash
ros2 topic echo /cloud_in --once
ros2 topic echo /imu_transformed --once
```

Inputs:

- `/scan` (`sensor_msgs/LaserScan`)
- `/camera/camera/imu` (`sensor_msgs/Imu`)

Outputs:

- `/cloud_in` (`sensor_msgs/PointCloud2`)
- `/imu_transformed` (`sensor_msgs/Imu`)
