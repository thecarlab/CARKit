# Sensors

The sensors module contains CARKit sensor transform nodes. External sensor
drivers are fetched into this folder by `carkit/setup_vendor_repos.sh`, which
is called by `docker/build_workspace.sh`.

## Fetch Drivers

Inside Docker, from the repository root:

```bash
./carkit/setup_vendor_repos.sh
```

This creates:

- `carkit/sensors/realsense-ros`
- `carkit/sensors/sllidar_ros2`

## SLLiDAR

The top-level navigation launch starts the SLLiDAR driver by default while
mapping or navigating. For a direct driver-only check:

```bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

Expected topic:

- `/scan` (`sensor_msgs/LaserScan`)

Verify:

```bash
ros2 topic echo /scan --once
```

## RealSense

Perception expects color, aligned depth, and aligned camera info:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true
```

Expected topics:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/camera_info`
  (`sensor_msgs/CameraInfo`)

For IMU experiments, enable gyro and accel:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2
```

## CARKit Sensor Transforms

Package: `carkit_sensor_transforms`

Executables:

```bash
ros2 run carkit_sensor_transforms lidar_transformer_node
ros2 run carkit_sensor_transforms lidar_transformer_norotate_node
ros2 run carkit_sensor_transforms imu_transformer_node
```

`lidar_transformer_node` subscribes to `/scan` and publishes rotated
`/cloud_in` (`sensor_msgs/PointCloud2`) in `base_link`.

`lidar_transformer_norotate_node` subscribes to `/scan` and publishes
`/cloud_in` without the extra 180-degree rotation.

`imu_transformer_node` subscribes to `/camera/camera/imu` and publishes
`/imu_transformed` in `base_link`.
