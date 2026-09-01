# Sensors

The sensors module contains CARKit sensor transform nodes. External sensor
drivers are fetched into this folder by `carkit/setup_vendor_repos.sh`, which
is called by `docker/build_workspace.sh`.

On the OSRacer chassis, use `ros2 launch osracer_bringup sensors_launch.py`.
That adapter starts the bundled LakiBeam and UVC camera while preserving the
CARKit `/scan` and `/camera/camera/color/*` topic contracts.

The OSRacer camera uses the C++ `carkit_camera` node. It continuously drains
the camera at its native 30 FPS, keeps only the newest two-buffer V4L2 frame,
and publishes the newest MJPEG frame at 10 Hz. The normal compressed path does
not decode and re-encode the camera image. `/image_raw` is decoded lazily only
when a student node actually subscribes to it.

The complete CARKit bringup routes the chassis driver through `/scan/raw` and
then publishes `/scan` from the C++ `carkit_scan_filter` node. The filter removes
returns inside the lidar-centered 0.50 m × 0.25 m vehicle footprint, so every
localization, planning, behavior, and visualization consumer sees the same
self-filtered scan. Its `padding_m` parameter defaults to `0.0` and can be
increased if a chassis needs extra clearance.

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

The perception launch starts RealSense in color-only mode. For a direct
driver-only check, use the same low-CPU configuration:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=false \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  align_depth.enable:=false \
  enable_sync:=false \
  pointcloud.enable:=false
```

Expected topics:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

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
ros2 run carkit_scan_filter scan_footprint_filter_node
ros2 run carkit_camera low_latency_camera_node
```

`lidar_transformer_node` subscribes to `/scan` and publishes rotated
`/cloud_in` (`sensor_msgs/PointCloud2`) in `base_link`.

`lidar_transformer_norotate_node` subscribes to `/scan` and publishes
`/cloud_in` without the extra 180-degree rotation.

`imu_transformer_node` subscribes to `/camera/camera/imu` and publishes
`/imu_transformed` in `base_link`.
