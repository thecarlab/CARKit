# CARKit

CARKit is a ROS 2 Humble education stack for small Ackermann autonomous vehicles. It is designed for a Jetson Orin Nano with a 2D LiDAR, Intel RealSense camera, and an F1TENTH-style or CARKit/ADA vehicle controller.

CARKit uses one Docker image for the full build and runtime environment:

```text
ariiees/carkit:latest
```

You do not need to install ROS 2 on the host. The host only needs JetPack, Docker, Git, display access for RViz, and access to the connected sensors/devices.

## Supported Platform

- Jetson Orin Nano
- JetPack 6.x / L4T 36.x
- Ubuntu 22.04 host rootfs
- Docker with NVIDIA runtime
- ROS 2 Humble inside Docker

## Hardware

Recommended CARKit hardware:

- Jetson Orin Nano
- Slamtec SLLiDAR supported by `sllidar_ros2`
- Intel RealSense camera supported by `realsense2_camera`
- Ackermann/F1TENTH-style drive platform
- USB access for camera and LiDAR
- Serial, CAN, USB, or network access for your vehicle controller
- Monitor or X11 display forwarding for RViz

## Step 1: Clone CARKit

Run these commands on the Jetson host:

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
```

All later commands assume you are in this `CARKit` folder.

## Step 2: Clone Third-Party Source

CARKit keeps large third-party sensor and mapping packages out of git. Clone them into the required local folders before starting the Docker build workflow:

```bash
./carkit/setup_vendor_repos.sh
```

This reads [carkit/vendor.repos](carkit/vendor.repos) and clones:

- `IntelRealSense/realsense-ros` -> `carkit/sensors/realsense-ros`
- `Slamtec/sllidar_ros2` -> `carkit/sensors/sllidar_ros2`
- `rsasaki0109/ndt_omp_ros2` -> `carkit/mapping/ndt_omp_ros2`

These folders are ignored by git. If you need to refresh them later, update or remove the local folder and run the script again.

## Step 3: Pull The Docker Environment

Pull the CARKit environment image on the Jetson host:

```bash
docker pull ariiees/carkit:latest
```

This image contains ROS 2 Humble, RViz, colcon, rosdep, build tools, Python dependencies, and common sensor/mapping/control dependencies. It does not contain your CARKit checkout; your local repo is mounted into the container.

## Step 4: Start The Docker Container

From the CARKit repo on the Jetson host:

```bash
./docker/run_jetson.sh
```

The run script starts the container with:

- host networking for ROS 2 discovery
- NVIDIA runtime for Jetson acceleration
- `/dev` access for sensors, serial, USB, CAN, and vehicle devices
- `/dev/shm` for camera and perception workloads
- X11 mounts for RViz
- this repo mounted at `/workspaces/CARKit`

You should now be inside the container at:

```bash
/workspaces/CARKit
```

## Step 5: Build CARKit Inside Docker

Inside the container:

```bash
./docker/build_workspace.sh
source install/setup.bash
```

The build script runs the third-party clone script again if needed, installs dependencies with `rosdep`, builds the workspace with `colcon`, and checks that CARKit packages are visible.

## Step 6: Run CARKit

Launch the full CARKit stack:

```bash
ros2 launch carkit_bringup carkit.launch.py
```

The full stack starts LiDAR, RealSense, sensor transforms, LiDAR localization, pure pursuit, stop sign behavior, command mux, and RViz.

Check important topics:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /camera/camera/color/image_raw --once
ros2 topic echo /ackermann_cmd --once
```

## Control Bringup Choices

CARKit includes two control-only bringups.

Use the F1TENTH controller bringup when your vehicle already has a low-level F1TENTH ROS 2 controller package:

```bash
ros2 launch carkit_bringup f1tenth_control.launch.py \
  f1tenth_package:=f1tenth_stack \
  f1tenth_launch:=bringup_launch.py \
  vehicle_command_topic:=/drive
```

Use the CARKit/ADA control bringup when you want CARKit to run the path tracker, command mux, stop sign behavior, and optional demo nodes:

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py \
  vehicle_command_topic:=/ackermann_cmd
```

For demo nodes:

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py ada_demo:=demo1
ros2 launch carkit_bringup carkit_ada_control.launch.py ada_demo:=demo2
```

## Run Individual Modules

After building and sourcing `install/setup.bash` inside Docker:

```bash
# Sensors
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2

# Sensor transforms
ros2 run carkit_sensor_transforms lidar_transformer_node
ros2 run carkit_sensor_transforms imu_transformer_node

# Perception
ros2 run carkit_perception perception_node

# Localization and mapping
ros2 launch carkit_lidar_localization lidar_localization.launch.py
ros2 launch carkit_lidarslam lidarslam.launch.py

# Control and behavior
ros2 launch carkit_pure_pursuit pure_pursuit_system.launch.py waypoints_file:=carkit/bringup/waypoints/waypoints.yaml
ros2 run carkit_behaviors stop_sign_behavior_node
ros2 run carkit_command_mux carkit_command_mux_node
```

Each module folder has its own `README.md` with topics, launch commands, dependencies, and test commands.

## Repository Layout

```text
carkit/
  sensors/        sensor drivers and sensor transform nodes
  perception/     YOLO camera perception
  localization/   LiDAR NDT localization
  mapping/        LiDAR scan matching and graph SLAM
  planning/       behavior nodes such as stop sign handling
  control/        path tracking, emergency brake, command mux
  vehicle/        vehicle adapters, currently TODO
  bringup/        full-stack launch, maps, waypoints, RViz
  interfaces/     custom ROS 2 messages
  tools/          classroom demo and utility nodes
docker/           Docker image, run script, build/test scripts
docs/             topic graph, troubleshooting, migration notes
```

## Main Topic Flow

- `/scan` -> `carkit_sensor_transforms` -> `/cloud_in`
- `/camera/camera/color/image_raw` -> `carkit_perception` -> `/yolo/detections`
- `/camera/camera/imu` -> `carkit_sensor_transforms` -> `/imu_transformed`
- `/cloud_in` + map -> `carkit_lidar_localization` -> `/pcl_pose`
- `/pcl_pose` + `/follow_path` -> `carkit_pure_pursuit` -> `/purepursuit_cmd`
- `/joy_cmd`, `/purepursuit_cmd`, `/emergency_cmd`, `/stopsign_cmd` -> `carkit_command_mux` -> `/ackermann_cmd`

See [docs/topic_graph.md](docs/topic_graph.md) for more detail.

## Common Tests

Inside Docker after build:

```bash
source install/setup.bash
ros2 pkg list | grep carkit
ros2 launch carkit_bringup carkit.launch.py --show-args
ros2 launch carkit_bringup f1tenth_control.launch.py --show-args
ros2 launch carkit_bringup carkit_ada_control.launch.py --show-args
```

Sensor checks:

```bash
ros2 topic echo /scan --once
ros2 topic echo /camera/camera/color/image_raw --once
ros2 topic echo /camera/camera/imu --once
```

## Docker Image Maintenance

Most users should only pull `ariiees/carkit:latest`. Maintainers can rebuild and push the image from a Jetson with Docker access:

```bash
docker login
./docker/publish_image.sh
```

Test the image against this checkout:

```bash
./docker/test_workspace_in_docker.sh
```

Do not store Docker Hub passwords, tokens, or machine-specific secrets in this repository.

## Troubleshooting

See [docs/troubleshooting.md](docs/troubleshooting.md).

Common fixes:

- If Docker permission is denied, add your user to the `docker` group and start a new shell.
- If third-party packages are missing, run `./carkit/setup_vendor_repos.sh` on the host or inside Docker, then rebuild.
- If RViz does not open, verify `DISPLAY`, `/tmp/.X11-unix`, and `xhost +si:localuser:root`.
- If a sensor topic is missing, confirm the hardware is visible under `/dev` inside the container.
- If `ros2 launch` cannot find a package, run `source install/setup.bash` inside Docker.

## Contributing

- Keep CARKit code under the modular `carkit/` layout.
- Keep third-party source in the paths listed by `carkit/vendor.repos`.
- Do not commit build outputs, Docker credentials, tokens, or local machine paths.
- Update the module README when you add or change topics, parameters, launch files, or dependencies.
