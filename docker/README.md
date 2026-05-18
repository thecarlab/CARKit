# Docker

CARKit uses `ariiees/carkit:latest` as the single environment image. The image contains ROS 2 Humble, build tools, RViz, Python tools, and common CARKit system dependencies. It does not contain a baked copy of the CARKit workspace.

## User Flow

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
docker pull ariiees/carkit:latest
./docker/run_jetson.sh
```

Inside Docker:

```bash
./docker/build_workspace.sh
source install/setup.bash
ros2 launch carkit_bringup carkit.launch.py
```

## Scripts

- `run_jetson.sh`: starts the container with host networking, NVIDIA runtime, `/dev`, `/dev/shm`, and X11 display mounts.
- `build_workspace.sh`: clones external source packages, runs `rosdep`, builds with `colcon`, and lists CARKit packages.
- `publish_image.sh`: maintainer-only helper to build and push `ariiees/carkit:latest`.
- `test_workspace_in_docker.sh`: pulls/runs the image, builds this checkout, and checks launch arguments.

## Maintainer Publish

```bash
docker login
./docker/publish_image.sh
```

Never put Docker Hub credentials in this repository.

## Python ML Packages

The Dockerfile installs `ultralytics` with pip constraints instead of a plain `pip install ultralytics`. This avoids a Jetson/Ubuntu build failure where pip tries to uninstall apt-owned `sympy 1.9`, keeps NumPy below 2.x for ROS 2 Humble and Jetson compatibility, and keeps `setuptools<80` so `colcon-core` remains compatible.
