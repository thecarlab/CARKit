# Docker

CARKit uses `ariiees/carkit:latest` as the single Jetson runtime image. The
image contains ROS 2 Humble Desktop, Nav2, RViz/rqt, build tools, RealSense
SDK, TensorRT/CUDA Python ML dependencies, and the system
dependencies needed to build the mounted CARKit workspace.

The image does not contain a baked copy of this repository. `run_jetson.sh`
mounts the checkout at `/workspaces/CARKit`.

## User Flow

Clone the release branch, then run one setup command with the vehicle ID and
chassis:

```bash
git clone --branch ada2026 https://github.com/thecarlab/CARKit.git ~/CARKit
cd ~/CARKit
./docker/setup_jetson.sh ADA5 f1tenth
```

Use `osracer` instead of `f1tenth` for an OSRacer. The first run securely
prompts for the CARLab monitor token; later runs reuse the protected host
configuration. The setup installs missing host requirements, configures the
NVIDIA Docker runtime, pulls the image, installs applicable device rules, and
enables both the WebUI/container and IP reporter at boot.

Find the vehicle in the public fleet monitor:

[CARLab ADA Fleet Monitor](https://carlab-ada-monitor.udcarlab.chatgpt.site)

## Start Automatically at Boot

`setup_jetson.sh` installs and enables both boot services. Manage them with:

```bash
systemctl status carkit.service carkit-webmonitor.service
sudo systemctl restart carkit.service
sudo systemctl restart carkit-webmonitor.service
sudo systemctl stop carkit.service
journalctl -u carkit.service -f
journalctl -u carkit-webmonitor.service -f
```

The reporter updates hourly while connected. When the vehicle has no internet
access, it tries every five minutes and stops after 20 consecutive failures.
Restart `carkit-webmonitor.service` to begin a new retry window without
rebooting the vehicle.

Choose `osracer` or `f1tenth`, install/build it, then start the desired course
profile and components. Use `./docker/run_jetson.sh bash` when a terminal is
preferred.

`run_jetson.sh` pulls `ariiees/carkit:latest` only when it is missing locally.
Use `PULL_IMAGE=always ./docker/run_jetson.sh` to force a Docker Hub refresh,
or `PULL_IMAGE=never ./docker/run_jetson.sh` when testing a local image that
should not be refreshed from Docker Hub.

`./docker/run_jetson.sh` runs `docker/start_webui.sh` in the mounted workspace.
The dashboard owns the one unified `carkit_bringup` child launch. Supplying a
command after `run_jetson.sh` replaces the dashboard command.

Before starting the container command, `run_jetson.sh` verifies that the
selected image contains the navigation, sensor, chassis, and Python runtime
packages used by CARKit. It starts the container with host networking,
`/dev`, `/dev/shm`, and NVIDIA runtime support when the runtime is registered.

`setup_osracer_device.sh` installs host udev rules for the `303a:1001`
controller and `2993:0858` UVC camera. It verifies `/dev/osrbot_base` and
creates `/dev/osrbot_usb_cam`. It also identifies the Richbeam RNDIS adapter,
creates the persistent `carkit-lakibeam` NetworkManager connection at
`192.168.8.1/24`, and prevents LiDAR traffic from using `l4tbr0` or the CUDY
default route. Run it again after changing a rule or if a stale regular file
exists at either device path, then restart the container.

Start the OSRacer sensors inside the container with:

```bash
ros2 launch osracer_bringup sensors_launch.py
```

The container runs as root by default so hardware access and repeated ROS
terminals stay consistent on the Jetson.

Overrides:

```bash
# Run as your host UID/GID instead of root.
CARKIT_RUN_AS_ROOT=0 ./docker/run_jetson.sh

# Temporarily skip the CARKit runtime image preflight check.
CARKIT_REQUIRE_RUNTIME=0 ./docker/run_jetson.sh

# Do not repair old generated-file ownership when running as host UID/GID.
CARKIT_FIX_PERMISSIONS_ON_START=0 ./docker/run_jetson.sh
```

## Workspace Build

Inside Docker:

```bash
./docker/install_carkit.sh osracer
# or
./docker/install_carkit.sh f1tenth
source install/setup.bash
```

`build_workspace.sh`:

1. Fetches vendored source repos with `carkit/setup_vendor_repos.sh`.
2. Sources `/opt/ros/humble/setup.bash`.
3. Runs `rosdep install` for packages under `carkit/`.
4. Builds with `colcon build --symlink-install`.
5. Prints available `carkit` packages.

It defaults to `BUILD_JOBS=1` and `PARALLEL_WORKERS=1` to avoid out-of-memory
failures on an 8 GB Jetson Orin Nano.

## Scripts

- `setup_jetson.sh`: one-command host setup, boot services, and fleet reporting.
- `run_jetson.sh`: pulls/runs the runtime image and mounts this checkout.
- `start_webui.sh`: serves the browser dashboard on port `8080`.
- `install_carkit.sh`: selects/fetches one chassis adapter and builds CARKit.
- `setup_osracer_device.sh`: configures OSRacer device aliases and its dedicated
  LakiBeam USB network on the Jetson host.
- `build_workspace.sh`: fetches vendor repos, installs ROS dependencies, and
  builds the workspace.
- `publish_image.sh`: maintainer helper to build, check, and push
  `ariiees/carkit:latest`.
- `test_workspace_in_docker.sh`: pulls/runs the image, builds this checkout,
  and checks launch arguments.

## Maintainer Publish

```bash
docker login
./docker/publish_image.sh
```

To build and run the image preflight check without pushing:

```bash
PUSH_IMAGE=0 ./docker/publish_image.sh
```

After a successful local publish, the local `ariiees/carkit:latest` image is
already the image that was pushed. Other machines should pull it with:

```bash
docker pull ariiees/carkit:latest
```

Never put Docker Hub credentials in this repository.

## Python ML Packages

`docker/Dockerfile.jetson` installs NVIDIA's JetPack PyTorch wheel, builds a
matching torchvision from source, installs ONNX and Ultralytics with pinned
constraints, keeps NumPy compatible with ROS 2 Humble, and keeps
`setuptools<80` so `colcon-core` remains compatible.

## Rosdep Notes

`build_workspace.sh` refreshes apt indexes before running `rosdep install`
because the Docker image removes apt lists to keep the image smaller. It skips
the `librealsense2` rosdep key by default because CARKit builds and installs
the native RealSense SDK in the Docker image.

Override skipped keys if needed:

```bash
ROSDEP_SKIP_KEYS="" ./docker/build_workspace.sh
```
