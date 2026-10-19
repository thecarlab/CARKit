# Docker

CARKit uses `ariiees/carkit:latest` as the single Jetson runtime image. The
image contains ROS 2 Humble Desktop, Nav2, RViz/rqt, build tools, RealSense
SDK, TensorRT/CUDA Python ML dependencies, and the system
dependencies needed to build the mounted CARKit workspace.

The image does not contain a baked copy of this repository. `run_jetson.sh`
mounts the checkout at `/workspaces/CARKit`.

## Installation

Follow the [main installation guide](../README.md#installation) for network
prerequisites, cloning, and dashboard access. Run host setup with the actual
vehicle ID and one hardware selector:

```bash
# OSRacer
./docker/setup_jetson.sh ADA5 osracer
# F1TENTH (VESC / RPLidar / RealSense), instead of the command above
./docker/setup_jetson.sh ADA2 vesc_rplidar
```

Setup prompts for a monitor token when none is saved, prepares the host and
container, builds the selected chassis, and restarts the dashboard with the
new build. No additional browser installation click is needed. A build failure
stops setup before its completion message. Vehicle operation is not launched.
See the [F1TENTH hardware guide](../docs/vesc_rplidar_setup.md) for device aliases
and calibration, and the [reporter guide](../webmonitor/README.md#replacing-a-vehicle-token)
for updating an existing token.

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

The reporter updates every minute while connected. When the vehicle has no internet
access, it tries every five minutes and stops after 20 consecutive failures.
Restart `carkit-webmonitor.service` to begin a new retry window without
rebooting the vehicle.

After setup completes, choose the desired course profile and components in
the dashboard. Use `./docker/run_jetson.sh bash` when a terminal is
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
default route. Because the adapter's MAC-derived `enx*` name can change across
cold boots, `carkit.service` reruns the identity-based network helper before
starting the container. Run the setup again after changing a rule or if a
stale regular file exists at either device path, then restart the container.

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

- `setup_jetson.sh`: one-command host setup, chassis build, boot services, and fleet reporting.
- `configure_lakibeam_network.sh`: rebinds the Richbeam adapter and repairs its
  direct route before every OSRacer container start.
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

The WebUI and reporter run as system services without a display or desktop login.
At startup, routing and DNS get a two-minute grace period before normal upload
retries begin; container readiness and clock synchronization are always required.

CARKit setup now enables the host NTP provider, installing `systemd-timesyncd`
when none is available. Each CARKit service start re-enables synchronization;
the container shares the host clock and uses the host timezone by default.
The reporter starts with `carkit.service` and follows its stop/restart lifecycle.
Before every upload it verifies that the named `carkit` container is running,
the local WebUI landing page responds, and `timedatectl` confirms the clock is
synchronized. These readiness checks repeat every five seconds without consuming
upload retries. If time cannot synchronize, the WebUI can still run, but reporting
waits; inspect `timedatectl status` and the host NTP service in that case.

### OSRacer Wi-Fi assignment

`setup_jetson.sh ... osracer` binds the saved OSR Wi-Fi profile to the onboard
adapter and the saved eduroam profiles to the USB adapter. Other saved Wi-Fi
profiles have autoconnect disabled, preventing automatic fallback to another
network. This is NetworkManager connection policy; administrators can still
explicitly change or create profiles.

Save credentials for both networks before setup. The helper detects one onboard
and one USB Wi-Fi adapter from sysfs, and selects the unique saved `OSR-*` SSID.
It refuses missing or ambiguous adapters/profiles. For multiple saved OSR
networks, run the helper explicitly with `--osr-ssid OSR-F912`.

```bash
python3 docker/configure_osracer_wifi.py          # preview
sudo python3 docker/configure_osracer_wifi.py --apply
```

Applying saves previous profile settings under `/var/lib/carkit/network-backups`
and changes only saved connection settings; it does not disconnect or activate
an interface. Existing active connections must already match their assigned
networks. The reporter is pinned to the USB adapter through a systemd drop-in
and reports no substitute address if that adapter is unavailable. Restart only
`carkit-webmonitor.service` after applying this helper to a running installation.

OSRacer setup gives USB eduroam IPv4 routes metric 50 and permits its default
route, so it takes priority over the usual metric-100 Ethernet default. OSR
profile routing is unchanged. Existing installations can apply the saved USB
settings with `sudo nmcli device reapply <usb-interface>` without reconnecting
the onboard adapter.
