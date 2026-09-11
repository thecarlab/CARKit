# ADA2: VESC, RPLidar S3M1-R2 and RealSense D435i

This is the `vesc_rplidar` hardware selection. It is separate from `osracer`
(OSRacer/LakiBeam/UVC) and `f1tenth` (legacy F1TENTH/Hokuyo/RealSense).
The legacy F1TENTH installer requires access to `thecarlab/ada_system`;
this selection instead fetches public `f1tenth/vesc` Humble sources pinned to
`1bc8251296abb3936da5f30821b6311d67e861b7`.

## Install on another Jetson

```bash
git clone --branch ada2026 https://github.com/thecarlab/CARKit.git ~/CARKit
cd ~/CARKit
./docker/setup_jetson.sh ADA2 vesc_rplidar
```

Use that car's ADA number. The setup installs the NVIDIA Docker runtime,
pulls the runtime image, creates `/dev/carkit_vesc` and `/dev/carkit_rplidar`,
and enables the WebUI and authenticated IP reporter at boot. Supply the
monitor token at its hidden prompt; later runs reuse `/etc/carkit-webmonitor.env`.
Sites hides stored secret values, so account access alone cannot retrieve an
existing token. An administrator can provision a separate vehicle token
without replacing the shared fleet credential.

This profile adds the compressed-image transport and serial-driver runtime
packages in a small local image, `carkit-vesc-rplidar:local`, built on top of
`ariiees/carkit:latest`. The run script selects it automatically for this
profile. The image automatically detects CMake's OpenCV version and checks
the libraries loaded by `cv_bridge` and `compressed_image_transport`. Missing
or incompatible libraries are rebuilt from pinned ROS Humble sources against
that detected OpenCV; compatible installations skip the rebuild. The WebUI
installer repeats this check after installing dependencies and clears stale
OpenCV paths before building the camera. This works with the OpenCV 4 version
provided by each supported base image, without hardcoding a minor version or
architecture-specific path. Python's `cv2` wheel version is separate from the
C++ library used by the camera.
Mixing Ubuntu's OpenCV 4.5 ROS binaries with the camera's OpenCV 4.8 causes
invalid JPEG images and enormous, spurious memory-allocation errors. Building
these two libraries takes about a minute on ADA2. The fix persists
across container restarts. OSRacer keeps using the original image.

For installation before a token is available:

```bash
CARKIT_SKIP_REPORTER=1 ./docker/setup_jetson.sh ADA2 vesc_rplidar
```

This skips reporter configuration; it does not enable reporting or disable an
existing reporter. Rerun normally when the token is available.

Open `http://<Jetson-IP>:8080` from a device with a route to the Jetson.
On ADA2 this is the eduroam network. Select **VESC + RPLidar + RealSense**,
click **Install / build selected chassis**, and wait for successful completion.
The first build can take tens of minutes on an 8 GB Jetson. The WebUI remains
available during compilation. Hardware/ROS launches are started from the
dashboard; only the container and WebUI automatically start at boot.

## Hardware configuration

[`vesc_rplidar.yaml`](../carkit/core/carkit_bringup/config/vesc_rplidar.yaml)
contains the serial devices and calibration. The S3M1-R2 uses **1,000,000 baud**
and **DenseBoost**, matching
[Slamtec's S3 launch configuration](https://github.com/Slamtec/sllidar_ros2/blob/34300099fadfc772965962dec837bf436706188f/launch/sllidar_s3_launch.py).
It is not an A2 lidar; the A2 serial rates will not work.
The RealSense publishes color at 640×480, 15 FPS; depth and IMU are disabled
for this 2D driving profile.
The ROS camera wrapper is built with GLSL disabled for this selection so it
can start from systemd without an X display. CUDA/TensorRT perception remains
available. This does not change the legacy F1TENTH camera build defaults.

The udev rules assume one VESC and one CP210x lidar adapter. If multiple
matching USB devices are connected, qualify the rules with their USB serial
numbers. The container uses host device access; ROS runs inside Docker.

Start a stationary check from the dashboard by selecting only **chassis** and
**sensors**, with camera and lidar checked. Leave control, planning, behavior
and perception unchecked for this first check. Inspect `/sensors/core`,
`/scan`, `/odom`, and `/camera/camera/color/image_raw/compressed`.

The installed VESC speed/steering gains, wheelbase, and lidar/camera mounting
transforms are nominal starting values, not measured ADA2 calibration.
Calibrate these before driving or relying on localization. The initial VESC
speed limit is ±2500 electrical RPM. Check the VESC firmware's command timeout
and motor/servo direction with wheels lifted before enabling driving.
Use the control component for the normal WebUI command watchdog and arbiter.
USB/telemetry responses confirm communication, not motor or steering operation.

Advanced container commands:

```bash
docker exec -it carkit bash
source install/setup.bash
# Stop any dashboard launch before opening the same hardware here.
ros2 launch carkit_bringup carkit.launch.py chassis:=vesc_rplidar \
  start_chassis:=true start_sensors:=true start_control:=false \
  start_planning:=false start_perception:=false start_behavior:=false
```

To repeat the build without the browser, run
`./docker/install_carkit.sh vesc_rplidar` inside the container.
Rebuild the selected chassis after changing its configuration.

For a repeatable check that observes hardware without publishing commands:

```bash
docker exec carkit bash -lc 'source install/setup.bash && python3 tools/check_vesc_rplidar_topics.py'
```

The upstream VESC odometry node waits for the first steering command before
publishing. This check does not send a steering command to satisfy that condition.
