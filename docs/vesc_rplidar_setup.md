# ADA2: VESC, RPLidar S3M1-R2 and RealSense D435i

F1TENTH uses the `vesc_rplidar` selector for its VESC, RPLidar, and RealSense
hardware. Its installer fetches public `f1tenth/vesc` sources pinned to
`1bc8251296abb3936da5f30821b6311d67e861b7`.

## Install on another Jetson

```bash
git clone --branch ada2026 https://github.com/thecarlab/CARKit.git ~/CARKit
cd ~/CARKit
./docker/setup_jetson.sh ADA2 vesc_rplidar
```

Use that car's ADA number. The setup installs the NVIDIA Docker runtime,
pulls the runtime image, creates `/dev/carkit_vesc` and `/dev/carkit_rplidar`,
builds the selected chassis, and enables the WebUI and authenticated IP reporter
at boot. Supply the
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

Forward ports 8080 and 9090 over SSH and open `http://127.0.0.1:8080`.
Sign in with the private `.carkit/access.key` on the vehicle computer, then select
**Take control**. See [WebUI access](webui_access.md) for explicit network binding.
Use the eduroam address for remote access. Host setup already builds this
configuration; **Install / build selected chassis** is available for subsequent
manual rebuilds or hardware changes. The initial build can take tens of minutes
on an 8 GB Jetson. Wait for the setup completion message before operating. Hardware/ROS launches are started from the
dashboard; only the container and WebUI automatically start at boot.

## Hardware configuration

[`vesc_rplidar.yaml`](../carkit/core/carkit_bringup/config/hardware/vesc_rplidar.yaml)
contains the serial devices and calibration. The S3M1-R2 uses **1,000,000 baud**
and **DenseBoost**, matching
[Slamtec's S3 launch configuration](https://github.com/Slamtec/sllidar_ros2/blob/34300099fadfc772965962dec837bf436706188f/launch/sllidar_s3_launch.py).
It is not an A2 lidar; the A2 serial rates will not work.
The RealSense publishes color at 640×480, 15 FPS, with gyroscope and accelerometer
streams enabled; depth remains disabled for this 2D driving profile. Raw IMU data
is available on `/camera/camera/gyro/sample` and `/camera/camera/accel/sample`.
Enabling these streams does not fuse IMU measurements into odometry: VESC odometry
still estimates yaw from motor speed and commanded steering.
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
speed limit is ±0.5 m/s (±2307 electrical RPM at the bundled gain). Check the VESC firmware's command timeout
and motor/servo direction with wheels lifted before enabling driving.
The command watchdog and arbiter remain active even in a manual-only session.
WASD keyboard driving is the default; Caps Lock toggles autonomous driving when
the autonomous control component is enabled.
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

Integrated VESC odometry uses measured RPM and the RealSense gyro; it does not
need a steering command. Keep the car still for the two-second IMU bias check
after startup. Missing gyro feedback or mounting TF pauses `/odom` rather than
substituting a commanded turn. Camera video may be disabled while the IMU stays
enabled. See [localization and feedback](hardware_profiles.md#localization-and-feedback).

### Build inference engines on the vehicle

If TensorRT warns that an engine was built for a different device model, stop the
running stack and export both active models inside the vehicle's container. Keep
camera resolution, inference image size, and model weights unchanged when comparing
performance. Back up the existing engines and their JSON metadata first.

```bash
docker exec -it carkit bash
cd /workspaces/CARKit
python3 carkit/perception/carkit_perception/util/export_fp16_engine.py \
  --source carkit/perception/carkit_perception/models/yolo11n.pt \
  --name yolo11n_fp16.engine --workspace-gib 1
python3 carkit/perception/carkit_perception/util/export_fp16_engine.py \
  --source carkit/perception/carkit_perception/models/traffic_sign_1.pt \
  --name traffic_sign_1_fp16.engine --workspace-gib 1
```

The exporter writes batch-one FP16 engines at 448 pixels with matching hashes,
runtime versions, GPU details, and workspace limits in their JSON metadata. The
workspace limit bounds TensorRT tactic selection memory; total export memory is
higher. Export each model sequentially on the 8 GB Jetson.

Perception preserves the phase of its target schedule: a 15 FPS camera can feed
10 inference slots per second instead of being rounded down to every other frame.
Missed slots are skipped after a stall. Fresh completed inference is published
immediately instead of waiting for a second output timer; expired outputs are
still discarded.
Measure distinct source timestamps and source-to-receipt latency, not repeated
publication counts, when checking the resulting rate.
