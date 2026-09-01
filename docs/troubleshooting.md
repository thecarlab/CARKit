# Troubleshooting

- `colcon: command not found`: install `python3-colcon-common-extensions` or use the CARKit Docker image.
- `docker: command not found`: install Docker on the Jetson host before pulling `ariiees/carkit:latest`.
- `unknown or invalid runtime name: nvidia`: Docker is not configured with the NVIDIA runtime. The `docker/run_jetson.sh` script now falls back without `--runtime nvidia`; if GPU or TensorRT access fails, install and configure `nvidia-container-toolkit` on the Jetson host.
- Files created by Docker show a lock icon on the host: pull the latest `develop` branch and start with `./docker/run_jetson.sh`. The script now runs the container as your host UID/GID and repairs ownership for common generated folders on startup. Keep `CARKIT_RUN_AS_ROOT` unset for normal development.
- Missing `realsense2_camera` or `sllidar_ros2`: run `./carkit/setup_vendor_repos.sh`.
- `fatal: detected dubious ownership` in a sensor driver repo: pull the latest `develop` branch and rerun `./docker/build_workspace.sh`. The setup script marks the cloned sensor driver folders as safe for the root user inside Docker.
- `rosdep` cannot locate packages such as `python3-requests`, `python3-tqdm`, or `ros-humble-xacro`: pull the latest `develop` branch and rerun `./docker/build_workspace.sh`. The script now refreshes apt indexes before running `rosdep install`.
- `rosdep` cannot locate `ros-humble-librealsense2`: the build script skips the `librealsense2` rosdep key by default because that package is not consistently available on Jetson ROS apt repositories. If your local RealSense driver build needs a custom SDK install, install it on the image or pass a different `ROSDEP_SKIP_KEYS` value.
- `realsense2_camera` fails with `RealSense SDK 2.0 is missing`: rebuild the Docker image with the current `docker/Dockerfile.jetson`. The image now builds and installs `librealsense2` from source because the ROS apt package is not consistently available on Jetson.
- `package 'osracer_base' not found`: rerun `./docker/build_workspace.sh`, source `install/setup.bash`, and verify `carkit/vehicle/osracer` is present.
- Need manual vehicle control: use `ros2 launch carkit_human_control joystick.launch.py` for the joystick/OSRacer stack.
- `/dev/osrbot_base` is missing or is a regular file: stop the old container, run `./docker/setup_osracer_device.sh` on the host, and restart with `./docker/run_jetson.sh`. Never bind-mount a possibly missing `/dev/ttyACM0` path to the alias.
- OSRacer logs `Chassis firmware identity unavailable`: this controller uses the legacy protocol. Launch through CARKit (default `protocol_mode:=legacy`) or pass that argument explicitly. Use `modern` only with Proto 1.1 firmware.
- OSRacer connects but does not move: confirm `ros2 topic info /ackermann_cmd -v` reports `ackermann_msgs/msg/AckermannDriveStamped`, check that the driver logs `Serial control is active`, and publish a low-speed test with the driven path clear.
- LiDAR permission denied: confirm the device path, then add a persistent udev rule or temporarily run `sudo chmod 666 /dev/ttyUSB0`.
- `c++: fatal error: Killed signal terminated program cc1plus`: the Jetson likely ran out of memory during compilation. Rerun `./docker/build_workspace.sh`; the default build uses one compiler job and one colcon worker. Close extra browser windows during build, and add swap if the failure continues.
- WebUI cannot connect: confirm the container is running, then open `http://<jetson-ip>:8080`. The Docker runner uses host networking, so no extra port mapping is needed.

## Map Missing In The WebUI

If the WebUI receives `/scan` but does not display the occupancy map, check the
Nav2 lifecycle state first. Seeing `/map` in the topic list is not enough: an
inactive map server advertises the topic without publishing the saved map.

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map -v
ros2 topic echo /map --once \
  --qos-durability transient_local \
  --qos-reliability reliable
```

Both lifecycle nodes should report `active [3]`, and the final command should
return an `OccupancyGrid` with `header.frame_id: map`. Use **Republish map** in
the Overview after changing the map selection.

If `map_server` is `inactive`, `amcl` is `unconfigured`, or the map echo times
out, check for processes left behind by earlier navigation launches:

```bash
ros2 node list 2>/dev/null | sort | uniq -cd
ps -ef | grep -E \
  'navigation.launch|map_server|amcl|lifecycle_manager|odom_tf_broadcaster' \
  | grep -v grep
```

Stop every previous navigation launch with `Ctrl-C` and wait for its child
processes to exit. If orphaned navigation processes remain, terminate those
processes or restart the CARKit container, then launch one navigation stack:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  map:=/workspaces/CARKit/map/map.yaml
```

Confirm the launch output says that `map_server` loaded the YAML and PGM,
`amcl` received the map, and the localization lifecycle manager reports
`Managed nodes are active`.

After a healthy launch, localization remains incomplete until AMCL has an
initial pose. This is expected: AMCL does not publish `map -> odom` before it
has one. Select the initial-pose tool in the Overview and place the vehicle on
the map; the laser scan should then align with the map.

- Stop-sign or traffic-light locations are missing in the WebUI: confirm `/behavior/stop_sign_markers` and `/behavior/traffic_light_markers` exist. A marker is published only after behavior receives a qualifying `/yolo/detections_2d`, a matching `/scan` return, and a valid transform from the scan frame to `map`.
- YOLO model load fails: verify the `model_path` parameter points to an installed model file. TensorRT engine files may be hardware/runtime specific.
- Docker build fails with `Cannot uninstall sympy 1.9`: rebuild with the current `docker/Dockerfile.jetson`. The image installs `ultralytics` with pip constraints and `--ignore-installed` so pip does not try to remove apt-owned Python packages from the Jetson base image.
- Docker build warns that `colcon-core` requires `setuptools<80`: rebuild with the current `docker/Dockerfile.jetson`. The image pins `setuptools<80` before and during the `ultralytics` install.
- `rosdep` cannot resolve another dependency: update rosdep first with `rosdep update`; unresolved external package keys should be marked TODO rather than guessed.
