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
- `package 'f1tenth_stack' not found`: rebuild the workspace from the latest `develop` branch. CARKit now vendors the F1TENTH/ADA control stack under `carkit/vehicle/f1tenth_system`.
- Need manual vehicle control: use `ros2 launch carkit_human_control joystick.launch.py` for the joystick/VESC stack.
- VESC does not connect: confirm the serial device configured in `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml` exists inside Docker. `./docker/run_jetson.sh` mounts `/dev`, but the host may still need udev permissions.
- LiDAR permission denied: confirm the device path, then add a persistent udev rule or temporarily run `sudo chmod 666 /dev/ttyUSB0`.
- `c++: fatal error: Killed signal terminated program cc1plus`: the Jetson likely ran out of memory during compilation. Pull the latest `develop` branch and rerun `./docker/build_workspace.sh`; the default build now uses one compiler job and one colcon worker. Close RViz/browser windows during build, and add swap if the failure continues.
- RViz does not open in Docker: run `xhost +si:localuser:root`, mount `/tmp/.X11-unix`, and pass `DISPLAY`.
- Foxglove cannot connect: confirm navigation was started with `visualization:=foxglove`, then connect to `ws://<jetson-ip>:8765`. The Docker runner uses host networking, so no extra port mapping is needed.

## Map Missing In Foxglove

If Foxglove receives `/scan` but does not display the occupancy map, and the
3D panel marks the `map` fixed/display frame in red, check the Nav2 lifecycle
state first. Seeing `/map` in the topic list is not enough: an inactive map
server advertises the topic without publishing the saved map.

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map -v
ros2 topic echo /map --once \
  --qos-durability transient_local \
  --qos-reliability reliable
```

Both lifecycle nodes should report `active [3]`, and the final command should
return an `OccupancyGrid` with `header.frame_id: map`. The `/map` publisher and
Foxglove subscriber should both use reliable, transient-local QoS.

If `map_server` is `inactive`, `amcl` is `unconfigured`, or the map echo times
out, check for processes left behind by earlier navigation launches:

```bash
ros2 node list 2>/dev/null | sort | uniq -cd
ps -ef | grep -E \
  'navigation.launch|map_server|amcl|lifecycle_manager|foxglove_bridge|odom_tf_broadcaster' \
  | grep -v grep
```

Stop every previous navigation launch with `Ctrl-C` and wait for its child
processes to exit. If orphaned navigation processes remain, terminate those
processes or restart the CARKit container, then launch one navigation stack:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  map:=/workspaces/CARKit/map/map.yaml \
  visualization:=foxglove
```

Confirm the launch output says that `map_server` loaded the YAML and PGM,
`amcl` received the map, and the localization lifecycle manager reports
`Managed nodes are active`.

After a healthy launch, the `map` frame may remain red until AMCL is
initialized. This is expected: AMCL does not publish `map -> odom` before it
has an initial pose. In the Foxglove 3D panel, enable `/map`, select the
**2D Pose Estimate** tool, and place the vehicle on the map. The transform
warning should then clear and the laser scan should align with the map.

- Stop-sign or traffic-light locations are missing in Foxglove: import the latest `docs/carkit_foxglove_layout.json` and confirm `/behavior/stop_sign_markers` and `/behavior/traffic_light_markers` appear in the connection. A marker is published only after the behavior node receives a qualifying `/yolo/detections_2d`, a matching `/scan` return, and a valid transform from the scan frame to `map`.
- YOLO model load fails: verify the `model_path` parameter points to an installed model file. TensorRT engine files may be hardware/runtime specific.
- Docker build fails with `Cannot uninstall sympy 1.9`: rebuild with the current `docker/Dockerfile.jetson`. The image installs `ultralytics` with pip constraints and `--ignore-installed` so pip does not try to remove apt-owned Python packages from the Jetson base image.
- Docker build warns that `colcon-core` requires `setuptools<80`: rebuild with the current `docker/Dockerfile.jetson`. The image pins `setuptools<80` before and during the `ultralytics` install.
- `rosdep` cannot resolve another dependency: update rosdep first with `rosdep update`; unresolved external package keys should be marked TODO rather than guessed.
