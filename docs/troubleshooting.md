# Troubleshooting

- `colcon: command not found`: install `python3-colcon-common-extensions` or use the CARKit Docker image.
- `docker: command not found`: install Docker on the Jetson host before pulling `ariiees/carkit:latest`.
- Missing `realsense2_camera` or `sllidar_ros2`: run `./carkit/setup_vendor_repos.sh`.
- LiDAR permission denied: confirm the device path, then add a persistent udev rule or temporarily run `sudo chmod 666 /dev/ttyUSB0`.
- RViz does not open in Docker: run `xhost +si:localuser:root`, mount `/tmp/.X11-unix`, and pass `DISPLAY`.
- YOLO model load fails: verify the `model_path` parameter points to an installed model file. TensorRT engine files may be hardware/runtime specific.
- Docker build fails with `Cannot uninstall sympy 1.9`: rebuild with the current `docker/Dockerfile.jetson`. The image installs `ultralytics` with pip constraints and `--ignore-installed` so pip does not try to remove apt-owned Python packages from the Jetson base image.
- Docker build warns that `colcon-core` requires `setuptools<80`: rebuild with the current `docker/Dockerfile.jetson`. The image pins `setuptools<80` before and during the `ultralytics` install.
- `rosdep` cannot resolve a dependency: update rosdep first with `rosdep update`; unresolved external package keys should be marked TODO rather than guessed.
