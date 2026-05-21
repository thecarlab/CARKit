# Troubleshooting

- `colcon: command not found`: install `python3-colcon-common-extensions` or use the CARKit Docker image.
- `docker: command not found`: install Docker on the Jetson host before pulling `ariiees/carkit:latest`.
- `unknown or invalid runtime name: nvidia`: Docker is not configured with the NVIDIA runtime. The `docker/run_jetson.sh` script now falls back without `--runtime nvidia`; if GPU or TensorRT access fails, install and configure `nvidia-container-toolkit` on the Jetson host.
- Missing `realsense2_camera` or `sllidar_ros2`: run `./carkit/setup_vendor_repos.sh`.
- `fatal: detected dubious ownership` in a sensor driver repo: pull the latest `develop` branch and rerun `./docker/build_workspace.sh`. The setup script marks the cloned sensor driver folders as safe for the root user inside Docker.
- `git pull` refuses to overwrite `carkit/mapping/ndt_omp_ros2`: remove the old cloned NDT folder with `rm -rf carkit/mapping/ndt_omp_ros2`, then pull again. NDT is now vendored in CARKit instead of cloned by the setup script.
- `rosdep` cannot locate packages such as `python3-requests`, `python3-tqdm`, or `ros-humble-xacro`: pull the latest `develop` branch and rerun `./docker/build_workspace.sh`. The script now refreshes apt indexes before running `rosdep install`.
- `rosdep` cannot locate `ros-humble-librealsense2`: the build script skips the `librealsense2` rosdep key by default because that package is not consistently available on Jetson ROS apt repositories. If your local RealSense driver build needs a custom SDK install, install it on the image or pass a different `ROSDEP_SKIP_KEYS` value.
- `realsense2_camera` fails with `RealSense SDK 2.0 is missing`: rebuild the Docker image with the current `docker/Dockerfile.jetson`. The image now builds and installs `librealsense2` from source because the ROS apt package is not consistently available on Jetson.
- `package 'f1tenth_stack' not found`: `f1tenth_control.launch.py` is only for an external F1TENTH controller package. Use `ros2 launch carkit_bringup carkit_ada_control.launch.py` for CARKit control, or `ros2 launch carkit_bringup control_mux.launch.py` for only the command mux.
- LiDAR permission denied: confirm the device path, then add a persistent udev rule or temporarily run `sudo chmod 666 /dev/ttyUSB0`.
- `c++: fatal error: Killed signal terminated program cc1plus`: the Jetson likely ran out of memory during compilation. Pull the latest `develop` branch and rerun `./docker/build_workspace.sh`; the default build now uses one compiler job and one colcon worker. Close RViz/browser windows during build, and add swap if the failure continues.
- RViz does not open in Docker: run `xhost +si:localuser:root`, mount `/tmp/.X11-unix`, and pass `DISPLAY`.
- YOLO model load fails: verify the `model_path` parameter points to an installed model file. TensorRT engine files may be hardware/runtime specific.
- Docker build fails with `Cannot uninstall sympy 1.9`: rebuild with the current `docker/Dockerfile.jetson`. The image installs `ultralytics` with pip constraints and `--ignore-installed` so pip does not try to remove apt-owned Python packages from the Jetson base image.
- Docker build warns that `colcon-core` requires `setuptools<80`: rebuild with the current `docker/Dockerfile.jetson`. The image pins `setuptools<80` before and during the `ultralytics` install.
- `rosdep` cannot resolve another dependency: update rosdep first with `rosdep update`; unresolved external package keys should be marked TODO rather than guessed.
