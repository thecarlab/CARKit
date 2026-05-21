# Migration Report

Moved and renamed:

- `src/ada` -> `carkit/tools/carkit_tools` and full-stack launch -> `carkit/bringup/carkit_bringup`.
- `src/perception` -> `carkit/perception/carkit_perception`.
- `src/util` -> `carkit/sensors/carkit_sensor_transforms`; stop sign behavior moved to `carkit/planning/carkit_behaviors`.
- F1TENTH `ackermann_mux` remains the command mux under `carkit/vehicle/f1tenth_system`.
- `src/pure_pursuit_controller` -> `carkit/control/carkit_pure_pursuit`.
- `src/lidar_localization_ros2` -> `carkit/localization/carkit_lidar_localization`.
- `lidarslam_msgs` -> `carkit/interfaces/carkit_lidarslam_msgs`.
- SLAM packages moved under `carkit/mapping`.
- The F1TENTH/ADA vehicle control workspace previously run from `ariiees/ada:foxy-f1tenth` and `$HOME/ada_system` is now vendored under `carkit/vehicle/f1tenth_system`.
- CARKit vehicle launch now provides two entry points: `controller.launch.py` for gamepad/controller driving and `keyboard.launch.py` for keyboard driving.
- `map/`, `rviz/`, and `waypoints/` moved under `carkit/bringup`.

Removed from active source:

- Tracked `__pycache__` and `*.pyc` files.
- Backup header `lidar_localization_component.hpp.orig`.
- Old CARKit bringup wrappers `control_mux.launch.py` and `f1tenth_control.launch.py`; human control now lives in `carkit_human_control`.
- Old vehicle helper launch files `controller_only.launch.py` and `ackermann_input.launch.py`, plus the temporary `ackermann_relay` node.
- Upstream CI folders and the old standalone F1TENTH container helper script from the vendored control tree.

External setup:

- RealSense and SLLiDAR sensor drivers are listed in `carkit/vendor.repos`.
- Run `./carkit/setup_vendor_repos.sh` to clone the sensor drivers into the required local paths.
- `ndt_omp_ros2` is vendored under `carkit/mapping/ndt_omp_ros2` without upstream `.git` metadata because CARKit carries local mapping integration changes.
- `f1tenth_system` is vendored under `carkit/vehicle/f1tenth_system` without upstream `.git` metadata because CARKit now builds the former control Docker workspace in the same ROS 2 Humble Docker environment.
- `install_env.sh` and `build_ws.sh` were removed. CARKit now uses the Docker-only workflow: pull `ariiees/carkit:latest`, mount the repo, and run `./docker/build_workspace.sh` inside the container.
