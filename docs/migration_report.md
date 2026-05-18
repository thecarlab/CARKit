# Migration Report

Moved and renamed:

- `src/ada` -> `carkit/tools/carkit_tools` and full-stack launch -> `carkit/bringup/carkit_bringup`.
- `src/perception` -> `carkit/perception/carkit_perception`.
- `src/util` -> `carkit/sensors/carkit_sensor_transforms`; stop sign behavior moved to `carkit/planning/carkit_behaviors`.
- `src/control_center` -> `carkit/control/carkit_command_mux`.
- `src/pure_pursuit_controller` -> `carkit/control/carkit_pure_pursuit`.
- `src/lidar_localization_ros2` -> `carkit/localization/carkit_lidar_localization`.
- `lidarslam_msgs` -> `carkit/interfaces/carkit_lidarslam_msgs`.
- SLAM packages moved under `carkit/mapping`.
- `map/`, `rviz/`, and `waypoints/` moved under `carkit/bringup`.

Removed from active source:

- Tracked `__pycache__` and `*.pyc` files.
- Backup header `lidar_localization_component.hpp.orig`.

External setup:

- RealSense and SLLiDAR sensor drivers are listed in `carkit/vendor.repos`.
- Run `./carkit/setup_vendor_repos.sh` to clone the sensor drivers into the required local paths.
- `ndt_omp_ros2` is vendored under `carkit/mapping/ndt_omp_ros2` without upstream `.git` metadata because CARKit carries local mapping integration changes.
- `install_env.sh` and `build_ws.sh` were removed. CARKit now uses the Docker-only workflow: pull `ariiees/carkit:latest`, mount the repo, and run `./docker/build_workspace.sh` inside the container.
