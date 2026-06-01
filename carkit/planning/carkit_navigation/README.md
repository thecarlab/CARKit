# CARKit Navigation

Package: `carkit_navigation`

Clean Nav2 workflow for CARKit. This package does not replace the existing
NDT/PCD localization, mapping, pure pursuit, or demo modules.

## Mapping

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py mode:=mapping
ros2 run nav2_map_server map_saver_cli -f /workspaces/CARKit/carkit/planning/carkit_navigation/maps/map
```

Mapping uses `/scan`, `/odom`, `base_link -> laser` TF, and SLAM Toolbox.
On the physical car, start `carkit_human_control controller.launch.py
start_av_stack:=false` to provide VESC odometry.

## Navigation

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

Navigation uses the saved 2D map, AMCL, Nav2, `/cmd_vel`, and the
Twist-to-Ackermann bridge to publish `/drive`.

If another launch already starts the Ackermann mux or laser TF, disable the
duplicates:

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

## Topics

Inputs:

- `/scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`)
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)

Outputs:

- `/map` (`nav_msgs/OccupancyGrid`) in mapping mode
- `/cmd_vel` (`geometry_msgs/Twist`) from Nav2
- `/drive` (`ackermann_msgs/AckermannDriveStamped`)
