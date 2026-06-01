# Localization

Package: `carkit_navigation`

AMCL localization against a saved 2D occupancy grid map for Nav2 navigation.

## Launch

```bash
# Terminal 1: VESC odometry
ros2 launch carkit_human_control controller.launch.py start_av_stack:=false

# Terminal 2: Nav2 navigation (includes AMCL)
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

In RViz, use **2D Pose Estimate** to set the initial pose. AMCL starts publishing the `map → odom` TF once the initial pose is received.

## Config

`carkit/planning/carkit_navigation/config/nav2_params.yaml`

## Inputs

- `/scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`)
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) — set via RViz

## Outputs

- `map → odom` TF
- `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)

## Test

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

## Troubleshooting

- **`map` frame does not exist**: wait 10–15 seconds after launch before setting the initial pose
- **Heading drift**: calibrate VESC wheel parameters in `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`
