# Mapping

Package: `carkit_slam`

2D occupancy grid mapping using SLAM Toolbox for Nav2 navigation.

## Physical Car

```bash
# Terminal 1: VESC odometry
ros2 launch carkit_human_control controller.launch.py start_av_stack:=false

# Terminal 2: SLAM mapping
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=mapping \
  start_static_tf:=false
```

Drive the car through the environment. Save the map when done:

```bash
# Terminal 3
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/carkit/mapping/carkit_slam/maps/map
```

## Inputs

- `/scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`)

## Output

- `map_3f.pgm` + `map_3f.yaml` saved to `carkit/mapping/carkit_slam/maps/`

## Config

`carkit/mapping/carkit_slam/config/slam_toolbox_params.yaml`

## Available Maps

- `maps/map_3f.yaml` — Fintech 3rd floor (default)
- `maps/map.yaml` — generic map
