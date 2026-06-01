# CARKit Navigation

Package: `carkit_navigation`

Nav2 autonomous navigation using SLAM Toolbox (mapping) and AMCL (localization).

## Step 1: Build a Map

```bash
# Terminal 1: VESC odometry
ros2 launch carkit_human_control controller.launch.py start_av_stack:=false

# Terminal 2: SLAM mapping
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=mapping \
  start_static_tf:=false

# Terminal 3: Save map when done
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/carkit/planning/carkit_navigation/maps/map
```

## Step 2: Navigate

```bash
# Terminal 1: VESC odometry
ros2 launch carkit_human_control controller.launch.py start_av_stack:=false

# Terminal 2: Nav2 navigation
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

In RViz: set **2D Pose Estimate**, wait for particles to converge, then send a **Nav2 Goal**.

## Key Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `navigation` | `mapping` or `navigation` |
| `map` | `maps/map.yaml` | Map file for navigation |
| `start_static_tf` | `true` | Set `false` if VESC launch already publishes TF |
| `start_command_mux` | `true` | Set `false` if VESC launch already starts mux |

## Topics

Inputs: `/scan`, `/odom`, `/initialpose`, `/goal_pose`

Outputs: `/map` (mapping), `/drive`, `/ackermann_cmd` (navigation)

## Available Maps

- `maps/map.yaml` — default map
- `maps/map_3f.yaml` — Fintech 3rd floor
