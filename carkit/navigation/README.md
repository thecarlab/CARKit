# TODO
- [ ] Check the whole mapping code and localization & planning code, mapping should launch its own RViz config, localization & planning should have another RViz config. Or you can make all of them use the same RViz config, just make it clean and easy to modify.
- [ ] Try to see if you can replace the control with a different control algorithm. Which one would be better? Use the best-performing one.

# Navigation

The `navigation` module contains the complete supported Nav2 workflow:

- `carkit_slam`: SLAM Toolbox 2D occupancy-grid mapping
- `carkit_amcl`: AMCL localization, Nav2 configuration, and Ackermann bridge
- `carkit_navigation`: mapping and navigation launch orchestration

All occupancy maps are stored in `/workspaces/CARKit/map` inside Docker,
which is the repository's top-level `map/` folder. Do not create package-local
map folders.

## Mapping

Start VESC odometry:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Start Nav2 mapping:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping \
  start_static_tf:=false
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/map/map
```

This creates `/workspaces/CARKit/map/map.yaml` and
`/workspaces/CARKit/map/map.pgm`.

## Localization And Planning

Start VESC odometry:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Start Nav2:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

In RViz:

1. Use **2D Pose Estimate** to initialize AMCL.
2. Wait for the particle cloud and pose to converge.
3. Send a **Nav2 Goal**.

For AMCL localization debugging without the navigation orchestrator:

```bash
ros2 launch carkit_amcl nav2.launch.py \
  map:=/workspaces/CARKit/map/map.yaml
```

This direct launch uses the localization RViz configuration. The combined
`carkit_navigation` launch uses the planning RViz configuration in navigation
mode.

## Topic Flow

```text
Mapping:
  /scan + /odom -> SLAM Toolbox -> /map
  /map -> map_saver_cli -> /workspaces/CARKit/map/*.yaml + *.pgm

Navigation:
  /scan + /odom + saved map + /initialpose -> AMCL/Nav2
  Nav2 planner/controller -> /cmd_vel
  /cmd_vel -> twist_to_ackermann -> /drive
  /drive -> ackermann_mux -> /ackermann_cmd
```

## Common Arguments

- `mode:=mapping|navigation`: selects the Nav2 workflow
- `map:=/workspaces/CARKit/map/<name>.yaml`: selects the navigation map
- `start_lidar:=true|false`: starts or skips the SLLiDAR driver
- `start_static_tf:=true|false`: publishes or skips `base_link -> laser`
- `start_odom_tf:=true|false`: republishes `/odom` as an odometry TF
- `start_command_mux:=true|false`: starts or skips the Ackermann mux
- `use_rviz:=true|false`: starts or skips RViz
- `mapping_rviz_config`: overrides the mapping RViz configuration
- `planning_rviz_config`: overrides the Nav2 planning RViz configuration

Show all arguments:

```bash
ros2 launch carkit_navigation navigation.launch.py --show-args
```

## Configuration

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- `carkit/navigation/carkit_slam/config/slam_toolbox_params.yaml`
- `carkit/navigation/carkit_slam/rviz/mapping.rviz`
- `carkit/navigation/carkit_amcl/rviz/localization.rviz`
- `carkit/navigation/carkit_navigation/rviz/planning.rviz`
- `map/`: the only location for generated and included occupancy maps

## Verify

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

If AMCL does not publish, wait for the map to load and set the initial pose in
RViz. If heading drifts, calibrate the VESC wheel parameters in
`carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`.
