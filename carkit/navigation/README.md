# Navigation

The navigation module contains the supported Nav2 workflow:

- `carkit_slam`: SLAM Toolbox 2D occupancy-grid mapping
- `carkit_amcl`: AMCL, Nav2 parameters, and `/cmd_vel` to `/drive` bridge
- `carkit_navigation`: top-level mapping/navigation launch orchestration

All occupancy maps live in `/workspaces/CARKit/map` inside Docker, which is
the repository's top-level `map/` folder.

## Launch Model

`carkit_navigation` can start the LiDAR driver, `base_link -> laser` static
transform, odom TF broadcaster, RViz, SLAM Toolbox, and Nav2 depending on the
selected mode.

Nav2 publishes `/cmd_vel`; `twist_to_ackermann` converts it to `/drive`.
Autonomous driving routes `/drive` through `carkit_control_center`.
Mapping does not need the control center; use direct human control.

## Mapping

Start direct human control:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Start mapping:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/map/map
```

This creates `/workspaces/CARKit/map/map.yaml` and
`/workspaces/CARKit/map/map.pgm`.

## Localization And Planning

Start vehicle bringup with the legacy mux output remapped away from
`/ackermann_cmd`, then start the autonomous command arbiter:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

```bash
ros2 launch carkit_control_center control_center.launch.py
```

Start Nav2:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

In RViz:

1. Use **2D Pose Estimate** to initialize AMCL.
2. Wait for the particle cloud and pose to converge.
3. Send a **Nav2 Goal**.
4. Enter `AUTO_DRIVE` with the configured joystick button.

For AMCL/Nav2 debugging without the top-level orchestrator:

```bash
ros2 launch carkit_amcl nav2.launch.py \
  map:=/workspaces/CARKit/map/map.yaml \
  start_command_mux:=false
```

## Topic Flow

```text
Mapping:
  /scan + /odom -> SLAM Toolbox -> /map
  /joy -> joy_teleop -> /teleop -> ackermann_mux -> /ackermann_cmd
  /map -> map_saver_cli -> /workspaces/CARKit/map/*.yaml + *.pgm

Navigation:
  /scan + /odom + saved map + /initialpose -> AMCL/Nav2
  Nav2 planner/controller -> /cmd_vel
  /cmd_vel -> twist_to_ackermann -> /drive
  /drive -> carkit_control_center -> /ackermann_cmd

Cone obstacles:
  /behavior/cone_obstacles -> local/global Nav2 obstacle layers
```

## Common Arguments

- `mode:=mapping|navigation`: selects SLAM or AMCL/Nav2 workflow
- `map:=/workspaces/CARKit/map/<name>.yaml`: navigation map
- `start_lidar:=true|false`: starts or skips the SLLiDAR driver
- `start_static_tf:=true|false`: publishes or skips `base_link -> laser`
- `start_odom_tf:=true|false`: republishes `/odom` as odom TF
- `start_map_saver:=true|false`: starts map saver in mapping mode
- `start_cmd_bridge:=true|false`: starts `/cmd_vel` to `/drive` bridge
- `start_command_mux:=true|false`: starts the legacy Nav2 command mux. Use
  `false` for autonomous driving through `carkit_control_center`.
- `use_rviz:=true|false`: starts or skips RViz
- `mapping_rviz_config`: overrides mapping RViz configuration
- `planning_rviz_config`: overrides navigation RViz configuration

Show all arguments:

```bash
ros2 launch carkit_navigation navigation.launch.py --show-args
```

## Configuration

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- `carkit/navigation/carkit_slam/config/slam_toolbox_params.yaml`
- `carkit/navigation/carkit_slam/rviz/mapping.rviz`
- `carkit/navigation/carkit_amcl/rviz/localization.rviz`
- `carkit/navigation/carkit_navigation/rviz/navigation.rviz`
- `map/`: generated and included occupancy maps

The local and global obstacle layers use both `/scan` and
`/behavior/cone_obstacles`.

## Verify

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /drive --once
ros2 run tf2_ros tf2_echo map base_link
```

If AMCL does not publish, wait for the map to load and set the initial pose in
RViz. If heading drifts, calibrate the VESC wheel parameters in
`carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`.
