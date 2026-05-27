# Planning And Behaviors

## Nav2 AV Workflow

Package: `carkit_navigation`

CARKit's Nav2 workflow is additive to the existing NDT/PCD and pure-pursuit
modules. It uses direct 2D SLAM Toolbox mapping and saved-map Nav2 navigation.

Mapping:

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py mode:=mapping
ros2 run nav2_map_server map_saver_cli -f /workspaces/CARKit/carkit/planning/carkit_navigation/maps/map
```

Navigation:

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

Flow:

- `/scan` + `/odom` -> SLAM Toolbox or Nav2 AMCL
- RViz `2D Pose Estimate` -> `/initialpose`
- RViz Navigation2 Goal -> Nav2 planner/controller
- Nav2 `/cmd_vel` -> `twist_to_ackermann` -> `/drive`
- `/drive` + `/stopsign_cmd` -> `ackermann_mux` -> `/ackermann_cmd`

## Stop Sign Behavior

Package: `carkit_behaviors`

The current behavior module handles stop sign detections from perception.

## Launch

```bash
ros2 run carkit_behaviors stop_sign_behavior_node
```

## Test

Publish a simulated detection:

```bash
ros2 topic pub /yolo/detections std_msgs/msg/String \
  "{data: 'stop sign [10.0, 10.0, 100.0, 100.0] (0.95)'}" --rate 10
```

In another terminal:

```bash
ros2 topic echo /stopsign_cmd
```

Inputs:

- `/yolo/detections` (`std_msgs/String`)

Outputs:

- `/stopsign_cmd` (`ackermann_msgs/AckermannDriveStamped`)

Parameters:

- `stop_duration_sec`
- `approach_speed`
- `cmd_topic`
- `detections_topic`
- `frame_threshold`
