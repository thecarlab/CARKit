# Tools

Package: `carkit_tools`

These are classroom utilities and demos. They are not part of the active
Nav2-control-center autonomy path unless launched explicitly.

## Interactive Waypoints

```bash
ros2 launch carkit_tools interactive_waypoints.launch.py
```

The node waits for `/pcl_pose`, creates an interactive marker in `map`, and
publishes:

- `/follow_path` (`nav_msgs/Path`)

Verify:

```bash
ros2 topic echo /follow_path --once
```

## Legacy Object Tracking Demo

```bash
ros2 launch carkit_tools object_tracking.launch.py target_object_type:=book
```

This launch starts:

- `object_position`, which subscribes to legacy string `/yolo/detections` and
  aligned depth, then publishes `/object_position`
- `path_tracker`, which subscribes to `/object_position` and publishes
  `/object_path` and `/object_waypoints`

Current `carkit_perception` publishes typed `/yolo/detections_2d`, not the
legacy string `/yolo/detections`, so this demo needs a compatible legacy YOLO
publisher or a future adapter before it can consume current perception output.

Verify when a compatible detection source is running:

```bash
ros2 topic echo /object_position --once
ros2 topic echo /object_path --once
ros2 topic echo /object_waypoints --once
```

## Console Scripts

```bash
ros2 run carkit_tools interactive_waypoints
ros2 run carkit_tools demo1
ros2 run carkit_tools demo2
ros2 run carkit_tools distance_metrics
ros2 run carkit_tools object_angle
ros2 run carkit_tools object_position
ros2 run carkit_tools path_tracker
ros2 run carkit_tools cmd_vel_to_ackermann
```
