# Tools

Package: `carkit_tools`

Classroom utility and demo nodes.

## Launch Object Tracking Tools

```bash
ros2 launch carkit_tools object_tracking.launch.py target_object_type:=book
```

Test:

```bash
ros2 topic echo /object_position --once
ros2 topic echo /object_path --once
```

## Interactive Waypoints

```bash
ros2 launch carkit_tools interactive_waypoints.launch.py
```

Test:

```bash
ros2 topic echo /follow_path --once
```

## Other Utilities

```bash
ros2 run carkit_tools cmd_vel_to_ackermann
ros2 run carkit_tools distance_metrics
ros2 run carkit_tools object_angle
ros2 run carkit_tools path_tracker
```
