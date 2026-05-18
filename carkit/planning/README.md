# Planning And Behaviors

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
