# Perception

Package: `carkit_perception`

YOLO-based camera perception for classroom demos.

## Launch

```bash
ros2 run carkit_perception perception_node
```

Override model or topics:

```bash
ros2 run carkit_perception perception_node --ros-args \
  -p model_path:=/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n.engine \
  -p image_topic:=/camera/camera/color/image_raw
```

## Test

Start RealSense first, then:

```bash
ros2 topic echo /yolo/detections --once
ros2 topic echo /yolo/inference_image --once
```

Inputs:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)

Outputs:

- `/yolo/inference_image` (`sensor_msgs/Image`)
- `/yolo/detections` (`std_msgs/String`)

Parameters:

- `model_path`
- `image_topic`
- `inference_image_topic`
- `detection_topic`
