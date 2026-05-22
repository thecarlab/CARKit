# Perception

Package: `carkit_perception`

YOLO-based camera perception for classroom demos.

## Launch

Start the RealSense color camera first:

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true align_depth.enable:=true enable_sync:=true
```

Check the input image:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/image_raw --once
```

Then launch perception. This starts the YOLO node and RViz with the annotated
image view loaded:

```bash
ros2 launch carkit_perception perception.launch.py
```

If the TensorRT engine does not load on your machine, run with the PyTorch model:

```bash
ros2 launch carkit_perception perception.launch.py \
  model_path:=/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n.pt
```

Run only the node without RViz:

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
ros2 topic hz /yolo/inference_image
ros2 topic echo /yolo/detections --once
ros2 topic echo /yolo/inference_image --once
```

If RViz is blank, confirm `/camera/camera/color/image_raw` is publishing first,
then confirm `/yolo/inference_image` is publishing. If `/yolo/detections` keeps
printing `no detections`, put a common COCO object such as a person, bottle,
chair, or book in front of the camera.

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
