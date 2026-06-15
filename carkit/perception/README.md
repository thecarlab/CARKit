# TODO
- [ ] Add traffic light color detection
- [ ] Small model (at least 10Hz inference frequency)
- [ ] Link the perception results to reaction (put it under folder /control/carkit_behavior)
- [ ] Rewrite the perception README to write the launch command, Topic Flows, and Parameters that users could tune (check navigation folder)

# Perception

Package: `carkit_perception`

YOLO-based camera perception for classroom demos.

## Launch

Start the RealSense color camera and aligned depth first. The
`align_depth.enable:=true` option is required for camera-frame 3D detections.

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

The perception RViz configuration is owned by the package at
`carkit/perception/carkit_perception/rviz/perception.rviz`.

The default model is the PyTorch `.pt` file because it works without TensorRT.
Use the TensorRT engine only on systems that already have TensorRT installed:

```bash
ros2 launch carkit_perception perception.launch.py \
  model_path:=/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n.engine
```

Run only the node without RViz:

```bash
ros2 run carkit_perception perception_node
```

Run the 3D perception node directly. This node runs YOLO itself and uses aligned
depth plus camera info to publish camera-frame object positions:

```bash
ros2 run carkit_perception perception_3d_node
```

Or start it from the perception launch file:

```bash
ros2 launch carkit_perception perception.launch.py start_3d:=true
```

Override model or topics:

```bash
ros2 run carkit_perception perception_node --ros-args \
  -p model_path:=/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n.pt \
  -p image_topic:=/camera/camera/color/image_raw
```

## Test

Start RealSense first, then:

```bash
ros2 topic hz /yolo/inference_image
ros2 topic echo /yolo/detections --once
ros2 topic echo /yolo/detection --once
ros2 topic echo /yolo/detections_3d --once
ros2 topic echo /yolo/inference_image --once
```

If RViz is blank, confirm `/camera/camera/color/image_raw` is publishing first,
then confirm `/yolo/inference_image` is publishing. If `/yolo/detections` keeps
printing `no detections`, put a common COCO object such as a person, bottle,
chair, or book in front of the camera.

Inputs:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`, used by `perception_3d_node`)
- `/camera/camera/aligned_depth_to_color/camera_info` (`sensor_msgs/CameraInfo`, used by `perception_3d_node`)

Outputs:

- `/yolo/inference_image` (`sensor_msgs/Image`)
- `/yolo/detections` (`std_msgs/String`, 2D detections from `perception_node`)
- `/yolo/detection` (`std_msgs/String`, 2D detections from `perception_3d_node`)
- `/yolo/detections_3d` (`std_msgs/String`, camera-frame 3D detections from `perception_3d_node`)

Parameters:

- `model_path`
- `image_topic`
- `inference_image_topic`
- `detection_topic`
- `depth_topic`
- `camera_info_topic`
- `detection_3d_topic`
- `start_3d`
