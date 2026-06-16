# Perception

Packages:

- `carkit_perception`: synchronized YOLO inference, depth projection, and
  traffic-light color classification
- `carkit_perception_msgs`: typed 3D detection messages

Current perception publishes typed detections on `/yolo/detections_3d`. It
does not publish the older string `/yolo/detections` topic.

## Runtime Model

The supported runtime is a fixed-shape, batch-one, 640-pixel FP16 TensorRT
engine exported and used on the same Jetson software stack.

Export the engine inside the Jetson container after the workspace builds:

```bash
./docker/build_workspace.sh
python3 carkit/perception/carkit_perception/util/export_fp16_engine.py \
  --source carkit/perception/carkit_perception/models/yolo11n.pt \
  --output-dir carkit/perception/carkit_perception/models \
  --name yolo11n_fp16.engine \
  --image-size 640
```

The export writes:

- `carkit/perception/carkit_perception/models/yolo11n_fp16.engine`
- `carkit/perception/carkit_perception/models/yolo11n_fp16.json`

Generated engines and metadata are ignored by Git. Re-export after changing
the model, image size, JetPack, CUDA, TensorRT, PyTorch, or Ultralytics.

## Launch

Start RealSense color and aligned depth:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true
```

Start perception:

```bash
ros2 launch carkit_perception perception.launch.py
```

Start perception without RViz:

```bash
ros2 launch carkit_perception perception.launch.py start_rviz:=false
```

Start behavior after perception and the control center are running:

```bash
ros2 launch carkit_behavior behavior_center.launch.py
```

## Topic Flow

```text
color image + aligned depth + aligned camera_info
  -> perception_3d_node
  -> /yolo/detections_3d
  -> carkit_behavior
  -> /behavior/*
  -> carkit_control_center
  -> /ackermann_cmd
```

Inputs:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/camera_info`
  (`sensor_msgs/CameraInfo`)

Outputs:

- `/yolo/detections_3d`
  (`carkit_perception_msgs/msg/YoloDetection3DArray`)
- `/yolo/inference_image` (`sensor_msgs/Image`)

## Detection Message

```bash
ros2 interface show carkit_perception_msgs/msg/YoloDetection3D
ros2 interface show carkit_perception_msgs/msg/YoloDetection3DArray
```

Each detection contains class, confidence, bounding box, traffic-light color,
and camera optical-frame `x`, `y`, and `z`. In the optical frame, `x` points
right, `y` points down, and `z` points forward.

When depth is unavailable, the detection remains in the array with
`position_valid=false` and NaN coordinates. Frames without objects publish an
empty detection array.

Traffic-light color values:

- `0`: unknown
- `1`: red
- `2`: yellow
- `3`: green

## Launch Arguments

- `model_path`: FP16 TensorRT engine path
- `image_size`: fixed engine input size, default `640`
- `image_topic`: color image input
- `depth_topic`: aligned depth input
- `camera_info_topic`: aligned camera info input
- `inference_image_topic`: annotated image output
- `detection_3d_topic`: typed detection output
- `min_confidence`: YOLO confidence threshold
- `sync_slop`: approximate synchronization tolerance
- `rviz_config`: RViz config path
- `start_rviz`: starts or skips perception RViz

Example:

```bash
ros2 launch carkit_perception perception.launch.py \
  model_path:=/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n_fp16.engine \
  min_confidence:=0.3 \
  sync_slop:=0.05
```

## Verify And Benchmark

```bash
ros2 topic hz /yolo/detections_3d
ros2 topic hz /yolo/inference_image
ros2 topic echo /yolo/detections_3d --once
```

If perception does not start:

1. Confirm RealSense publishes color, aligned depth, and aligned camera info.
2. Confirm the `.engine` and `.json` metadata exist in
   `carkit/perception/carkit_perception/models`.
3. Re-export the engine after any Jetson software-stack change.
4. Confirm CUDA and TensorRT are available inside Docker.
5. Confirm `image_size` matches the engine metadata.
