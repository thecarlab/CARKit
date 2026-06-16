# Perception

Packages:

- `carkit_perception`: synchronized YOLO, depth projection, and traffic-light
  color classification
- `carkit_perception_msgs`: typed 3D detection messages

The supported runtime is a fixed-shape, batch-one, 640-pixel FP16 TensorRT
engine exported and used on the same Jetson Orin Nano 8 GB software stack.

## Build The FP16 Engine

Build the workspace and export the engine inside the Jetson container:

```bash
./docker/build_workspace.sh
python3 -m pip install "onnx==1.17.0"
python3 carkit/perception/carkit_perception/util/export_fp16_engine.py \
  --source carkit/perception/carkit_perception/models/yolo11n.pt \
  --output-dir carkit/perception/carkit_perception/models \
  --name yolo11n_fp16.engine \
  --image-size 640
```

ONNX is needed only as the intermediate model format. The exporter disables
ONNX simplification, so `onnxruntime-gpu` and `onnxslim` are not required.

The export writes:

- `/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n_fp16.engine`
- `/workspaces/CARKit/carkit/perception/carkit_perception/models/yolo11n_fp16.json`

The metadata records the engine hash, source-model hash, precision, shape, and
CUDA, TensorRT, PyTorch, and Ultralytics versions. Generated engines and
metadata are ignored by Git. Re-export after changing the model, image size,
JetPack, CUDA, TensorRT, PyTorch, or Ultralytics.

## Launch

Start the RealSense color camera and aligned depth:

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

Start the complete perception and behavior layer after camera and vehicle
bring-up:

```bash
ros2 launch carkit_behavior road_rules.launch.py
```

## Topic Flow

```text
color image + aligned depth + aligned camera_info
  -> synchronized perception_3d_node
  -> /yolo/detections_3d
  -> carkit_behavior
  -> /behavior
  -> ackermann_mux
  -> /ackermann_cmd
```

Inputs:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/camera_info`
  (`sensor_msgs/CameraInfo`)

Outputs:

- `/yolo/detections_3d`
  (`carkit_perception_msgs/YoloDetection3DArray`)
- `/yolo/inference_image` (`sensor_msgs/Image`)

There are no string or separate 2D detection topics.

## Detection Message

```bash
ros2 interface show carkit_perception_msgs/msg/YoloDetection3D
ros2 interface show carkit_perception_msgs/msg/YoloDetection3DArray
```

Each detection contains its class, confidence, color-image bounding box,
traffic-light color, and camera optical-frame `x`, `y`, and `z`. In the optical
frame, `x` points right, `y` points down, and `z` points forward.

When depth is unavailable, the detection remains in the array with
`position_valid=false` and NaN coordinates. Frames without objects publish an
empty detection array.

Traffic-light color values are:

- `0`: unknown
- `1`: red
- `2`: yellow
- `3`: green

## Parameters

- `model_path`: FP16 TensorRT engine path
- `image_size`: fixed engine input size, default `640`
- `image_topic`, `depth_topic`, `camera_info_topic`: synchronized inputs
- `inference_image_topic`: annotated image output
- `detection_3d_topic`: typed detection output
- `min_confidence`: YOLO confidence threshold
- `min_depth`, `max_depth`: accepted depth range in meters
- `sync_queue_size`: synchronized input queue, default `2`
- `sync_slop`: approximate synchronization tolerance in seconds
- `require_engine_metadata`: reject engines without matching metadata

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

The deployment target is at least 10 Hz for ten minutes, with p95 end-to-end
detection latency below 150 ms and no increasing input backlog.

If perception does not start:

1. Confirm the three RealSense input topics publish with compatible timestamps.
2. Confirm the engine and JSON metadata exist in
   `carkit/perception/carkit_perception/models`.
3. Re-export the engine after any Jetson software-stack change.
4. Confirm CUDA is available inside Docker.
5. Confirm `image_size` matches the engine metadata.
