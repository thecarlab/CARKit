# Perception

CARKit perception runs YOLO on the RealSense color stream only. It does not
start or subscribe to depth, infrared, alignment, point-cloud, or IMU streams.

Packages:

- `carkit_perception`: color-only YOLO and traffic-light color classification
- `carkit_perception_msgs`: typed 2D detection messages

The supported runtime is a fixed-shape, batch-one, 640-pixel FP16 TensorRT
engine exported and used on the same Jetson Orin Nano software stack.

## Launch

Start the color-only RealSense driver and perception together:

```bash
ros2 launch carkit_perception perception.launch.py
```

Perception visualization is off by default. To start RViz:

```bash
ros2 launch carkit_perception perception.launch.py visualization:=rviz
```

To start Foxglove Bridge instead:

```bash
ros2 launch carkit_perception perception.launch.py visualization:=foxglove
```

Connect Foxglove to:

```text
ws://<jetson-ip>:8765
```

Import `docs/carkit_perception_layout.json` for a single-panel view of the
YOLO inference image.

If another launch already owns the camera:

```bash
ros2 launch carkit_perception perception.launch.py start_camera:=false
```

## Topic Flow

```text
/camera/camera/color/image_raw
  -> perception_2d_node
  -> /yolo/detections_2d
  -> carkit_behavior
  -> /behavior/*
  -> carkit_control_center
```

Inputs:

- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)

Outputs:

- `/yolo/detections_2d`
  (`carkit_perception_msgs/msg/YoloDetection2DArray`)
- `/yolo/inference_image` (`sensor_msgs/Image`)

Ordinary detections contain their class, confidence, and color-image bounding
box. Traffic lights are published in the array's `traffic_lights` field as
`YoloTrafficLightDetection2D` records; only those records contain a
`traffic_light_color`. The array also contains the source image dimensions so
consumers can use normalized box sizes. Empty frames publish empty arrays.

The behavior layer uses a configured 0.08 m forward camera-to-lidar offset for
horizontal bearing fusion. The camera is also mounted 0.08 m below the lidar;
because `/scan` is planar, detected objects must intersect that scan plane.

Traffic-light color values are unknown `0`, red `1`, yellow `2`, and green
`3`.

## Parameters

- `model_path`: FP16 TensorRT engine path
- `image_size`: fixed engine input size, default `640`
- `image_topic`: color image input
- `inference_image_topic`: annotated image output
- `detection_2d_topic`: typed detection output
- `min_confidence`: YOLO confidence threshold
- `require_engine_metadata`: reject engines without matching metadata
- `start_camera`: launch the RealSense color driver, default `true`
- `visualization`: `none`, `rviz`, or `foxglove`, default `none`
- `rviz_config`: RViz config used with `visualization:=rviz`
- `foxglove_address`: Foxglove Bridge bind address, default `0.0.0.0`
- `foxglove_port`: Foxglove Bridge WebSocket port, default `8765`

## Build The FP16 Engine

```bash
./docker/build_workspace.sh
python3 -m pip install "onnx==1.17.0"
python3 carkit/perception/carkit_perception/util/export_fp16_engine.py \
  --source carkit/perception/carkit_perception/models/yolo11n.pt \
  --output-dir carkit/perception/carkit_perception/models \
  --name yolo11n_fp16.engine \
  --image-size 640
```

Re-export after changing the model, image size, JetPack, CUDA, TensorRT,
PyTorch, or Ultralytics.

## Verify

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /yolo/detections_2d
ros2 topic echo /yolo/detections_2d --once
ros2 topic list | grep depth
```

The final command should produce no RealSense depth image topics.
