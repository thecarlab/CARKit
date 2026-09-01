# Perception

CARKit perception can run the general YOLO detector, the traffic-sign detector,
both detectors, or a course-specific custom model on the color stream only. It
does not start or subscribe to depth,
infrared, alignment, point-cloud, or IMU streams.

Packages:

- `carkit_perception`: color-only YOLO, traffic-sign detection, and
  traffic-light color classification
- `carkit_perception_msgs`: typed 2D detection messages

The supported runtime is a fixed-shape, batch-one, 448-pixel FP16 TensorRT
engine exported and used on the same Jetson Orin Nano software stack.

## Launch

Start the sensor package for the installed chassis, then start perception:

```bash
ros2 launch osracer_bringup sensors_launch.py
ros2 launch carkit_perception perception.launch.py
```

Camera ownership stays in central sensor bringup so changing a perception
implementation cannot start a duplicate driver. View the source camera and
annotated results at `http://<jetson-ip>:8080`.

## Topic Flow

```text
/camera/camera/color/image_raw/compressed
  -> perception_2d_node
  -> /yolo/detections_2d
  -> carkit_behavior
  -> /behavior/*
  -> carkit_control_center
```

Inputs:

- `/camera/camera/color/image_raw/compressed`
  (`sensor_msgs/CompressedImage`)

Outputs:

- `/yolo/detections_2d`
  (`carkit_perception_msgs/msg/YoloDetection2DArray`)
- `/yolo/inference_image` (`sensor_msgs/Image`)
- `/yolo/inference_image/compressed` (`sensor_msgs/CompressedImage`), used by
  the WebUI for the live detection overlay

Detection arrays and the cached annotated view publish at a stable 10 Hz. YOLO
always processes the newest available frame; it does not build up an old-frame
queue when the system is busy.

Ordinary detections contain their class, confidence, and color-image bounding
box. This includes detections from both the general YOLO model and the
traffic-sign model. Traffic lights are published in the array's
`traffic_lights` field as `YoloTrafficLightDetection2D` records; only those
records contain a `traffic_light_color`. The array also contains the source
image dimensions so consumers can use normalized box sizes. Empty frames
publish empty arrays.

The behavior layer uses a configured 0.08 m forward camera-to-lidar offset for
horizontal bearing fusion. The camera is also mounted 0.08 m below the lidar;
because `/scan` is planar, detected objects must intersect that scan plane.

Traffic-light color values are unknown `0`, red `1`, yellow `2`, and green
`3`.

## Parameters

- `model_path`: FP16 TensorRT engine path
- `traffic_sign_model_path`: traffic-sign YOLO model path
- `model_profile`: `generic_coco`, `traffic_signs`, `combined`, or `custom`
- `custom_model_path`: FP16 TensorRT engine used by the `custom` profile
- `image_size`: fixed engine input size, default `448`
- `image_topic`: compressed color image input
- `input_transport`: `compressed` by default, or `raw`
- `inference_image_topic`: annotated image output
- `inference_compressed_topic`: browser-ready annotated JPEG output
- `detection_2d_topic`: typed detection output
- `max_inference_rate_hz`: inference and result output target, default `10.0`
- `secondary_inference_interval`: combined-mode sign-model interval, default `2`
- `inference_jpeg_quality`: WebUI overlay JPEG quality, default `70`
- `min_confidence`: YOLO confidence threshold
- `traffic_sign_min_confidence`: traffic-sign model confidence threshold
- `require_engine_metadata`: reject engines without matching metadata

## Build The FP16 Engine

```bash
./docker/build_workspace.sh
python3 -m pip install "onnx==1.17.0"
python3 carkit/perception/carkit_perception/util/export_fp16_engine.py \
  --source carkit/perception/carkit_perception/models/yolo11n.pt \
  --output-dir carkit/perception/carkit_perception/models \
  --name yolo11n_fp16.engine \
  --image-size 448
```

Re-export after changing the model, image size, JetPack, CUDA, TensorRT,
PyTorch, or Ultralytics.

## Verify

```bash
ros2 topic hz /camera/camera/color/camera_info
ros2 topic hz /yolo/detections_2d
ros2 topic hz /yolo/inference_image/compressed
ros2 topic echo /yolo/detections_2d --once
ros2 topic list | grep depth
```

The final command should produce no RealSense depth image topics.
