// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#ifndef CARKIT_INTRO2AV_CPP__PERCEPTION_ALGORITHM_HPP_
#define CARKIT_INTRO2AV_CPP__PERCEPTION_ALGORITHM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "carkit_perception_msgs/msg/yolo_detection2_d.hpp"
#include "carkit_perception_msgs/msg/yolo_traffic_light_detection2_d.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// CARKit learning annotation: declares interfaces implemented by the corresponding source.
namespace carkit_intro2av_cpp
{

struct PerceptionConfig
{
  double minimum_confidence{0.20};
  int image_size{448};
  std::string model_path;
};

struct PerceptionResult
{
  std::vector<carkit_perception_msgs::msg::YoloDetection2D> detections;
  std::vector<carkit_perception_msgs::msg::YoloTrafficLightDetection2D> traffic_lights;
  sensor_msgs::msg::CompressedImage::SharedPtr preview;
};

PerceptionResult process_image(
  const sensor_msgs::msg::CompressedImage & image,
  const sensor_msgs::msg::CameraInfo * camera_info,
  const PerceptionConfig & config);

}  // namespace carkit_intro2av_cpp

#endif  // CARKIT_INTRO2AV_CPP__PERCEPTION_ALGORITHM_HPP_
