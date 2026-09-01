// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#include "carkit_intro2av_cpp/perception_algorithm.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace carkit_intro2av_cpp
{

PerceptionResult process_image(
  const sensor_msgs::msg::CompressedImage & image,
  const sensor_msgs::msg::CameraInfo * camera_info,
  const PerceptionConfig & config)
{
  // TODO(Intro2AV C++): Decode image.data, run the detector, and return typed
  // detections plus an optional annotated JPEG in result.preview.
  (void)image;
  (void)camera_info;
  (void)config;
  return {};
}

}  // namespace carkit_intro2av_cpp
