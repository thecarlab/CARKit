// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "carkit_intro2av_cpp/perception_algorithm.hpp"
#include "carkit_perception_msgs/msg/yolo_detection2_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
class Intro2AvPerception : public rclcpp::Node
{
public:
  Intro2AvPerception() : Node("carkit_intro2av_cpp_perception")
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/camera/color/image_raw/compressed");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/color/camera_info");
    detection_topic_ = declare_parameter<std::string>(
      "detection_2d_topic", "/yolo/detections_2d");
    preview_topic_ = declare_parameter<std::string>(
      "inference_compressed_topic", "/yolo/inference_image/compressed");
    const auto rate = std::max(0.1, declare_parameter("max_inference_rate_hz", 10.0));
    declare_parameter("minimum_confidence", 0.20);
    declare_parameter("image_size", 448);
    declare_parameter<std::string>("model_path", "");

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    detections_pub_ = create_publisher<
      carkit_perception_msgs::msg::YoloDetection2DArray>(detection_topic_, 10);
    preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      preview_topic_, sensor_qos);
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, sensor_qos,
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        latest_image_ = message;
        ++image_generation_;
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_ = message;
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {publish_result();});
    RCLCPP_INFO(
      get_logger(),
      "Intro2AV C++ perception ready; implement perception_algorithm.cpp");
  }

private:
  carkit_intro2av_cpp::PerceptionConfig configuration() const
  {
    carkit_intro2av_cpp::PerceptionConfig config;
    config.minimum_confidence = get_parameter("minimum_confidence").as_double();
    config.image_size = get_parameter("image_size").as_int();
    config.model_path = get_parameter("model_path").as_string();
    return config;
  }

  static void validate(const carkit_intro2av_cpp::PerceptionResult & result)
  {
    for (const auto & detection : result.detections) {
      const double values[] = {
        detection.confidence, detection.bbox_x_min, detection.bbox_y_min,
        detection.bbox_x_max, detection.bbox_y_max};
      for (const double value : values) {
        if (!std::isfinite(value)) {
          throw std::runtime_error("detector returned a non-finite value");
        }
      }
      if (detection.confidence < 0.0 || detection.confidence > 1.0) {
        throw std::runtime_error("detection confidence must be in [0, 1]");
      }
    }
  }

  void process_latest_image()
  {
    try {
      auto result = carkit_intro2av_cpp::process_image(
        *latest_image_, camera_info_ ? camera_info_.get() : nullptr, configuration());
      validate(result);
      auto output = std::make_shared<carkit_perception_msgs::msg::YoloDetection2DArray>();
      output->header = latest_image_->header;
      if (camera_info_) {
        output->image_width = camera_info_->width;
        output->image_height = camera_info_->height;
      }
      output->detections = std::move(result.detections);
      output->traffic_lights = std::move(result.traffic_lights);
      latest_output_ = output;
      latest_preview_ = result.preview ? result.preview :
        std::make_shared<sensor_msgs::msg::CompressedImage>(*latest_image_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Perception algorithm failed: %s", error.what());
      latest_output_ =
        std::make_shared<carkit_perception_msgs::msg::YoloDetection2DArray>();
      latest_output_->header = latest_image_->header;
      if (camera_info_) {
        latest_output_->image_width = camera_info_->width;
        latest_output_->image_height = camera_info_->height;
      }
      latest_preview_ =
        std::make_shared<sensor_msgs::msg::CompressedImage>(*latest_image_);
    }
    processed_generation_ = image_generation_;
  }

  void publish_result()
  {
    if (!latest_image_) {
      return;
    }
    if (processed_generation_ != image_generation_) {
      process_latest_image();
    }
    if (latest_output_) {
      detections_pub_->publish(*latest_output_);
    }
    if (latest_preview_ && preview_pub_->get_subscription_count() > 0) {
      preview_pub_->publish(*latest_preview_);
    }
  }

  std::string image_topic_, camera_info_topic_, detection_topic_, preview_topic_;
  uint64_t image_generation_{0};
  uint64_t processed_generation_{static_cast<uint64_t>(-1)};
  sensor_msgs::msg::CompressedImage::ConstSharedPtr latest_image_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  sensor_msgs::msg::CompressedImage::SharedPtr latest_preview_;
  carkit_perception_msgs::msg::YoloDetection2DArray::SharedPtr latest_output_;
  rclcpp::Publisher<carkit_perception_msgs::msg::YoloDetection2DArray>::SharedPtr
    detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intro2AvPerception>());
  rclcpp::shutdown();
  return 0;
}
