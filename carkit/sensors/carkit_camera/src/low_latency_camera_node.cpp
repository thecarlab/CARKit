// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace
{

int retry_ioctl(const int fd, const unsigned long request, void * argument)
{
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::runtime_error system_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

struct MappedBuffer
{
  void * address{MAP_FAILED};
  std::size_t length{0};
};

struct LatestFrame
{
  std::vector<uint8_t> jpeg;
  rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
  uint64_t sequence{0};
};

}  // namespace

class LowLatencyCameraNode : public rclcpp::Node
{
public:
  LowLatencyCameraNode()
  : Node("carkit_camera")
  {
    device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    width_ = declare_parameter<int>("image_width", 640);
    height_ = declare_parameter<int>("image_height", 480);
    capture_rate_ = declare_parameter<int>("capture_framerate", 30);
    publish_rate_ = declare_parameter<double>("publish_framerate", 10.0);
    requested_buffers_ = declare_parameter<int>("buffer_count", 2);
    const auto camera_name = declare_parameter<std::string>("camera_name", "rgb");
    const auto camera_info_url =
      declare_parameter<std::string>("camera_info_url", "");

    if (width_ <= 0 || height_ <= 0 || capture_rate_ <= 0 || publish_rate_ <= 0.0) {
      throw std::invalid_argument("image dimensions and frame rates must be positive");
    }
    requested_buffers_ = std::max(2, std::min(requested_buffers_, 4));

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    compressed_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/camera/color/image_raw/compressed", qos);
    raw_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", qos);
    info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", qos);

    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name, camera_info_url);

    open_camera();
    running_.store(true);
    capture_thread_ = std::thread(&LowLatencyCameraNode::capture_loop, this);
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&LowLatencyCameraNode::publish_latest, this));

    RCLCPP_INFO(
      get_logger(),
      "Capturing MJPEG from %s at %dx%d/%d FPS with %zu buffers; publishing newest frame at %.1f Hz",
      device_.c_str(), width_, height_, capture_rate_, buffers_.size(), publish_rate_);
  }

  ~LowLatencyCameraNode() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    close_camera();
  }

private:
  void open_camera()
  {
    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      throw system_error("cannot open " + device_);
    }

    v4l2_capability capability{};
    if (retry_ioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0) {
      throw system_error("VIDIOC_QUERYCAP");
    }
    if (!(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
      !(capability.capabilities & V4L2_CAP_STREAMING))
    {
      throw std::runtime_error(device_ + " does not support streaming video capture");
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(width_);
    format.fmt.pix.height = static_cast<uint32_t>(height_);
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (retry_ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
      throw system_error("VIDIOC_S_FMT");
    }
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
      throw std::runtime_error(device_ + " did not accept MJPEG capture");
    }
    width_ = static_cast<int>(format.fmt.pix.width);
    height_ = static_cast<int>(format.fmt.pix.height);

    v4l2_streamparm stream_parameters{};
    stream_parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_parameters.parm.capture.timeperframe.numerator = 1;
    stream_parameters.parm.capture.timeperframe.denominator =
      static_cast<uint32_t>(capture_rate_);
    if (retry_ioctl(fd_, VIDIOC_S_PARM, &stream_parameters) < 0) {
      RCLCPP_WARN(get_logger(), "Camera rejected the requested capture frame rate");
    }

    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(requested_buffers_);
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (retry_ioctl(fd_, VIDIOC_REQBUFS, &request) < 0) {
      throw system_error("VIDIOC_REQBUFS");
    }
    if (request.count < 2) {
      throw std::runtime_error("camera supplied fewer than two capture buffers");
    }

    buffers_.resize(request.count);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (retry_ioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        throw system_error("VIDIOC_QUERYBUF");
      }
      buffers_[index].length = buffer.length;
      buffers_[index].address = mmap(
        nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
      if (buffers_[index].address == MAP_FAILED) {
        throw system_error("mmap camera buffer");
      }
      if (retry_ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        throw system_error("VIDIOC_QBUF");
      }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (retry_ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throw system_error("VIDIOC_STREAMON");
    }
    streaming_ = true;
  }

  void close_camera() noexcept
  {
    if (fd_ < 0) {
      return;
    }
    if (streaming_) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      retry_ioctl(fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto & buffer : buffers_) {
      if (buffer.address != MAP_FAILED) {
        munmap(buffer.address, buffer.length);
        buffer.address = MAP_FAILED;
      }
    }
    close(fd_);
    fd_ = -1;
  }

  /// Drain the V4L2 queue on a dedicated thread and retain only the newest JPEG.
  /// This bounds latency when inference is temporarily slower than camera capture.
  void capture_loop()
  {
    pollfd descriptor{fd_, POLLIN, 0};
    while (running_.load()) {
      const int ready = poll(&descriptor, 1, 100);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "Camera poll failed: %s", std::strerror(errno));
        break;
      }
      if (ready == 0 || !(descriptor.revents & POLLIN)) {
        continue;
      }

      // Drain every ready V4L2 buffer. ROS only sees the final, freshest frame.
      while (running_.load()) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (retry_ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
          if (errno == EAGAIN) {
            break;
          }
          RCLCPP_ERROR(get_logger(), "VIDIOC_DQBUF failed: %s", std::strerror(errno));
          running_.store(false);
          break;
        }
        if (buffer.index >= buffers_.size() || buffer.bytesused == 0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Camera returned an invalid buffer");
        } else {
          LatestFrame next;
          const auto * begin = static_cast<const uint8_t *>(buffers_[buffer.index].address);
          next.jpeg.assign(begin, begin + buffer.bytesused);
          // Dequeue occurs as soon as capture completes, so this stamp reflects
          // sensor availability and avoids propagating stale driver timestamps.
          next.stamp = now();
          {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            next.sequence = latest_frame_.sequence + 1;
            latest_frame_ = std::move(next);
          }
        }
        if (retry_ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
          RCLCPP_ERROR(get_logger(), "VIDIOC_QBUF failed: %s", std::strerror(errno));
          running_.store(false);
          break;
        }
      }
    }
  }

  /// Publish the newest unseen frame at the configured 10 Hz presentation rate.
  /// Raw BGR decoding is deferred unless a raw-image subscriber actually exists.
  void publish_latest()
  {
    LatestFrame frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.sequence == 0 || latest_frame_.sequence == published_sequence_) {
        return;
      }
      frame = latest_frame_;
      published_sequence_ = frame.sequence;
    }

    std_msgs::msg::Header header;
    header.stamp = frame.stamp;
    header.frame_id = frame_id_;

    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = header;
    compressed.format = "jpeg";
    compressed.data = frame.jpeg;
    compressed_publisher_->publish(std::move(compressed));

    auto camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header = header;
    info_publisher_->publish(camera_info);

    // Decoding is lazy: the normal perception/WebUI path consumes the camera's
    // JPEG directly and pays no RGB conversion cost.
    if (raw_publisher_->get_subscription_count() > 0) {
      const cv::Mat encoded(1, static_cast<int>(frame.jpeg.size()), CV_8UC1, frame.jpeg.data());
      const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
      if (image.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to decode camera JPEG");
        return;
      }
      sensor_msgs::msg::Image raw;
      raw.header = header;
      raw.height = static_cast<uint32_t>(image.rows);
      raw.width = static_cast<uint32_t>(image.cols);
      raw.encoding = "bgr8";
      raw.is_bigendian = false;
      raw.step = static_cast<uint32_t>(image.cols * image.elemSize());
      raw.data.assign(image.datastart, image.dataend);
      raw_publisher_->publish(std::move(raw));
    }
  }

  std::string device_;
  std::string frame_id_;
  int width_{640};
  int height_{480};
  int capture_rate_{30};
  double publish_rate_{10.0};
  int requested_buffers_{2};
  int fd_{-1};
  bool streaming_{false};
  std::vector<MappedBuffer> buffers_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::mutex frame_mutex_;
  LatestFrame latest_frame_;
  uint64_t published_sequence_{0};

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LowLatencyCameraNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("carkit_camera"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
