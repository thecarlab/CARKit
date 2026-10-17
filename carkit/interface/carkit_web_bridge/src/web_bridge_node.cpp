// Copyright 2026 University of Delaware
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <carkit_perception_msgs/msg/yolo_detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#ifdef CARKIT_HAS_VESC_MSGS
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#endif

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace carkit_web_bridge
{

namespace net = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = net::ip::tcp;
using Json = nlohmann::json;
using SteadyClock = std::chrono::steady_clock;

constexpr std::size_t kMaximumQueueDepth = 32;

const std::unordered_set<std::string> kAllowedSubscriptions = {
  "/carkit/hardware", "/map", "/scan", "/plan", "/odom", "/amcl_pose", "/control_center/main_state",
  "/ackermann_cmd", "/camera/camera/color/image_raw/compressed",
  "/yolo/detections_2d", "/battery_state", "/sensors/core",
};

/// Convert time data into a JSON object for browser transport.
Json time_json(const builtin_interfaces::msg::Time & value)
{
  return {{"sec", value.sec}, {"nanosec", value.nanosec}};
}

/// Convert header data into a JSON object for browser transport.
Json header_json(const std_msgs::msg::Header & value)
{
  return {{"stamp", time_json(value.stamp)}, {"frame_id", value.frame_id}};
}

/// Convert vector data into a JSON object for browser transport.
Json vector_json(const geometry_msgs::msg::Vector3 & value)
{
  return {{"x", value.x}, {"y", value.y}, {"z", value.z}};
}

/// Convert point data into a JSON object for browser transport.
Json point_json(const geometry_msgs::msg::Point & value)
{
  return {{"x", value.x}, {"y", value.y}, {"z", value.z}};
}

/// Convert quaternion data into a JSON object for browser transport.
Json quaternion_json(const geometry_msgs::msg::Quaternion & value)
{
  return {{"x", value.x}, {"y", value.y}, {"z", value.z}, {"w", value.w}};
}

/// Convert pose data into a JSON object for browser transport.
Json pose_json(const geometry_msgs::msg::Pose & value)
{
  return {
    {"position", point_json(value.position)},
    {"orientation", quaternion_json(value.orientation)},
  };
}

/// Convert pose stamped data into a JSON object for browser transport.
Json pose_stamped_json(const geometry_msgs::msg::PoseStamped & value)
{
  return {{"header", header_json(value.header)}, {"pose", pose_json(value.pose)}};
}

/// Convert detection data into a JSON object for browser transport.
Json detection_json(const carkit_perception_msgs::msg::YoloDetection2D & value)
{
  return {
    {"class_id", value.class_id},
    {"class_name", value.class_name},
    {"confidence", value.confidence},
    {"bbox_x_min", value.bbox_x_min},
    {"bbox_y_min", value.bbox_y_min},
    {"bbox_x_max", value.bbox_x_max},
    {"bbox_y_max", value.bbox_y_max},
  };
}

std_msgs::msg::Header parse_header(const Json & value)
{
  std_msgs::msg::Header output;
  if (!value.is_object()) {
    return output;
  }
  output.frame_id = value.value("frame_id", "");
  const auto stamp = value.value("stamp", Json::object());
  output.stamp.sec = stamp.value("sec", 0);
  output.stamp.nanosec = stamp.value("nanosec", 0U);
  return output;
}

geometry_msgs::msg::Point parse_point(const Json & value)
{
  geometry_msgs::msg::Point output;
  output.x = value.value("x", 0.0);
  output.y = value.value("y", 0.0);
  output.z = value.value("z", 0.0);
  return output;
}

geometry_msgs::msg::Quaternion parse_quaternion(const Json & value)
{
  geometry_msgs::msg::Quaternion output;
  output.x = value.value("x", 0.0);
  output.y = value.value("y", 0.0);
  output.z = value.value("z", 0.0);
  output.w = value.value("w", 1.0);
  return output;
}

geometry_msgs::msg::Pose parse_pose(const Json & value)
{
  geometry_msgs::msg::Pose output;
  output.position = parse_point(value.value("position", Json::object()));
  output.orientation = parse_quaternion(value.value("orientation", Json::object()));
  return output;
}

struct OutgoingMessage
{
  std::string topic;
  std::shared_ptr<const std::vector<std::uint8_t>> data;
};

struct Subscription
{
  std::chrono::milliseconds throttle{0};
  std::optional<SteadyClock::time_point> last_sent;
};

class WebSocketServer;

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession>
{
public:
  WebSocketSession(tcp::socket socket, WebSocketServer & server)
  : socket_(std::move(socket)), server_(server) {}

  void start();
  void event(const Json & message) {
    const std::string key = "/carkit/session/" + message.value("kind", "event");
    subscriptions_[key] = Subscription{std::chrono::milliseconds(0), std::nullopt};
    const auto bytes = Json::to_cbor(Json{{"op", "publish"}, {"topic", "/carkit/session"}, {"msg", message}});
    deliver(key, std::make_shared<const std::vector<std::uint8_t>>(bytes));
  }
  bool authenticated() const {return authenticated_;}
  const auto & subscriptions() const {return subscriptions_;}
  bool ready(const std::string & topic) const {
    const auto it = subscriptions_.find(topic);
    return it != subscriptions_.end() && (!it->second.last_sent ||
      SteadyClock::now() - *it->second.last_sent >= it->second.throttle);
  }
  void deliver(
    const std::string & topic,
    const std::shared_ptr<const std::vector<std::uint8_t>> & data);

private:
  void read();
  void on_read(beast::error_code error, std::size_t bytes_transferred);
  void on_write(beast::error_code error, std::size_t bytes_transferred);
  void close();
  void handle_request(const Json & request);
  void write_next();

  websocket::stream<tcp::socket> socket_;
  WebSocketServer & server_;
  beast::flat_buffer input_;
  beast::http::request<beast::http::string_body> handshake_;
  bool authenticated_{false};
  std::unordered_map<std::string, Subscription> subscriptions_;
  std::deque<OutgoingMessage> output_;
  bool writing_{false};
  bool closed_{false};
};

class WebSocketServer
{
public:
  using PublishHandler = std::function<void (const std::string &, const Json &)>;

  WebSocketServer(
    std::string address, std::uint16_t port, std::size_t maximum_clients,
    PublishHandler publish_handler)
  : acceptor_(context_), maximum_clients_(maximum_clients),
    publish_handler_(std::move(publish_handler)), lease_timer_(context_)
  {
    const tcp::endpoint endpoint(net::ip::make_address(address), port);
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(net::socket_base::max_listen_connections);
    const char * token = std::getenv("CARKIT_CONTROL_TOKEN");
    token_ = token ? token : "";
  }

  ~WebSocketServer()
  {
    stop();
  }

  void start()
  {
    accept();
    check_lease();
    thread_ = std::thread([this]() {context_.run();});
  }

  void stop()
  {
    if (stopped_.exchange(true)) {
      return;
    }
    net::post(
      context_, [this]() {
        beast::error_code error;
        acceptor_.close(error);
        context_.stop();
      });
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  bool has_clients() const
  {
    return client_count_.load() != 0U;
  }

  bool has_interest(const std::string & topic) const {
    std::lock_guard<std::mutex> lock(interest_mutex_);
    return interested_topics_.count(topic) != 0U;
  }

  // Called only on the network executor; readers on the ROS executor see a snapshot.
  void refresh_interest() {
    std::unordered_set<std::string> topics;
    for (const auto & session : sessions_) {
      if (!session->authenticated()) {continue;}
      for (const auto & entry : session->subscriptions()) {topics.insert(entry.first);}
    }
    std::lock_guard<std::mutex> lock(interest_mutex_);
    interested_topics_ = std::move(topics);
  }

  void broadcast_lazy(const std::string & topic, std::function<Json()> make_message, bool retain = false)
  {
    if (!retain && !has_interest(topic)) {return;}
    net::post(context_, [this, topic, make_message = std::move(make_message), retain]() {
      // Reject unrequested/throttled samples before JSON construction and CBOR encoding.
      const bool due = std::any_of(sessions_.begin(), sessions_.end(), [&topic](const auto & session) {
        return session->authenticated() && session->ready(topic);
      });
      if (!retain && !due) {return;}
      const Json packet = {{"op", "publish"}, {"topic", topic}, {"msg", make_message()}};
      auto encoded = std::make_shared<const std::vector<std::uint8_t>>(Json::to_cbor(packet));
      if (retain) {retained_[topic] = encoded;}
      for (const auto & session : sessions_) {
        if (session->authenticated()) {session->deliver(topic, encoded);}
      }
    });
  }

  void broadcast(const std::string & topic, const Json & message, bool retain = false) {
    broadcast_lazy(topic, [message]() {return message;}, retain);
  }

  void replay(
    const std::string & topic,
    const std::shared_ptr<WebSocketSession> & session) const
  {
    const auto retained = retained_.find(topic);
    if (retained != retained_.end()) {
      session->deliver(topic, retained->second);
    }
  }

  void join(const std::shared_ptr<WebSocketSession> & session)
  {
    sessions_.insert(session);
    client_count_.store(sessions_.size());
  }

  void leave(const std::shared_ptr<WebSocketSession> & session)
  {
    if (operator_owner_.lock() == session) {
      release_operator();
    }
    sessions_.erase(session);
    refresh_interest();
    client_count_.store(sessions_.size());
  }

  bool full() const
  {
    return sessions_.size() >= maximum_clients_;
  }

  bool operator_available() {expire_operator(); return operator_owner_.expired();}

  bool authenticate(const std::string & candidate) const {
    if (token_.size() < 32 || candidate.size() != token_.size()) {return false;}
    unsigned difference = 0;
    for (size_t i = 0; i < token_.size(); ++i) {difference |= candidate[i] ^ token_[i];}
    return difference == 0;
  }

  bool valid_origin(const std::string & origin, const std::string & host) const {
    if (origin.empty()) {return true;}  // CLI clients must still authenticate.
    const char * configured = std::getenv("CARKIT_ALLOWED_ORIGINS");
    std::istringstream origins(configured ? configured : "");
    for (std::string item; std::getline(origins, item, ',');) {
      if (!item.empty() && item == origin) {return true;}
    }
    const auto colon = host.rfind(':');
    const auto hostname = colon == std::string::npos ? host : host.substr(0, colon);
    const char * port = std::getenv("CARKIT_WEBUI_PORT");
    return origin == "http://" + hostname + ":" + (port ? port : "8080") ||
           origin == "https://" + hostname;
  }

  void operator_request(const std::string & operation,
    const std::shared_ptr<WebSocketSession> & session)
  {
    expire_operator();
    const auto owner = operator_owner_.lock();
    if (operation == "claim_control" && (!owner || owner == session)) {
      operator_owner_ = session;
      operator_received_ = SteadyClock::now();
    } else if (operation == "control_heartbeat" && owner == session) {
      operator_received_ = SteadyClock::now();
    } else if (operation == "release_control" && owner == session) {
      release_operator();
    }
    notify_ownership();
  }

  void publish_from_browser(
    const std::string & topic, const Json & message,
    const std::shared_ptr<WebSocketSession> & session)
  {
    expire_operator();
    const bool emergency_stop = topic == "/control_center/emergency_stop" &&
      message.value("data", false);
    const bool allowed_topic = topic == "/teleop" || topic == "/enable_autonomous_control" ||
      topic == "/goal_pose" || topic == "/initialpose" || topic == "/control_center/emergency_stop";
    const bool accepted = allowed_topic && (emergency_stop || topic == "/initialpose" || operator_owner_.lock() == session);
    if (!accepted) {
      session->event({{"kind", "command"}, {"topic", topic}, {"accepted", false},
        {"reason", "Take control before sending this command"}});
      return;
    }
    if (topic == "/teleop") {
      const auto drive = message.at("drive");
      if (!std::isfinite(drive.at("speed").get<double>()) ||
          !std::isfinite(drive.at("steering_angle").get<double>())) {return;}
    }
    publish_handler_(topic, message);
    session->event({{"kind", "command"}, {"topic", topic}, {"accepted", true}});
  }

private:
  void notify_ownership() {
    const auto owner = operator_owner_.lock();
    for (const auto & session : sessions_) {
      if (session->authenticated()) {
        session->event({{"kind", "ownership"}, {"operator", owner == session},
          {"available", !owner}});
      }
    }
  }
  void release_operator() {
    publish_handler_("/teleop", {{"drive", {{"speed", 0.0}, {"steering_angle", 0.0}}}});
    operator_owner_.reset();
    notify_ownership();
  }
  void expire_operator() {
    if (!operator_owner_.expired() &&
      SteadyClock::now() - operator_received_ > std::chrono::seconds(3)) {release_operator();}
  }
  void check_lease() {
    lease_timer_.expires_after(std::chrono::milliseconds(250));
    lease_timer_.async_wait([this](beast::error_code error) {
      if (!error && !stopped_.load()) {expire_operator(); check_lease();}
    });
  }
  void accept()
  {
    acceptor_.async_accept(
      [this](beast::error_code error, tcp::socket socket) {
        if (!error) {
          if (!full()) {
            std::make_shared<WebSocketSession>(std::move(socket), *this)->start();
          } else {
            beast::error_code ignored;
            socket.shutdown(tcp::socket::shutdown_both, ignored);
            socket.close(ignored);
          }
        }
        if (!stopped_.load()) {
          accept();
        }
      });
  }

  net::io_context context_{1};
  tcp::acceptor acceptor_;
  std::unordered_set<std::shared_ptr<WebSocketSession>> sessions_;
  std::unordered_map<
    std::string, std::shared_ptr<const std::vector<std::uint8_t>>> retained_;
  const std::size_t maximum_clients_;
  PublishHandler publish_handler_;
  std::string token_;
  net::steady_timer lease_timer_;
  std::weak_ptr<WebSocketSession> operator_owner_;
  SteadyClock::time_point operator_received_{};
  std::atomic<std::size_t> client_count_{0};
  std::atomic<bool> stopped_{false};
  std::thread thread_;
  mutable std::mutex interest_mutex_;
  std::unordered_set<std::string> interested_topics_;
};

void WebSocketSession::start()
{
  socket_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
  socket_.set_option(
    websocket::stream_base::decorator(
      [](websocket::response_type & response) {
        response.set(beast::http::field::server, "carkit-web-bridge");
      }));
  socket_.read_message_max(64 * 1024);
  beast::http::async_read(socket_.next_layer(), input_, handshake_,
    [self = shared_from_this()](beast::error_code error, std::size_t) {
      if (error || !self->server_.valid_origin(
        self->handshake_[beast::http::field::origin].to_string(),
        self->handshake_[beast::http::field::host].to_string())) {
        self->close(); return;
      }
      self->socket_.async_accept(self->handshake_, [self](beast::error_code accept_error) {
        if (accept_error) {return;}
        self->server_.join(self);
        self->read();
      });
    });
}

void WebSocketSession::read()
{
  socket_.async_read(
    input_, [self = shared_from_this()](beast::error_code error, std::size_t bytes) {
      self->on_read(error, bytes);
    });
}

void WebSocketSession::on_read(beast::error_code error, std::size_t)
{
  if (error) {
    close();
    return;
  }
  try {
    const auto text = beast::buffers_to_string(input_.data());
    handle_request(Json::parse(text));
  } catch (const Json::exception &) {
    // Invalid browser requests are ignored without affecting other clients.
  }
  input_.consume(input_.size());
  read();
}

/// Implement the small rosbridge-compatible operation set required by the dashboard.
/// Topic allowlisting prevents a browser client from subscribing to arbitrary ROS data.
void WebSocketSession::handle_request(const Json & request)
{
  const auto operation = request.value("op", "");
  if (operation == "authenticate") {
    authenticated_ = server_.authenticate(request.value("token", ""));
    server_.refresh_interest();
    event({{"kind", "authentication"}, {"authenticated", authenticated_},
      {"available", server_.operator_available()}});
    if (!authenticated_) {server_.operator_request("release_control", shared_from_this());}
    return;
  }
  if (!authenticated_) {
    event({{"kind", "authentication"}, {"authenticated", false}});
    return;
  }
  if (operation == "claim_control" || operation == "release_control" || operation == "control_heartbeat") {
    server_.operator_request(operation, shared_from_this()); return;
  }
  const auto topic = request.value("topic", "");
  if (operation == "subscribe") {
    if (kAllowedSubscriptions.count(topic) == 0U) {
      return;
    }
    const auto milliseconds = std::max(0, request.value("throttle_rate", 0));
    subscriptions_[topic] = Subscription{std::chrono::milliseconds(milliseconds), std::nullopt};
    server_.refresh_interest();
    server_.replay(topic, shared_from_this());
  } else if (operation == "unsubscribe") {
    subscriptions_.erase(topic);
    server_.refresh_interest();
  } else if (operation == "publish" && request.contains("msg")) {
    server_.publish_from_browser(topic, request.at("msg"), shared_from_this());
  }
  // "advertise" is intentionally accepted as a no-op for rosbridge client compatibility.
}

/// Apply per-client throttling and replace queued samples of the same topic.
/// Coalescing prevents a slow browser from accumulating stale sensor frames.
void WebSocketSession::deliver(
  const std::string & topic,
  const std::shared_ptr<const std::vector<std::uint8_t>> & data)
{
  const auto iterator = subscriptions_.find(topic);
  if (iterator == subscriptions_.end()) {
    return;
  }
  auto & subscription = iterator->second;
  const auto now = SteadyClock::now();
  if (subscription.last_sent && now - *subscription.last_sent < subscription.throttle) {
    return;
  }
  subscription.last_sent = now;

  const auto queued = std::find_if(
    output_.rbegin(), output_.rend(),
    [&topic](const OutgoingMessage & value) {return value.topic == topic;});
  if (queued != output_.rend()) {
    const auto index = static_cast<std::size_t>(std::distance(queued, output_.rend()) - 1);
    if (!writing_ || index != 0U) {
      output_[index].data = data;
      return;
    }
  }
  if (output_.size() >= kMaximumQueueDepth) {
    if (writing_ && output_.size() > 1U) {
      output_.erase(output_.begin() + 1);
    } else if (!writing_) {
      output_.pop_front();
    }
  }
  output_.push_back({topic, data});
  if (!writing_) {
    write_next();
  }
}

void WebSocketSession::write_next()
{
  if (output_.empty() || closed_) {
    writing_ = false;
    return;
  }
  writing_ = true;
  socket_.binary(true);
  socket_.async_write(
    net::buffer(*output_.front().data),
    [self = shared_from_this()](beast::error_code error, std::size_t bytes) {
      self->on_write(error, bytes);
    });
}

void WebSocketSession::on_write(beast::error_code error, std::size_t)
{
  if (error) {
    close();
    return;
  }
  output_.pop_front();
  write_next();
}

void WebSocketSession::close()
{
  if (closed_) {
    return;
  }
  closed_ = true;
  server_.leave(shared_from_this());
  beast::error_code ignored;
  socket_.close(websocket::close_code::normal, ignored);
}

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("carkit_web_bridge")
  {
    const auto address = declare_parameter<std::string>("address", "127.0.0.1");
    const auto port = declare_parameter("port", 9090);
    const auto maximum_clients = declare_parameter("maximum_clients", 5);
    keyboard_enabled_ = declare_parameter<std::string>("manual_input", "keyboard") == "keyboard";
    max_speed_ = declare_parameter("max_speed", 0.8);
    max_reverse_speed_ = declare_parameter("max_reverse_speed", 0.8);
    max_steering_ = declare_parameter("max_steering_angle", 0.34);
    if (!std::isfinite(max_speed_) || max_speed_ <= 0 ||
      !std::isfinite(max_reverse_speed_) || max_reverse_speed_ <= 0 ||
      !std::isfinite(max_steering_) || max_steering_ <= 0)
    {
      throw std::invalid_argument("Manual driving limits must be finite and positive");
    }
    if (port < 1 || port > 65535 || maximum_clients < 1 || maximum_clients > 32) {
      throw std::invalid_argument("invalid WebSocket port or client limit");
    }

    autonomous_publisher_ = create_publisher<std_msgs::msg::Int8>(
      "/enable_autonomous_control", 10);
    teleop_publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/teleop", 10);
    estop_publisher_ = create_publisher<std_msgs::msg::Bool>("/control_center/emergency_stop", 10);
    initial_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    goal_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    server_ = std::make_unique<WebSocketServer>(
      address, static_cast<std::uint16_t>(port), static_cast<std::size_t>(maximum_clients),
      [this](const std::string & topic, const Json & message) {
        publish_from_browser(topic, message);
      });
    create_subscriptions();
    server_->start();
    server_->broadcast("/carkit/hardware", Json{{"laser_mount",
      declare_parameter<std::vector<double>>("laser_mount", {0., 0., 0., 0., 0., 0.})}, {"footprint", {
      {"length", declare_parameter("footprint_length", 0.5)},
      {"width", declare_parameter("footprint_width", 0.25)},
      {"center_x", declare_parameter("footprint_center_x", 0.0)},
      {"center_y", declare_parameter("footprint_center_y", 0.0)}}}}, true);
    RCLCPP_INFO(
      get_logger(), "C++ WebUI bridge listening on %s:%d (maximum %d clients)",
      address.c_str(), static_cast<int>(port), static_cast<int>(maximum_clients));
  }

  ~BridgeNode() override
  {
    // Join network callbacks while the publishers they use are still alive.
    server_->stop();
  }

private:
  template<typename MessageT, typename CallbackT>
  typename rclcpp::Subscription<MessageT>::SharedPtr subscribe(
    const std::string & topic, const rclcpp::QoS & qos, CallbackT callback,
    bool retain = false)
  {
    return create_subscription<MessageT>(
      topic, qos,
      [this, topic, callback, retain](const typename MessageT::ConstSharedPtr message) {
        if (retain || server_->has_interest(topic)) {
          server_->broadcast_lazy(topic, [message, callback]() {return callback(*message);}, retain);
        }
      });
  }

  void create_subscriptions()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    map_subscription_ = subscribe<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      [](const auto & message) {
        return Json{
          {"header", header_json(message.header)},
          {"info", {
              {"map_load_time", time_json(message.info.map_load_time)},
              {"resolution", message.info.resolution},
              {"width", message.info.width},
              {"height", message.info.height},
              {"origin", pose_json(message.info.origin)},
            }},
          {"data", message.data},
        };
      }, true);
    scan_subscription_ = subscribe<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos,
      [](const auto & message) {
        return Json{
          {"header", header_json(message.header)},
          {"angle_min", message.angle_min}, {"angle_max", message.angle_max},
          {"angle_increment", message.angle_increment},
          {"time_increment", message.time_increment}, {"scan_time", message.scan_time},
          {"range_min", message.range_min}, {"range_max", message.range_max},
          {"ranges", message.ranges}, {"intensities", message.intensities},
        };
      });
    path_subscription_ = subscribe<nav_msgs::msg::Path>(
      "/plan", rclcpp::QoS(1),
      [](const auto & message) {
        Json poses = Json::array();
        for (const auto & pose : message.poses) {
          poses.push_back(pose_stamped_json(pose));
        }
        return Json{{"header", header_json(message.header)}, {"poses", std::move(poses)}};
      });
    odom_subscription_ = subscribe<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos,
      [](const auto & message) {
        return Json{
          {"header", header_json(message.header)}, {"child_frame_id", message.child_frame_id},
          {"pose", {{"pose", pose_json(
                message.pose.pose)}, {"covariance", message.pose.covariance}}},
          {"twist", {{"twist", {
                {"linear", vector_json(message.twist.twist.linear)},
                {"angular", vector_json(message.twist.twist.angular)},
              }}, {"covariance", message.twist.covariance}}},
        };
      });
    amcl_pose_subscription_ = subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(1).reliable(),
      [](const auto & message) {
        return Json{
          {"header", header_json(message.header)},
          {"pose", {
              {"pose", pose_json(message.pose.pose)},
              {"covariance", message.pose.covariance},
            }},
        };
      });
    control_state_subscription_ = subscribe<std_msgs::msg::String>(
      "/control_center/main_state", rclcpp::QoS(1),
      [](const auto & message) {return Json{{"data", message.data}};});
    command_subscription_ = subscribe<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd", rclcpp::QoS(1),
      [](const auto & message) {
        return Json{
          {"header", header_json(message.header)},
          {"drive", {
              {"steering_angle", message.drive.steering_angle},
              {"steering_angle_velocity", message.drive.steering_angle_velocity},
              {"speed", message.drive.speed}, {"acceleration", message.drive.acceleration},
              {"jerk", message.drive.jerk},
            }},
        };
      });
    image_demand_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      const std::string topic = "/camera/camera/color/image_raw/compressed";
      if (!server_->has_interest(topic)) {camera_subscription_.reset(); return;}
      if (camera_subscription_) {return;}
      camera_subscription_ = subscribe<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::SensorDataQoS().keep_last(1), [](const auto & message) {
          return Json{{"header", header_json(message.header)}, {"format", message.format},
            {"data", Json::binary(message.data)}};
        });
    });
    detection_subscription_ = subscribe<carkit_perception_msgs::msg::YoloDetection2DArray>(
      "/yolo/detections_2d", sensor_qos,
      [](const auto & message) {
        Json detections = Json::array();
        for (const auto & detection : message.detections) {
          detections.push_back(detection_json(detection));
        }
        Json lights = Json::array();
        for (const auto & light : message.traffic_lights) {
          lights.push_back(
          {
            {"detection", detection_json(light.detection)},
            {"traffic_light_color", light.traffic_light_color},
          });
        }
        return Json{
          {"header", header_json(message.header)}, {"image_width", message.image_width},
          {"image_height", message.image_height}, {"detections", std::move(detections)},
          {"traffic_lights", std::move(lights)},
        };
      });
    battery_subscription_ = subscribe<sensor_msgs::msg::BatteryState>(
      "/battery_state", sensor_qos,
      [](const auto & message) {
        return Json{{"header", header_json(message.header)}, {"voltage", message.voltage},
          {"percentage", message.percentage}};
      });
#ifdef CARKIT_HAS_VESC_MSGS
    vesc_subscription_ = subscribe<vesc_msgs::msg::VescStateStamped>(
      "/sensors/core", sensor_qos,
      [](const auto & message) {
        return Json{{"header", header_json(message.header)},
          {"state", {{"voltage_input", message.state.voltage_input}}}};
      });
#endif
  }

  void publish_from_browser(const std::string & topic, const Json & message)
  {
    try {
      if (topic == "/teleop" && keyboard_enabled_) {
        const auto drive = message.at("drive");
        const double speed = drive.at("speed").get<double>();
        const double steering = drive.at("steering_angle").get<double>();
        ackermann_msgs::msg::AckermannDriveStamped output;
        output.header.stamp = now();
        output.header.frame_id = "base_link";
        if (std::isfinite(speed) && std::isfinite(steering)) {
          output.drive.speed = std::clamp(speed, -max_reverse_speed_, max_speed_);
          output.drive.steering_angle = std::clamp(steering, -max_steering_, max_steering_);
        }
        teleop_publisher_->publish(output);
      } else if (topic == "/control_center/emergency_stop") {
        std_msgs::msg::Bool output;
        output.data = message.at("data").get<bool>();
        estop_publisher_->publish(output);
      } else if (topic == "/enable_autonomous_control") {
        std_msgs::msg::Int8 output;
        output.data = static_cast<std::int8_t>(std::clamp(message.value("data", 0), -128, 127));
        autonomous_publisher_->publish(output);
      } else if (topic == "/goal_pose") {
        geometry_msgs::msg::PoseStamped output;
        output.header = parse_header(message.value("header", Json::object()));
        output.header.stamp = now();
        if (output.header.frame_id.empty()) {
          output.header.frame_id = "map";
        }
        output.pose = parse_pose(message.value("pose", Json::object()));
        goal_pose_publisher_->publish(output);
      } else if (topic == "/initialpose") {
        geometry_msgs::msg::PoseWithCovarianceStamped output;
        output.header = parse_header(message.value("header", Json::object()));
        output.header.stamp = now();
        if (output.header.frame_id.empty()) {
          output.header.frame_id = "map";
        }
        const auto pose_with_covariance = message.value("pose", Json::object());
        output.pose.pose = parse_pose(pose_with_covariance.value("pose", Json::object()));
        const auto covariance = pose_with_covariance.value("covariance", std::vector<double>{});
        std::copy_n(
          covariance.begin(), std::min(covariance.size(), output.pose.covariance.size()),
          output.pose.covariance.begin());
        initial_pose_publisher_->publish(output);
      }
    } catch (const Json::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Rejected malformed browser publication on %s: %s",
        topic.c_str(), error.what());
    }
  }

  std::unique_ptr<WebSocketServer> server_;
  rclcpp::TimerBase::SharedPtr image_demand_timer_;
  bool keyboard_enabled_{true};
  double max_speed_{0.8}, max_reverse_speed_{0.8}, max_steering_{0.34};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr teleop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr autonomous_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_subscription_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscription_;
  rclcpp::Subscription<carkit_perception_msgs::msg::YoloDetection2DArray>::SharedPtr
    detection_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
#ifdef CARKIT_HAS_VESC_MSGS
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_subscription_;
#endif
};


}  // namespace carkit_web_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<carkit_web_bridge::BridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("carkit_web_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
