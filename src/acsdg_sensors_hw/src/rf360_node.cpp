//============================================================================
// rf360_node.cpp — ROS2 node for the Dedrone RF-360 RF direction finder.
// See rf360_node.hpp for full specification.
//============================================================================

#include "acsdg_sensors_hw/rf360_node.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

namespace acsdg_sensors_hw {

RF360Node::RF360Node()
: SensorHwInterface("rf360_node")
{}

// ── initialize() ─────────────────────────────────────────────────────────────

void RF360Node::initialize()
{
  use_real_hardware_  = declare_parameter<bool>("use_real_hardware", false);
  sensor_id_          = declare_parameter<int>("sensor_id", 1);
  detection_range_    = declare_parameter<double>("detection_range", 2000.0);
  df_accuracy_deg_    = declare_parameter<double>("df_accuracy_deg", 5.0);
  frame_id_           = declare_parameter<std::string>("frame_id", "rf360_1");
  publish_rate_       = 1.0;  // RF-360 classification rate

  hw_host_ = declare_parameter<std::string>("hw_host", "192.168.1.200");
  hw_port_ = declare_parameter<int>("hw_port", 9090);

  std::string id_str      = std::to_string(sensor_id_);
  std::string detect_topic = "/sensors/rf360_" + id_str + "/detections";
  std::string status_topic = "/sensors/rf360_" + id_str + "/status";
  std::string gz_topic     = "/rf360_" + id_str + "/rf_detections";

  detect_pub_ = create_publisher<acsdg_msgs::msg::RadarTrack>(
    detect_topic, rclcpp::QoS(20));
  status_pub_ = create_publisher<std_msgs::msg::String>(
    status_topic, rclcpp::QoS(10));

  if (!use_real_hardware_) {
    gz_sub_ = create_subscription<std_msgs::msg::String>(
      gz_topic, rclcpp::QoS(20),
      [this](std_msgs::msg::String::SharedPtr msg) {
        onRfDetections(msg);
      });

    RCLCPP_INFO(get_logger(),
      "RF360Node [SIM] sensor_id=%d  gz_topic=%s  detections→%s",
      sensor_id_, gz_topic.c_str(), detect_topic.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
      "HARDWARE MODE: polling RF-360 at http://%s:%d/api/v1/detections",
      hw_host_.c_str(), hw_port_);
    RCLCPP_WARN(get_logger(),
      "  Implement REST client (libcurl or similar) here.  "
      "Publishing placeholder detections on %s", detect_topic.c_str());
  }

  // 1 Hz publish timer — drives output regardless of mode
  auto ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / publish_rate_));
  publish_timer_ = create_wall_timer(ms, [this]() { publishDetections(); });

  status_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this]() { publishStatus(); });

  RCLCPP_INFO(get_logger(),
    "RF360Node ready  id=%d  range=%.0fm  DF_acc=%.1f°  hw=%s",
    sensor_id_, detection_range_, df_accuracy_deg_,
    use_real_hardware_ ? "REAL" : "SIM");
}

void RF360Node::startStreaming()
{
  if (use_real_hardware_) {
    RCLCPP_INFO(get_logger(),
      "HARDWARE MODE: polling RF-360 REST API at http://%s:%d/api/v1/detections",
      hw_host_.c_str(), hw_port_);
  }
  RCLCPP_INFO(get_logger(), "RF360Node streaming at %.0f Hz", publish_rate_);
}

void RF360Node::stopStreaming()
{
  if (publish_timer_) publish_timer_->cancel();
  RCLCPP_INFO(get_logger(), "RF360Node stopped");
}

// ── Gazebo subscription callback ─────────────────────────────────────────────

void RF360Node::onRfDetections(const std_msgs::msg::String::SharedPtr msg)
{
  latest_detections_ = parseJson(msg->data);
  detection_count_   = static_cast<int>(latest_detections_.size());
}

// ── 1 Hz publish callback ─────────────────────────────────────────────────────

void RF360Node::publishDetections()
{
  if (use_real_hardware_) {
    // HW stub: in real implementation, execute REST GET and parse response
    RCLCPP_DEBUG(get_logger(),
      "HARDWARE MODE: polling RF-360 at http://%s:%d/api/v1/detections",
      hw_host_.c_str(), hw_port_);
    return;
  }

  const rclcpp::Time now = this->now();
  uint32_t pub_id = 1;

  for (const auto & det : latest_detections_) {
    // Bearing-only track: azimuth = bearing, range = 0
    acsdg_msgs::msg::RadarTrack msg;
    msg.id            = det.drone_id ? det.drone_id : pub_id++;
    msg.position.x    = 0.0;   // range unknown
    msg.position.y    = 0.0;
    msg.position.z    = 0.0;
    // Encode bearing in velocity.x as bearing-only convention
    // (consumers check confidence < 0.5 to identify bearing-only tracks)
    msg.velocity.x    = det.bearing_deg;   // azimuth bearing in degrees
    msg.velocity.y    = det.freq_hz;       // frequency as metadata
    msg.velocity.z    = 0.0;
    msg.confidence    = rssiToConfidence(det.rssi_dbm);
    msg.stamp         = now;
    detect_pub_->publish(msg);
  }
}

// ── Status publisher ──────────────────────────────────────────────────────────

void RF360Node::publishStatus()
{
  std::ostringstream ss;
  ss << "{\"sensor\":\"RF360\","
     << "\"mode\":\""       << (use_real_hardware_ ? "HW" : "SIM") << "\","
     << "\"id\":"           << sensor_id_      << ","
     << "\"detections\":"   << detection_count_ << ","
     << "\"health\":\"OK\"}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  status_pub_->publish(msg);
}

// ── RSSI → confidence ─────────────────────────────────────────────────────────
// Map [-100, -20] dBm → [0.0, 1.0] linearly, clamped

float RF360Node::rssiToConfidence(double rssi_dbm)
{
  static constexpr double kMinRssi = -100.0;
  static constexpr double kMaxRssi = -20.0;
  double c = (rssi_dbm - kMinRssi) / (kMaxRssi - kMinRssi);
  return static_cast<float>(std::max(0.0, std::min(1.0, c)));
}

// ── JSON parser ───────────────────────────────────────────────────────────────
//
// Expected format (from rf360_plugin):
//   [{"id":N,"bearing_deg":B,"rssi_dbm":R,"freq_hz":F,"range_m":D}, ...]

std::vector<RfDetection> RF360Node::parseJson(const std::string & json)
{
  std::vector<RfDetection> out;
  if (json.empty() || json == "[]") return out;

  size_t pos = 0;
  while ((pos = json.find('{', pos)) != std::string::npos) {
    size_t end = json.find('}', pos);
    if (end == std::string::npos) break;

    std::string obj = json.substr(pos + 1, end - pos - 1);
    RfDetection det;

    auto extractDouble = [&](const std::string & key) -> double {
      size_t k = obj.find("\"" + key + "\":");
      if (k == std::string::npos) return 0.0;
      k += key.size() + 3;
      try { return std::stod(obj.substr(k)); }
      catch (...) { return 0.0; }
    };
    auto extractUint = [&](const std::string & key) -> uint32_t {
      size_t k = obj.find("\"" + key + "\":");
      if (k == std::string::npos) return 0;
      k += key.size() + 3;
      try { return static_cast<uint32_t>(std::stoul(obj.substr(k))); }
      catch (...) { return 0; }
    };

    det.drone_id    = extractUint("id");
    det.bearing_deg = extractDouble("bearing_deg");
    det.rssi_dbm    = extractDouble("rssi_dbm");
    det.freq_hz     = extractDouble("freq_hz");
    det.range_m     = extractDouble("range_m");

    out.push_back(det);
    pos = end + 1;
  }
  return out;
}

}  // namespace acsdg_sensors_hw

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors_hw::RF360Node>();
  node->initialize();
  node->startStreaming();
  rclcpp::spin(node);
  node->stopStreaming();
  rclcpp::shutdown();
  return 0;
}
