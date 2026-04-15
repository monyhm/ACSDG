//============================================================================
// echoguard_node.cpp — ROS2 node for the Echodyne EchoGuard phased-array
// radar.  See echoguard_node.hpp for full specification.
//============================================================================

#include "acsdg_sensors_hw/echoguard_node.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>

namespace acsdg_sensors_hw {

// ── Constructor ─────────────────────────────────────────────────────────────

EchoGuardNode::EchoGuardNode()
: SensorHwInterface("echoguard_node")
{}

// ── initialize() ────────────────────────────────────────────────────────────

void EchoGuardNode::initialize()
{
  // Declare parameters
  use_real_hardware_ = declare_parameter<bool>("use_real_hardware", false);
  sensor_id_         = declare_parameter<int>("sensor_id", 1);
  frame_id_          = declare_parameter<std::string>("frame_id", "echoguard_1");
  publish_rate_      = declare_parameter<double>("detection_range", 1000.0);
  // (detection_range stored in publish_rate_ slot — use dedicated variable)
  double detection_range = publish_rate_;
  publish_rate_          = 25.0;  // fixed EchoGuard update rate

  sensor_x_   = declare_parameter<double>("sensor_x",   0.0);
  sensor_y_   = declare_parameter<double>("sensor_y",   0.0);
  sensor_z_   = declare_parameter<double>("sensor_z",   1.5);
  sensor_yaw_ = declare_parameter<double>("sensor_yaw", 0.0);

  hw_host_ = declare_parameter<std::string>("hw_host", "192.168.1.100");
  hw_port_ = declare_parameter<int>("hw_port", 5555);

  // Build topic names from sensor_id
  std::string id_str     = std::to_string(sensor_id_);
  std::string track_topic  = "/sensors/echoguard_" + id_str + "/tracks";
  std::string status_topic = "/sensors/echoguard_" + id_str + "/status";
  std::string gz_topic     = "/echoguard_" + id_str + "/radar_returns";

  // Publishers (same regardless of mode — key design requirement)
  track_pub_  = create_publisher<acsdg_msgs::msg::RadarTrack>(
    track_topic,  rclcpp::QoS(50));
  status_pub_ = create_publisher<std_msgs::msg::String>(
    status_topic, rclcpp::QoS(10));

  if (!use_real_hardware_) {
    // SIM MODE: subscribe to bridged Gazebo topic
    gz_sub_ = create_subscription<std_msgs::msg::String>(
      gz_topic, rclcpp::QoS(50),
      [this](std_msgs::msg::String::SharedPtr msg) {
        onRadarReturns(msg);
      });

    RCLCPP_INFO(get_logger(),
      "EchoGuardNode [SIM] sensor_id=%d  gz_topic=%s  tracks→%s",
      sensor_id_, gz_topic.c_str(), track_topic.c_str());
  } else {
    // HW MODE: hardware driver stub
    RCLCPP_WARN(get_logger(),
      "HARDWARE MODE: connect to EchoGuard Ethernet API at %s:%d",
      hw_host_.c_str(), hw_port_);
    RCLCPP_WARN(get_logger(),
      "  Implement EchoGuard SDK integration here.  "
      "Publishing placeholder tracks on %s", track_topic.c_str());

    // In real implementation: open TCP socket to hw_host_:hw_port_,
    // receive JSON track stream, parse with parseJson(), call
    // associateAndPublish().  The timer below would drive polling.
  }

  // 1 Hz status publisher (both modes)
  status_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() { publishStatus(); });

  RCLCPP_INFO(get_logger(),
    "EchoGuardNode ready  id=%d  range=%.0fm  frame=%s  hw=%s",
    sensor_id_, detection_range, frame_id_.c_str(),
    use_real_hardware_ ? "REAL" : "SIM");
}

void EchoGuardNode::startStreaming()
{
  // In sim mode, data arrives via subscription callback.
  // In HW mode the timer is started here for polling.
  if (use_real_hardware_) {
    auto ms = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / publish_rate_));
    hw_poll_timer_ = create_wall_timer(ms, [this]() {
      // HW stub: nothing to publish until SDK is wired
      RCLCPP_DEBUG(get_logger(),
        "HARDWARE MODE: polling EchoGuard at %s:%d",
        hw_host_.c_str(), hw_port_);
    });
  }
  RCLCPP_INFO(get_logger(), "EchoGuardNode streaming at %.0f Hz", publish_rate_);
}

void EchoGuardNode::stopStreaming()
{
  if (hw_poll_timer_) hw_poll_timer_->cancel();
  RCLCPP_INFO(get_logger(), "EchoGuardNode stopped");
}

// ── Sim-mode callback ────────────────────────────────────────────────────────

void EchoGuardNode::onRadarReturns(const std_msgs::msg::String::SharedPtr msg)
{
  auto returns = parseJson(msg->data);
  const rclcpp::Time now = this->now();
  associateAndPublish(returns, now);
}

// ── Track association and publish ────────────────────────────────────────────

void EchoGuardNode::associateAndPublish(
  const std::vector<RadarReturn> & returns,
  const rclcpp::Time & now)
{
  static constexpr double kGate = 5.0;    // metres, nearest-neighbour gate
  static constexpr double kDropAge = 0.5; // seconds without update → drop track

  // Drop stale tracks
  for (auto it = tracks_.begin(); it != tracks_.end(); ) {
    double age = (now - it->second.last_seen).seconds();
    if (age > kDropAge) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }

  // Associate each return to existing track or create new one
  for (const auto & ret : returns) {
    // Convert spherical (range, az, el) → Cartesian sensor-frame, then world-frame
    double az_r = ret.azimuth_deg   * M_PI / 180.0;
    double el_r = ret.elevation_deg * M_PI / 180.0;

    // Sensor body-frame Cartesian
    double lx = ret.range * std::cos(el_r) * std::cos(az_r);
    double ly = ret.range * std::cos(el_r) * std::sin(az_r);
    double lz = ret.range * std::sin(el_r);

    // Rotate back to world frame (add sensor yaw)
    double cosY = std::cos(sensor_yaw_);
    double sinY = std::sin(sensor_yaw_);
    double wx   = sensor_x_ + lx * cosY - ly * sinY;
    double wy   = sensor_y_ + lx * sinY + ly * cosY;
    double wz   = sensor_z_ + lz;

    // Nearest-neighbour search
    uint32_t best_id   = 0;
    double   best_dist = std::numeric_limits<double>::max();
    for (auto & [id, ts] : tracks_) {
      double dx = wx - ts.x, dy = wy - ts.y, dz = wz - ts.z;
      double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (d < best_dist) { best_dist = d; best_id = id; }
    }

    if (best_dist <= kGate && best_id != 0) {
      auto & ts = tracks_.at(best_id);
      ts.x  = wx;  ts.y  = wy;  ts.z  = wz;
      ts.last_seen = now;
      // Confidence from detection range: closer = higher confidence
      ts.confidence = static_cast<float>(
        std::max(0.0, 1.0 - ret.range / 1000.0));
    } else {
      uint32_t nid = next_track_id_++;
      RadarTrackState ts;
      ts.id         = nid;
      ts.x          = wx;  ts.y  = wy;  ts.z  = wz;
      ts.confidence = static_cast<float>(
        std::max(0.0, 1.0 - ret.range / 1000.0));
      ts.last_seen  = now;
      tracks_[nid]  = ts;
    }
  }

  // Publish all active tracks
  active_track_count_ = static_cast<int>(tracks_.size());
  for (auto & [id, ts] : tracks_) {
    acsdg_msgs::msg::RadarTrack msg;
    msg.id          = ts.id;
    msg.position.x  = ts.x;
    msg.position.y  = ts.y;
    msg.position.z  = ts.z;
    msg.velocity.x  = ts.vx;
    msg.velocity.y  = ts.vy;
    msg.velocity.z  = ts.vz;
    msg.confidence  = ts.confidence;
    msg.stamp       = now;
    track_pub_->publish(msg);
  }
}

// ── Status publisher ─────────────────────────────────────────────────────────

void EchoGuardNode::publishStatus()
{
  std::ostringstream ss;
  ss << "{\"sensor\":\"EchoGuard\","
     << "\"mode\":\""    << (use_real_hardware_ ? "HW" : "SIM") << "\","
     << "\"id\":"        << sensor_id_            << ","
     << "\"tracks\":"    << active_track_count_   << ","
     << "\"health\":\"OK\"}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  status_pub_->publish(msg);
}

// ── JSON parser ──────────────────────────────────────────────────────────────
//
// Expected format (from echoguard_radar_plugin):
//   [{"range":R,"az_deg":A,"el_deg":E,"doppler":D,"prob":P}, ...]

std::vector<RadarReturn> EchoGuardNode::parseJson(const std::string & json)
{
  std::vector<RadarReturn> out;
  if (json.empty() || json == "[]") return out;

  // Simple field extraction without external JSON library
  size_t pos = 0;
  while ((pos = json.find('{', pos)) != std::string::npos) {
    size_t end = json.find('}', pos);
    if (end == std::string::npos) break;

    std::string obj = json.substr(pos + 1, end - pos - 1);
    RadarReturn ret;

    auto extractDouble = [&](const std::string & key) -> double {
      size_t k = obj.find("\"" + key + "\":");
      if (k == std::string::npos) return 0.0;
      k += key.size() + 3;
      return std::stod(obj.substr(k));
    };

    ret.range          = extractDouble("range");
    ret.azimuth_deg    = extractDouble("az_deg");
    ret.elevation_deg  = extractDouble("el_deg");
    ret.doppler_mps    = extractDouble("doppler");

    out.push_back(ret);
    pos = end + 1;
  }
  return out;
}

}  // namespace acsdg_sensors_hw

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors_hw::EchoGuardNode>();
  node->initialize();
  node->startStreaming();
  rclcpp::spin(node);
  node->stopStreaming();
  rclcpp::shutdown();
  return 0;
}
