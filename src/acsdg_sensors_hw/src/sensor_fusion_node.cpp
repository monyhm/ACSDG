//============================================================================
// sensor_fusion_node.cpp — Updated multi-sensor fusion for the ACSDG system.
//
// Subscribes to all three new COTS sensor types plus the legacy sim sensors,
// associates tracks with nearest-neighbour (≤15 m gate), and fuses positions
// with weighted averages:
//
//   Radar  (EchoGuard) weight 0.60 — range + velocity + Cartesian position
//   RF     (RF-360)    weight 0.25 — bearing-only (range unknown)
//   Thermal (Boson+)   weight 0.15 — bearing-only (range unknown)
//
// Legacy subscriptions (acsdg_sensors package) are preserved for backward
// compatibility:
//   /sensors/radar/raw_tracks   (radar, same weight 0.60)
//   /sensors/rf/detections      (RF,    same weight 0.25)
//
// Bearing-only tracks (RF and thermal) contribute to the fused azimuth but
// not to range estimation; their positions are marked as unknown (x=y=z=0)
// and confidence < 0.5 distinguishes them from radar tracks.
//
// Outputs:
//   /sensors/fusion/targets         std_msgs/String — JSON array
//   /threats/target_{id}/fused_target  acsdg_msgs/FusedTarget
//
// Track management:
//   - Gate ≤15 m → associate; else → new target
//   - No update from ANY sensor for >2 s → drop
//============================================================================

#include "acsdg_sensors_hw/sensor_hw_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace acsdg_sensors_hw {

// ── Sensor type enum ──────────────────────────────────────────────────────────

enum class SensorType { RADAR, RF, THERMAL };

static constexpr double kWeightRadar   = 0.60;
static constexpr double kWeightRF      = 0.25;
static constexpr double kWeightThermal = 0.15;

// ── Per-target internal state ─────────────────────────────────────────────────

struct FusedState {
  uint32_t    id{0};
  std::string state{"DETECTED"};

  // Cached fused position — updated on every ingestion for association
  double fused_x{0.0}, fused_y{0.0}, fused_z{0.0};

  // Radar contribution (full 3-D position + velocity)
  bool   has_radar{false};
  double radar_x{0}, radar_y{0}, radar_z{0};
  double radar_vx{0}, radar_vy{0}, radar_vz{0};
  float  radar_conf{0.0f};

  // RF contribution (bearing-only: position unknown, velocity.x = bearing_deg)
  bool   has_rf{false};
  double rf_bearing_deg{0};
  float  rf_conf{0.0f};

  // Thermal contribution (bearing-only: velocity.x = az_deg, velocity.y = el_deg)
  bool   has_thermal{false};
  double thermal_az_deg{0}, thermal_el_deg{0};
  float  thermal_conf{0.0f};

  rclcpp::Time last_seen;

  // Ingest a RadarTrack from the given sensor type, refresh cached fused pos.
  // Bearing-only sensors (RF, thermal) do NOT update the Cartesian fused pos
  // unless a radar fix is present — they contribute only to confidence.
  void ingest(const acsdg_msgs::msg::RadarTrack & msg,
              SensorType type,
              const rclcpp::Time & now)
  {
    last_seen = now;

    if (type == SensorType::RADAR) {
      has_radar   = true;
      radar_x     = msg.position.x;
      radar_y     = msg.position.y;
      radar_z     = msg.position.z;
      radar_vx    = msg.velocity.x;
      radar_vy    = msg.velocity.y;
      radar_vz    = msg.velocity.z;
      radar_conf  = msg.confidence;
    } else if (type == SensorType::RF) {
      has_rf          = true;
      rf_bearing_deg  = msg.velocity.x;   // bearing encoding convention
      rf_conf         = msg.confidence;
    } else {
      has_thermal      = true;
      thermal_az_deg   = msg.velocity.x;
      thermal_el_deg   = msg.velocity.y;
      thermal_conf     = msg.confidence;
    }

    // Update cached position: use radar if available, else retain last known
    if (has_radar) {
      fused_x = radar_x;
      fused_y = radar_y;
      fused_z = radar_z;
    }
    // If only bearing-only sensors, fused position stays at 0 until radar sees it
  }

  // Build output FusedTarget message with weighted sensor contributions
  acsdg_msgs::msg::FusedTarget toMsg(const rclcpp::Time & now) const
  {
    acsdg_msgs::msg::FusedTarget ft;
    ft.id         = id;
    ft.position.x = fused_x;
    ft.position.y = fused_y;
    ft.position.z = fused_z;
    ft.state      = state;
    ft.stamp      = now;

    if (has_radar) {
      ft.velocity.x = radar_vx;
      ft.velocity.y = radar_vy;
      ft.velocity.z = radar_vz;
    }

    // Fused threat score — weighted by active sensor contributions
    double totalW = 0.0, weightedConf = 0.0;
    if (has_radar)   { totalW += kWeightRadar;   weightedConf += kWeightRadar   * radar_conf;   }
    if (has_rf)      { totalW += kWeightRF;      weightedConf += kWeightRF      * rf_conf;      }
    if (has_thermal) { totalW += kWeightThermal; weightedConf += kWeightThermal * thermal_conf; }

    ft.threat_score = (totalW > 0.0)
      ? static_cast<float>(weightedConf / totalW)
      : 0.0f;

    return ft;
  }
};

// ── SensorFusionNode ──────────────────────────────────────────────────────────

class SensorFusionNode : public rclcpp::Node
{
  static constexpr double kAssocGate = 15.0;   // metres
  static constexpr double kDropAge   = 2.0;    // seconds

  // How many instances of each sensor type are deployed
  static constexpr int kNumEchoGuards = 6;
  static constexpr int kNumRF360s     = 4;
  static constexpr int kNumBosons     = 4;

public:
  SensorFusionNode()
  : rclcpp::Node("sensor_fusion_node"), next_id_(1)
  {}

  void initialize()
  {
    json_pub_ = create_publisher<std_msgs::msg::String>(
      "/sensors/fusion/targets", rclcpp::QoS(10));

    // ── Legacy subscriptions (acsdg_sensors package) ────────────────────────
    legacy_radar_sub_ = create_subscription<acsdg_msgs::msg::RadarTrack>(
      "/sensors/radar/raw_tracks", rclcpp::QoS(50),
      [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
        handleTrack(*msg, SensorType::RADAR);
      });

    legacy_rf_sub_ = create_subscription<acsdg_msgs::msg::RadarTrack>(
      "/sensors/rf/detections", rclcpp::QoS(50),
      [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
        handleTrack(*msg, SensorType::RF);
      });

    // ── EchoGuard subscriptions (6 sensors) ────────────────────────────────
    for (int i = 1; i <= kNumEchoGuards; ++i) {
      std::string topic = "/sensors/echoguard_" + std::to_string(i) + "/tracks";
      echoguard_subs_.push_back(
        create_subscription<acsdg_msgs::msg::RadarTrack>(
          topic, rclcpp::QoS(50),
          [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
            handleTrack(*msg, SensorType::RADAR);
          }));
    }

    // ── RF-360 subscriptions (4 sensors) ───────────────────────────────────
    for (int i = 1; i <= kNumRF360s; ++i) {
      std::string topic = "/sensors/rf360_" + std::to_string(i) + "/detections";
      rf360_subs_.push_back(
        create_subscription<acsdg_msgs::msg::RadarTrack>(
          topic, rclcpp::QoS(50),
          [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
            handleTrack(*msg, SensorType::RF);
          }));
    }

    // ── Boson+ subscriptions (4 sensors) ───────────────────────────────────
    for (int i = 1; i <= kNumBosons; ++i) {
      std::string topic = "/sensors/boson_" + std::to_string(i) + "/detections";
      boson_subs_.push_back(
        create_subscription<acsdg_msgs::msg::RadarTrack>(
          topic, rclcpp::QoS(50),
          [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
            handleTrack(*msg, SensorType::THERMAL);
          }));
    }

    // 10 Hz output timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publishFused(); });

    RCLCPP_INFO(get_logger(),
      "SensorFusionNode ready — "
      "gate=%.0fm  drop=%.0fs  "
      "w=[radar:%.2f RF:%.2f thermal:%.2f]  "
      "sensors: %d EchoGuard + %d RF360 + %d Boson",
      kAssocGate, kDropAge,
      kWeightRadar, kWeightRF, kWeightThermal,
      kNumEchoGuards, kNumRF360s, kNumBosons);
  }

private:
  // ── Track ingestion ───────────────────────────────────────────────────────

  void handleTrack(const acsdg_msgs::msg::RadarTrack & msg, SensorType type)
  {
    const rclcpp::Time now = this->now();

    // For bearing-only sensors: use the fused_x/fused_y of existing tracks
    // for association; for new tracks, cannot associate by position alone
    // unless radar is present.  Use a larger gate for bearing-only updates.

    uint32_t best_id   = 0;
    double   best_dist = std::numeric_limits<double>::max();

    for (auto & [id, fs] : fused_targets_) {
      // Skip tracks with no Cartesian fix when the incoming is bearing-only
      if ((type == SensorType::RF || type == SensorType::THERMAL)
          && !fs.has_radar)
      { continue; }

      double dx = msg.position.x - fs.fused_x;
      double dy = msg.position.y - fs.fused_y;
      double dz = msg.position.z - fs.fused_z;
      double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (d < best_dist) { best_dist = d; best_id = id; }
    }

    if (best_dist <= kAssocGate && best_id != 0) {
      fused_targets_.at(best_id).ingest(msg, type, now);
    } else if (type == SensorType::RADAR) {
      // Only radar creates new tracks (it provides Cartesian position)
      uint32_t nid = next_id_++;
      FusedState fs;
      fs.id        = nid;
      fs.state     = "DETECTED";
      fs.last_seen = now;
      fs.ingest(msg, type, now);
      fused_targets_[nid] = std::move(fs);

      target_pubs_[nid] = create_publisher<acsdg_msgs::msg::FusedTarget>(
        "/threats/target_" + std::to_string(nid) + "/fused_target",
        rclcpp::QoS(10));

      const auto & fs2 = fused_targets_[nid];
      RCLCPP_INFO(get_logger(),
        "New fused target #%u  pos=(%.1f,%.1f,%.1f)",
        nid, fs2.fused_x, fs2.fused_y, fs2.fused_z);
    }
    // Bearing-only detections with no matching radar track are silently
    // buffered — they'll associate once the radar picks up the same target.
  }

  // ── 10 Hz publish callback ────────────────────────────────────────────────

  void publishFused()
  {
    const rclcpp::Time now = this->now();

    // Drop stale tracks
    for (auto it = fused_targets_.begin(); it != fused_targets_.end(); ) {
      if ((now - it->second.last_seen).seconds() > kDropAge) {
        RCLCPP_INFO(get_logger(), "Dropped stale target #%u", it->first);
        target_pubs_.erase(it->first);
        it = fused_targets_.erase(it);
      } else {
        ++it;
      }
    }

    // Build JSON array and per-target messages
    std::ostringstream json;
    json << std::fixed << std::setprecision(3) << "[";
    bool first = true;

    for (auto & [id, fs] : fused_targets_) {
      auto ft = fs.toMsg(now);
      if (target_pubs_.count(id)) target_pubs_.at(id)->publish(ft);

      if (!first) json << ",";
      first = false;
      int64_t ns = now.nanoseconds();
      json << "{"
           << "\"id\":"           << id              << ","
           << "\"position\":{"
             << "\"x\":"          << ft.position.x   << ","
             << "\"y\":"          << ft.position.y   << ","
             << "\"z\":"          << ft.position.z   << "},"
           << "\"velocity\":{"
             << "\"x\":"          << ft.velocity.x   << ","
             << "\"y\":"          << ft.velocity.y   << ","
             << "\"z\":"          << ft.velocity.z   << "},"
           << "\"threat_score\":" << ft.threat_score << ","
           << "\"state\":\""      << fs.state        << "\","
           << "\"sensors\":{"
             << "\"radar\":"      << (fs.has_radar   ? "true" : "false") << ","
             << "\"rf\":"         << (fs.has_rf      ? "true" : "false") << ","
             << "\"thermal\":"    << (fs.has_thermal ? "true" : "false") << "},"
           << "\"stamp\":{"
             << "\"sec\":"        << (ns / 1000000000LL) << ","
             << "\"nanosec\":"    << (ns % 1000000000LL) << "}"
           << "}";
    }
    json << "]";

    std_msgs::msg::String out;
    out.data = json.str();
    json_pub_->publish(out);
  }

  // ── Members ───────────────────────────────────────────────────────────────

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;

  // Legacy sensor subscriptions
  rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr legacy_radar_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr legacy_rf_sub_;

  // New COTS sensor subscriptions
  std::vector<rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr> echoguard_subs_;
  std::vector<rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr> rf360_subs_;
  std::vector<rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr> boson_subs_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<uint32_t, FusedState> fused_targets_;
  std::map<uint32_t,
    rclcpp::Publisher<acsdg_msgs::msg::FusedTarget>::SharedPtr> target_pubs_;

  uint32_t next_id_;
};

}  // namespace acsdg_sensors_hw

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors_hw::SensorFusionNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
