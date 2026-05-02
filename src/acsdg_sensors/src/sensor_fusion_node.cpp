//============================================================================
// sensor_fusion_node.cpp — Multi-sensor fusion for the ACSDG system.
//
// Subscribes to both radar and RF tracks, associates them using nearest-
// neighbour matching (≤15 m gate), and fuses positions with a weighted
// average (radar 0.7, RF 0.3).  Outputs:
//
//   /sensors/fusion/targets         std_msgs/String  — JSON array of all
//                                                      active FusedTargets
//   /threats/target_{id}/fused_target  acsdg_msgs/FusedTarget — one per target
//
// Track management:
//   - New detection from either sensor → nearest-neighbour association
//   - If no match within gate → new fused target assigned
//   - Track not seen from ANY sensor for >2 s → dropped
//   - State set to DETECTED on first fusion
//============================================================================

#include "acsdg_sensors/sensor_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>

namespace acsdg_sensors {

// ── Per-target internal state ─────────────────────────────────────────────

struct FusedState {
  uint32_t    id{0};
  std::string state{"DETECTED"};

  // Cached fused position — updated on every detection, used for association
  double fused_x{0.0}, fused_y{0.0}, fused_z{0.0};

  // Radar contribution
  bool   has_radar{false};
  double radar_x{0}, radar_y{0}, radar_z{0};
  double radar_vx{0}, radar_vy{0}, radar_vz{0};
  float  radar_confidence{0.0f};

  // RF contribution
  bool   has_rf{false};
  double rf_x{0}, rf_y{0}, rf_z{0};
  double rf_vx{0}, rf_vy{0}, rf_vz{0};
  float  rf_confidence{0.0f};

  rclcpp::Time last_seen;

  // Ingest a detection and immediately refresh the cached fused position.
  // Weighted fusion: radar 0.7, RF 0.3.  Falls back to whichever is present.
  void ingest(const acsdg_msgs::msg::RadarTrack & msg,
              bool is_radar,
              const rclcpp::Time & now)
  {
    if (is_radar) {
      has_radar         = true;
      radar_x           = msg.position.x;
      radar_y           = msg.position.y;
      radar_z           = msg.position.z;
      radar_vx          = msg.velocity.x;
      radar_vy          = msg.velocity.y;
      radar_vz          = msg.velocity.z;
      radar_confidence  = msg.confidence;
    } else {
      has_rf            = true;
      rf_x              = msg.position.x;
      rf_y              = msg.position.y;
      rf_z              = msg.position.z;
      rf_vx             = msg.velocity.x;
      rf_vy             = msg.velocity.y;
      rf_vz             = msg.velocity.z;
      rf_confidence     = msg.confidence;
    }
    last_seen = now;

    // Recompute cached position for next association round
    if (has_radar && has_rf) {
      fused_x = 0.7 * radar_x + 0.3 * rf_x;
      fused_y = 0.7 * radar_y + 0.3 * rf_y;
      fused_z = 0.7 * radar_z + 0.3 * rf_z;
    } else if (has_radar) {
      fused_x = radar_x;  fused_y = radar_y;  fused_z = radar_z;
    } else {
      fused_x = rf_x;     fused_y = rf_y;     fused_z = rf_z;
    }
  }

  // Fill a FusedTarget message from current state
  acsdg_msgs::msg::FusedTarget toMsg(const rclcpp::Time & now) const
  {
    acsdg_msgs::msg::FusedTarget ft;
    ft.id           = id;
    ft.position.x   = fused_x;
    ft.position.y   = fused_y;
    ft.position.z   = fused_z;
    ft.state        = state;
    ft.stamp        = now;

    if (has_radar && has_rf) {
      ft.velocity.x   = 0.7 * radar_vx + 0.3 * rf_vx;
      ft.velocity.y   = 0.7 * radar_vy + 0.3 * rf_vy;
      ft.velocity.z   = 0.7 * radar_vz + 0.3 * rf_vz;
      ft.threat_score = 0.7f * radar_confidence + 0.3f * rf_confidence;
    } else if (has_radar) {
      ft.velocity.x   = radar_vx;
      ft.velocity.y   = radar_vy;
      ft.velocity.z   = radar_vz;
      ft.threat_score = radar_confidence;
    } else {
      ft.velocity.x   = rf_vx;
      ft.velocity.y   = rf_vy;
      ft.velocity.z   = rf_vz;
      ft.threat_score = rf_confidence;
    }
    return ft;
  }
};

// ── SensorFusionNode ─────────────────────────────────────────────────────

class SensorFusionNode : public rclcpp::Node
{
  static constexpr double kAssocGate = 15.0;  // metres
  static constexpr double kDropAge   = 2.0;   // seconds

public:
  SensorFusionNode()
  : rclcpp::Node("sensor_fusion_node"), next_id_(1)
  {}

  void initialize()
  {
    json_pub_ = create_publisher<std_msgs::msg::String>(
      "/sensors/fusion/targets", rclcpp::QoS(10));

    radar_sub_ = create_subscription<acsdg_msgs::msg::RadarTrack>(
      "/sensors/radar/raw_tracks", rclcpp::QoS(50),
      [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
        handleTrack(*msg, /*is_radar=*/true);
      });

    rf_sub_ = create_subscription<acsdg_msgs::msg::RadarTrack>(
      "/sensors/rf/detections", rclcpp::QoS(50),
      [this](acsdg_msgs::msg::RadarTrack::SharedPtr msg) {
        handleTrack(*msg, /*is_radar=*/false);
      });

    // Engagement outcomes. A target flagged NEUTRALIZED here is dropped
    // from the fused output so C2 doesn't keep re-assigning interceptors
    // to a ghost track the sim hasn't despawned yet.
    ack_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/engagement_ack", rclcpp::QoS(10),
      [this](std_msgs::msg::String::SharedPtr msg) { onAck(*msg); });

    // New wave → enemy drones are re-activated in-place. Clear the killed
    // set so radar tracks for the re-flying drones are no longer ignored.
    wave_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mission/wave_trigger", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          RCLCPP_INFO(get_logger(),
            "Wave trigger — clearing %zu killed-target ids", killed_ids_.size());
          killed_ids_.clear();
        }
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),   // 10 Hz output
      [this]() { publishFused(); });

    RCLCPP_INFO(get_logger(),
      "SensorFusionNode ready — gate=%.0fm, drop=%.0fs, w=[radar:0.7, RF:0.3]",
      kAssocGate, kDropAge);
  }

private:
  // ── Track ingestion (called on every radar or RF message) ──────────────

  void onAck(const std_msgs::msg::String & msg)
  {
    // Lightweight JSON parse — we only need target_id + outcome.
    const std::string & s = msg.data;
    auto find_int = [&](const std::string & key) -> int {
      auto k = s.find("\"" + key + "\":");
      if (k == std::string::npos) return 0;
      auto start = s.find_first_of("0123456789-", k);
      if (start == std::string::npos) return 0;
      auto end = s.find_first_not_of("0123456789-", start);
      return std::stoi(s.substr(start, end - start));
    };
    auto find_str = [&](const std::string & key) -> std::string {
      auto k = s.find("\"" + key + "\":\"");
      if (k == std::string::npos) return "";
      auto start = s.find("\"", k + key.size() + 3);
      if (start == std::string::npos) return "";
      auto end = s.find("\"", start + 1);
      return s.substr(start + 1, end - start - 1);
    };
    int tid              = find_int("target_id");
    std::string outcome  = find_str("outcome");
    if (tid <= 0) return;
    if (outcome == "NEUTRALIZED" || outcome == "BREACHED") {
      killed_ids_.insert(static_cast<uint32_t>(tid));
      fused_targets_.erase(static_cast<uint32_t>(tid));
      target_pubs_.erase(static_cast<uint32_t>(tid));
      RCLCPP_INFO(get_logger(),
        "Dropped target #%d from fusion (outcome=%s)", tid, outcome.c_str());
    }
  }

  void handleTrack(const acsdg_msgs::msg::RadarTrack & msg, bool is_radar)
  {
    // Ignore updates for targets we've already counted as killed — the
    // Gazebo model may still be publishing odometry until we despawn it.
    if (killed_ids_.count(msg.id)) return;

    const rclcpp::Time now = this->now();

    // Nearest-neighbour search over cached fused positions
    uint32_t best_id   = 0;
    double   best_dist = std::numeric_limits<double>::max();

    for (auto & [id, fs] : fused_targets_) {
      double dx = msg.position.x - fs.fused_x;
      double dy = msg.position.y - fs.fused_y;
      double dz = msg.position.z - fs.fused_z;
      double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (d < best_dist) { best_dist = d; best_id = id; }
    }

    if (best_dist <= kAssocGate && best_id != 0) {
      fused_targets_.at(best_id).ingest(msg, is_radar, now);
    } else {
      // New target. Prefer the upstream track id (e.g. radar_node sets
      // msg.id = enemy model index) so fused_target_N maps back to
      // enemy_N — lets BREACHED acks from enemy_driver correlate to a
      // fused track. Fall back to auto-increment if the id is zero or
      // already in use.
      uint32_t nid = msg.id;
      if (nid == 0 || fused_targets_.count(nid)) {
        nid = next_id_++;
      }
      if (nid >= next_id_) { next_id_ = nid + 1; }

      FusedState fs;
      fs.id        = nid;
      fs.state     = "DETECTED";
      fs.last_seen = now;
      fs.ingest(msg, is_radar, now);
      fused_targets_[nid] = std::move(fs);

      std::string topic =
        "/threats/target_" + std::to_string(nid) + "/fused_target";
      target_pubs_[nid] = create_publisher<acsdg_msgs::msg::FusedTarget>(
        topic, rclcpp::QoS(10));

      RCLCPP_INFO(get_logger(), "New fused target #%u  src=%s  pos=(%.1f,%.1f,%.1f)",
        nid, is_radar ? "radar" : "RF",
        fused_targets_[nid].fused_x,
        fused_targets_[nid].fused_y,
        fused_targets_[nid].fused_z);
    }
  }

  // ── 10 Hz publish callback ──────────────────────────────────────────────

  void publishFused()
  {
    const rclcpp::Time now = this->now();

    // Drop stale tracks
    for (auto it = fused_targets_.begin(); it != fused_targets_.end(); ) {
      double age = (now - it->second.last_seen).seconds();
      if (age > kDropAge) {
        RCLCPP_INFO(get_logger(), "Dropped stale target #%u", it->first);
        target_pubs_.erase(it->first);
        it = fused_targets_.erase(it);
      } else {
        ++it;
      }
    }

    // Build JSON and per-target messages
    std::ostringstream json;
    json << std::fixed << std::setprecision(3) << "[";
    bool first = true;

    for (auto & [id, fs] : fused_targets_) {
      auto ft = fs.toMsg(now);

      if (target_pubs_.count(id)) {
        target_pubs_.at(id)->publish(ft);
      }

      if (!first) { json << ","; }
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

  // ── Members ─────────────────────────────────────────────────────────────

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
  rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr radar_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::RadarTrack>::SharedPtr rf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       ack_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          wave_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<uint32_t, FusedState> fused_targets_;
  std::map<uint32_t,
    rclcpp::Publisher<acsdg_msgs::msg::FusedTarget>::SharedPtr> target_pubs_;

  uint32_t next_id_;
  std::set<uint32_t> killed_ids_;
};

}  // namespace acsdg_sensors

// ── Entry point ───────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors::SensorFusionNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
