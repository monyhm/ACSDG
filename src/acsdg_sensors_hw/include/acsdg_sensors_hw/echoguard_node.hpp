#pragma once
//============================================================================
// echoguard_node.hpp — Echodyne EchoGuard phased-array radar node.
//
// Real specs modelled:
//   Range      : 1000 m (0.01 m² RCS drone)
//   FOV        : 120° az × 80° el  (electronic scanning)
//   Update rate: 25 Hz
//   Range σ    : 0.5 m   Az σ: 0.3°   Velocity σ: 0.05 m/s
//   P(detect)  : 1.0 inside 200 m, linear falloff to 0 at 1000 m
//
// Sim mode (use_real_hardware=false):
//   Subscribes to /echoguard_{id}/radar_returns (std_msgs/String JSON)
//   bridged from the EchoGuardRadarPlugin via ros_gz_bridge.
//
// HW mode (use_real_hardware=true):
//   Connects to EchoGuard Gigabit Ethernet API, parses JSON track output.
//   (Stub: logs connection attempt, publishes no tracks.)
//
// Publishes (both modes, identical topics):
//   /sensors/echoguard_{id}/tracks  acsdg_msgs/RadarTrack  @ 25 Hz
//   /sensors/echoguard_{id}/status  std_msgs/String  JSON  @ 1 Hz
//============================================================================

#include "acsdg_sensors_hw/sensor_hw_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>
#include <std_msgs/msg/string.hpp>

#include <map>
#include <random>
#include <vector>

namespace acsdg_sensors_hw {

// ── Per-return raw data from the plugin JSON ──────────────────────────────
struct RadarReturn {
  double range{0};
  double azimuth_deg{0};
  double elevation_deg{0};
  double doppler_mps{0};   // positive = moving away
};

// ── Track state for nearest-neighbour association ─────────────────────────
struct RadarTrackState {
  uint32_t id{0};
  double   x{0}, y{0}, z{0};   // Cartesian, world frame
  double   vx{0}, vy{0}, vz{0};
  float    confidence{0};
  rclcpp::Time last_seen;
};

// ── EchoGuardNode ─────────────────────────────────────────────────────────

class EchoGuardNode : public SensorHwInterface
{
public:
  EchoGuardNode();

  void initialize()    override;
  void startStreaming() override;
  void stopStreaming()  override;

private:
  // Callbacks
  void onRadarReturns(const std_msgs::msg::String::SharedPtr msg);
  void publishStatus();

  // Track association (nearest-neighbour, 5 m gate)
  void associateAndPublish(const std::vector<RadarReturn> & returns,
                           const rclcpp::Time & now);

  // Helpers
  static std::vector<RadarReturn> parseJson(const std::string & json);

  // Members
  rclcpp::Publisher<acsdg_msgs::msg::RadarTrack>::SharedPtr track_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    gz_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr hw_poll_timer_;   // used in HW mode

  // Sensor mounting pose (populated from params)
  double sensor_x_{0}, sensor_y_{0}, sensor_z_{1.5};
  double sensor_yaw_{0};   // heading the EchoGuard faces (radians)

  // Track management
  std::map<uint32_t, RadarTrackState> tracks_;
  uint32_t next_track_id_{1};

  // HW mode connection params
  std::string hw_host_{"192.168.1.100"};
  int         hw_port_{5555};

  // Stats for status message
  int active_track_count_{0};
};

}  // namespace acsdg_sensors_hw
