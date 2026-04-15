#pragma once
//============================================================================
// rf360_node.hpp — Dedrone RF-360 direction-finding RF detector node.
//
// Real specs modelled:
//   Detection range : 2000 m (typical), 5000 m (ideal)
//   DF accuracy     : ±5° AoA
//   Update rate     : 1 Hz (classification scan)
//   Frequency bands : 433 MHz, 915 MHz, 2.4 GHz, 5.8 GHz
//
// Sim mode (use_real_hardware=false):
//   Subscribes to /rf360_{id}/rf_detections (std_msgs/String JSON)
//   bridged from RF360Plugin via ros_gz_bridge.
//   Publishes bearing-only tracks (range=0, azimuth=bearing).
//
// HW mode (use_real_hardware=true):
//   Polls RF-360 REST API: GET http://{host}:{port}/api/v1/detections
//   (Stub: logs polling address, publishes no tracks.)
//
// Publishes (both modes, identical topics):
//   /sensors/rf360_{id}/detections  acsdg_msgs/RadarTrack  @ 1 Hz
//   /sensors/rf360_{id}/status      std_msgs/String  JSON  @ 1 Hz
//============================================================================

#include "acsdg_sensors_hw/sensor_hw_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

namespace acsdg_sensors_hw {

// ── Single RF detection from plugin JSON ─────────────────────────────────
struct RfDetection {
  uint32_t    drone_id{0};
  double      bearing_deg{0};   // AoA bearing, degrees, 0=East
  double      rssi_dbm{0};      // negative value, e.g. -60
  double      freq_hz{0};       // 2.4e9, 5.8e9, etc.
  double      range_m{0};       // 0 in bearing-only mode
};

// ── RF360Node ─────────────────────────────────────────────────────────────

class RF360Node : public SensorHwInterface
{
public:
  RF360Node();

  void initialize()    override;
  void startStreaming() override;
  void stopStreaming()  override;

private:
  void onRfDetections(const std_msgs::msg::String::SharedPtr msg);
  void publishDetections();
  void publishStatus();

  static std::vector<RfDetection> parseJson(const std::string & json);

  // Confidence from RSSI: map [-100, -20] dBm → [0.0, 1.0]
  static float rssiToConfidence(double rssi_dbm);

  // Members
  rclcpp::Publisher<acsdg_msgs::msg::RadarTrack>::SharedPtr detect_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    gz_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Params
  double      df_accuracy_deg_{5.0};
  double      detection_range_{2000.0};
  std::string hw_host_{"192.168.1.200"};
  int         hw_port_{9090};

  // Latest detections (refreshed each incoming message)
  std::vector<RfDetection> latest_detections_;
  int detection_count_{0};
};

}  // namespace acsdg_sensors_hw
