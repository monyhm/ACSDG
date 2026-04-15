//============================================================================
// rf_node.cpp — Simulated RF-emission detector for the ACSDG system.
//
// Models a single omnidirectional RF receiver co-located at the origin.
// It detects drone control-link emissions out to 300 m but with a lower
// maximum confidence than radar (0.7) due to multipath and spoofing margin.
// Noise is independent of the radar node, simulating an uncorrelated sensor.
//
// Uses the same deterministic drone trajectories as radar_node so the
// fusion node can correctly associate tracks.
//
// Publishes: /sensors/rf/detections  (acsdg_msgs/RadarTrack)  @ 5 Hz
//============================================================================

#include "acsdg_sensors/sensor_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>

#include <chrono>
#include <cmath>
#include <random>
#include <array>

namespace acsdg_sensors {

// ── Simulation constants ──────────────────────────────────────────────────

static constexpr double kRfMaxRange    = 300.0; // omnidirectional, metres
static constexpr double kRfMaxConfidence = 0.7; // upper bound on confidence
static constexpr double kDroneAltitude = 50.0;
static constexpr double kNoiseStdDev   = 1.2;   // RF noisier than radar

// Same drone specs as radar_node — both nodes compute identical true positions
struct DroneSpec {
  uint32_t id;
  double   start_radius;
  double   angle_rad;
  double   speed;
};

static constexpr std::array<DroneSpec, 3> kDrones{{
  {1u, 300.0, M_PI / 4.0,          8.0},
  {2u, 250.0, 3.0 * M_PI / 4.0,   12.0},
  {3u, 350.0, 3.0 * M_PI / 2.0,   10.0},
}};

// ── Helper: drone kinematics (identical formula to radar_node) ────────────

static geometry_msgs::msg::Point dronePosition(const DroneSpec & d, double t)
{
  double cycle  = d.start_radius / d.speed;
  double radius = d.start_radius - d.speed * std::fmod(t, cycle);
  geometry_msgs::msg::Point p;
  p.x = radius * std::cos(d.angle_rad);
  p.y = radius * std::sin(d.angle_rad);
  p.z = kDroneAltitude;
  return p;
}

static geometry_msgs::msg::Vector3 droneVelocity(const DroneSpec & d)
{
  geometry_msgs::msg::Vector3 v;
  v.x = -d.speed * std::cos(d.angle_rad);
  v.y = -d.speed * std::sin(d.angle_rad);
  v.z = 0.0;
  return v;
}

// ── RfNode ────────────────────────────────────────────────────────────────

class RfNode : public SensorInterface
{
public:
  RfNode()
  : SensorInterface("rf_node"),
    rng_(std::random_device{}()),
    noise_(0.0, kNoiseStdDev)
  {}

  void initialize() override
  {
    publish_rate_ = 5.0;
    frame_id_     = "world";

    pub_ = create_publisher<acsdg_msgs::msg::RadarTrack>(
      "/sensors/rf/detections", rclcpp::QoS(50));

    RCLCPP_INFO(
      get_logger(),
      "RfNode: omnidirectional receiver, range=%.0fm, max-conf=%.1f, noise σ=%.1fm",
      kRfMaxRange, kRfMaxConfidence, kNoiseStdDev);
  }

  void startStreaming() override
  {
    start_time_ = this->now();
    auto ms = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(ms, [this]() { publishDetections(); });
    RCLCPP_INFO(get_logger(), "RfNode streaming at %.0f Hz", publish_rate_);
  }

  void stopStreaming() override
  {
    if (timer_) { timer_->cancel(); }
    RCLCPP_INFO(get_logger(), "RfNode stopped");
  }

private:
  // ── Called at 5 Hz ─────────────────────────────────────────────────────
  void publishDetections()
  {
    const double t = (this->now() - start_time_).seconds();

    for (const auto & drone : kDrones) {
      auto true_pos = dronePosition(drone, t);
      auto vel      = droneVelocity(drone);

      // Distance from origin (RF receiver location)
      double dist = std::sqrt(
        true_pos.x * true_pos.x +
        true_pos.y * true_pos.y +
        true_pos.z * true_pos.z);

      if (dist > kRfMaxRange) {
        continue;  // below detectable signal strength
      }

      // Confidence: linear falloff scaled to kRfMaxConfidence
      float confidence = static_cast<float>(
        kRfMaxConfidence * std::max(0.0, 1.0 - dist / kRfMaxRange));

      acsdg_msgs::msg::RadarTrack msg;
      msg.id         = drone.id;
      msg.position.x = true_pos.x + noise_(rng_);
      msg.position.y = true_pos.y + noise_(rng_);
      msg.position.z = true_pos.z + noise_(rng_);
      msg.velocity   = vel;
      msg.confidence = confidence;
      msg.stamp      = this->now();

      pub_->publish(msg);
    }
  }

  rclcpp::Publisher<acsdg_msgs::msg::RadarTrack>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
};

}  // namespace acsdg_sensors

// ── Entry point ───────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors::RfNode>();
  node->initialize();
  node->startStreaming();
  rclcpp::spin(node);
  node->stopStreaming();
  rclcpp::shutdown();
  return 0;
}
