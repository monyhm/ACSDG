//============================================================================
// radar_node.cpp — Simulated phased-array radar for the ACSDG system.
//
// Models 6 radar sensors placed in a hexagon (r=150m) around the defended
// origin.  Each sensor has a 200 m detection radius.  Three simulated drones
// approach from different directions; their positions are computed from
// elapsed time so the simulation is deterministic across nodes.
//
// Parameter: use_gazebo_truth (bool, default false)
//   When true:  subscribes to /gz/drone_{i}/odometry (nav_msgs/Odometry)
//               published by ros_gz_bridge and converts those ground-truth
//               positions into RadarTrack messages with added Gaussian noise.
//               This is the hardware-abstraction path: the same radar_node
//               runs against real Gazebo physics without modifying the fusion
//               layer or downstream C2 logic.
//   When false: runs the built-in deterministic drone simulation (default).
//
// Publishes: /sensors/radar/raw_tracks  (acsdg_msgs/RadarTrack)  @ 10 Hz
// Subscribes (use_gazebo_truth=true):
//   /gz/drone_{1..8}/odometry  nav_msgs/Odometry
//============================================================================

#include "acsdg_sensors/sensor_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <array>
#include <map>

namespace acsdg_sensors {

// ── Simulation constants ──────────────────────────────────────────────────

static constexpr double kRadarMaxRange  = 200.0; // metres from sensor centre
static constexpr double kSensorRadius   = 150.0; // hexagon radius, metres
static constexpr int    kNumSensors     = 6;
static constexpr double kDroneAltitude  = 50.0;  // metres AGL (sim mode)
static constexpr double kNoiseStdDev    = 0.5;   // Gaussian position noise, m
static constexpr double kVelNoiseStdDev = 0.1;   // Gaussian velocity noise, m/s

// Number of enemy drones in Gazebo (must match world SDF include count)
static constexpr int kMaxGazeboDrones   = 4;

// ── Simulated drone trajectories ─────────────────────────────────────────
struct DroneSpec {
  uint32_t id;
  double   start_radius;
  double   angle_rad;
  double   speed;
};

static constexpr std::array<DroneSpec, 3> kDrones{{
  {1u, 300.0, M_PI / 4.0,          8.0},   // NE approach
  {2u, 250.0, 3.0 * M_PI / 4.0,   12.0},   // NW approach
  {3u, 350.0, 3.0 * M_PI / 2.0,   10.0},   // S  approach
}};

// ── Helpers ───────────────────────────────────────────────────────────────

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

// ── Ground-truth state from Gazebo odometry ───────────────────────────────

struct GazeboState {
  double x{0}, y{0}, z{0};
  double vx{0}, vy{0}, vz{0};
  bool   fresh{false};
};

// ── RadarNode ─────────────────────────────────────────────────────────────

class RadarNode : public SensorInterface
{
public:
  RadarNode()
  : SensorInterface("radar_node"),
    rng_(std::random_device{}()),
    noise_(0.0, kNoiseStdDev),
    vel_noise_(0.0, kVelNoiseStdDev)
  {
    for (int i = 0; i < kNumSensors; ++i) {
      double a = i * (2.0 * M_PI / kNumSensors);
      sensor_x_[i] = kSensorRadius * std::cos(a);
      sensor_y_[i] = kSensorRadius * std::sin(a);
    }
  }

  void initialize() override
  {
    publish_rate_    = 10.0;
    frame_id_        = "world";
    use_gz_truth_    = declare_parameter<bool>("use_gazebo_truth", false);

    pub_ = create_publisher<acsdg_msgs::msg::RadarTrack>(
      "/sensors/radar/raw_tracks", rclcpp::QoS(50));

    if (use_gz_truth_) {
      // Subscribe to Gazebo enemy drone odometry (ros_gz_bridge passes these
      // through from /model/enemy_{i}/odometry in the Gazebo world)
      for (int i = 1; i <= kMaxGazeboDrones; ++i) {
        gz_states_[i] = GazeboState{};
        std::string topic = "/model/enemy_" + std::to_string(i) + "/odometry";
        gz_subs_.push_back(
          create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::QoS(10),
            [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
              onGazeboOdom(i, msg);
            }));
      }
      RCLCPP_INFO(get_logger(),
        "RadarNode [GAZEBO-TRUTH]: subscribed to /model/enemy_{1..%d}/odometry "
        "pos_σ=%.1fm vel_σ=%.2fm/s", kMaxGazeboDrones, kNoiseStdDev, kVelNoiseStdDev);
    } else {
      RCLCPP_INFO(get_logger(),
        "RadarNode [SIM]: %d sensors r=%.0fm range=%.0fm σ=%.1fm",
        kNumSensors, kSensorRadius, kRadarMaxRange, kNoiseStdDev);
    }
  }

  void startStreaming() override
  {
    start_time_ = this->now();
    auto ms = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(ms, [this]() { publishTracks(); });
    RCLCPP_INFO(get_logger(), "RadarNode streaming at %.0f Hz", publish_rate_);
  }

  void stopStreaming() override
  {
    if (timer_) { timer_->cancel(); }
    RCLCPP_INFO(get_logger(), "RadarNode stopped");
  }

private:
  // ── Gazebo odometry callback ───────────────────────────────────────────

  void onGazeboOdom(int idx, const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(gz_mutex_);
    auto & st = gz_states_[idx];
    st.x  = msg->pose.pose.position.x;
    st.y  = msg->pose.pose.position.y;
    st.z  = msg->pose.pose.position.z;
    st.vx = msg->twist.twist.linear.x;
    st.vy = msg->twist.twist.linear.y;
    st.vz = msg->twist.twist.linear.z;
    st.fresh = true;
  }

  // ── Sensor range check ────────────────────────────────────────────────

  double minSensorDist(double px, double py) const
  {
    double min_d = std::numeric_limits<double>::max();
    for (int s = 0; s < kNumSensors; ++s) {
      double dx = px - sensor_x_[s];
      double dy = py - sensor_y_[s];
      min_d = std::min(min_d, std::sqrt(dx*dx + dy*dy));
    }
    return min_d;
  }

  // ── 10 Hz publish ─────────────────────────────────────────────────────

  void publishTracks()
  {
    if (use_gz_truth_) {
      publishFromGazebo();
    } else {
      publishFromSim();
    }
  }

  // ── Mode A: Gazebo ground-truth + noise ───────────────────────────────

  void publishFromGazebo()
  {
    std::lock_guard<std::mutex> lk(gz_mutex_);
    for (auto & [idx, st] : gz_states_) {
      if (!st.fresh) continue;

      double range = minSensorDist(st.x, st.y);
      if (range > kRadarMaxRange) continue;

      float confidence = static_cast<float>(
        std::max(0.0, 1.0 - range / kRadarMaxRange));

      acsdg_msgs::msg::RadarTrack msg;
      msg.id         = static_cast<uint32_t>(idx);
      msg.position.x = st.x  + noise_(rng_);
      msg.position.y = st.y  + noise_(rng_);
      msg.position.z = st.z  + noise_(rng_) * 0.5;  // less Z noise
      msg.velocity.x = st.vx + vel_noise_(rng_);
      msg.velocity.y = st.vy + vel_noise_(rng_);
      msg.velocity.z = st.vz + vel_noise_(rng_) * 0.5;
      msg.confidence = confidence;
      msg.stamp      = this->now();
      pub_->publish(msg);
    }
  }

  // ── Mode B: deterministic simulation ──────────────────────────────────

  void publishFromSim()
  {
    const double t = (this->now() - start_time_).seconds();

    for (const auto & drone : kDrones) {
      auto true_pos = dronePosition(drone, t);
      auto vel      = droneVelocity(drone);

      double range = minSensorDist(true_pos.x, true_pos.y);
      if (range > kRadarMaxRange) continue;

      float confidence = static_cast<float>(
        std::max(0.0, 1.0 - range / kRadarMaxRange));

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

  // ── Members ──────────────────────────────────────────────────────────

  rclcpp::Publisher<acsdg_msgs::msg::RadarTrack>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  bool use_gz_truth_{false};

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::normal_distribution<double> vel_noise_;

  double sensor_x_[kNumSensors]{};
  double sensor_y_[kNumSensors]{};

  // Gazebo-truth mode
  std::mutex gz_mutex_;
  std::map<int, GazeboState> gz_states_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> gz_subs_;
};

}  // namespace acsdg_sensors

// ── Entry point ───────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors::RadarNode>();
  node->initialize();
  node->startStreaming();
  rclcpp::spin(node);
  node->stopStreaming();
  rclcpp::shutdown();
  return 0;
}
