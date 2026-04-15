#pragma once
//============================================================================
// boson_node.hpp — FLIR Boson+ 640 thermal camera node.
//
// Real specs modelled:
//   Resolution  : 640 × 512 pixels
//   FOV         : 50° × 40° (9 mm lens, wide-area surveillance)
//   Spectral    : 8–14 μm LWIR
//   NETD        : <20 mK  (σ=0.7 LSB gaussian noise added to synthetic frames)
//   Frame rate  : 60 Hz
//   Interface   : USB (UVC) / MIPI CSI-2
//
// Sim mode (use_real_hardware=false):
//   Subscribes to /model/enemy_{1..N}/odometry to get drone world positions.
//   Synthesises 640×512 L16 thermal frames:
//     Background: 293 K (20°C ambient)  →  L16 value 29300
//     Drone motor hot spot: 340 K       →  L16 value 34000
//     Gaussian NETD noise σ=0.7 LSB     →  matches <20 mK spec
//   Projects each drone from world frame into image frame using sensor pose.
//   Runs thermal blob detector: pixels > 310 K → detection.
//   (Avoids gz thermal_camera sensor which requires GPU/Ogre2 unavailable
//    under WSL2 software rendering.)
//
// HW mode (use_real_hardware=true):
//   Opens /dev/video{N} as V4L2 UVC device, captures 640×512 L16 frames.
//   (Stub: logs device path.)
//
// Publishes (both modes, identical topics):
//   /sensors/boson_{id}/thermal_image  sensor_msgs/Image       @ 60 Hz
//   /sensors/boson_{id}/camera_info    sensor_msgs/CameraInfo  @ 60 Hz
//   /sensors/boson_{id}/detections     acsdg_msgs/RadarTrack   @ 60 Hz
//   /sensors/boson_{id}/status         std_msgs/String JSON    @ 1 Hz
//============================================================================

#include "acsdg_sensors_hw/sensor_hw_interface.hpp"
#include <acsdg_msgs/msg/radar_track.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <map>
#include <mutex>
#include <random>
#include <vector>

namespace acsdg_sensors_hw {

// ── Detected thermal blob ─────────────────────────────────────────────────
struct ThermalBlob {
  double cx{0}, cy{0};
  double mean_temp_k{0};
  int    pixel_count{0};
  float  confidence{0};
};

// ── Enemy drone state cached from odometry ────────────────────────────────
struct DroneState {
  double x{0}, y{0}, z{0};
  bool   fresh{false};
};

// ── BosonNode ─────────────────────────────────────────────────────────────

class BosonNode : public SensorHwInterface
{
public:
  BosonNode();

  void initialize()    override;
  void startStreaming() override;
  void stopStreaming()  override;

private:
  // Simulation synthetic frame generation
  void publishSyntheticFrame();

  // Real image path (from gz bridge, or V4L2 in HW mode)
  void onThermalImage(const sensor_msgs::msg::Image::SharedPtr msg);

  // Odometry callback — populates drone_states_
  void onDroneOdom(int idx, const nav_msgs::msg::Odometry::SharedPtr msg);

  void publishStatus();

  // Render a 640×512 L16 frame with hot spots at projected drone positions
  sensor_msgs::msg::Image synthesiseFrame(const rclcpp::Time & stamp) const;

  // Project a world-frame point into image pixel coordinates.
  // Returns false if the point is behind or outside the FOV.
  bool worldToPixel(double wx, double wy, double wz,
                    double & px, double & py) const;

  std::vector<ThermalBlob> detectBlobs(const sensor_msgs::msg::Image & img) const;
  void pixelToBearing(double cx, double cy,
                      double & az_deg, double & el_deg) const;
  sensor_msgs::msg::CameraInfo buildCameraInfo(
    const std_msgs::msg::Header & hdr) const;

  // ── Publishers / subscribers ─────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<acsdg_msgs::msg::RadarTrack>::SharedPtr  detect_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        status_pub_;

  // Sim mode: synthetic frame timer + odometry subscriptions
  rclcpp::TimerBase::SharedPtr synth_timer_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;

  // Real-image path (gz bridge)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr gz_img_sub_;

  // HW mode poll timer
  rclcpp::TimerBase::SharedPtr hw_poll_timer_;

  rclcpp::TimerBase::SharedPtr status_timer_;

  // ── Sensor geometry ──────────────────────────────────────────────────────
  double fov_hz_deg_{50.0};
  double fov_vt_deg_{40.0};
  static constexpr int kWidth  = 640;
  static constexpr int kHeight = 512;

  // Sensor world position and orientation (yaw only — cameras pan toward base)
  double sensor_x_{0}, sensor_y_{0}, sensor_z_{13.0};
  double sensor_yaw_{0};   // radians, pointing inward

  // Focal lengths in pixels
  double fx_{0}, fy_{0};

  // L16 temperatures (value = K / 0.01)
  static constexpr uint16_t kBgTemp_L16     = 29300;  // 293 K ambient
  static constexpr uint16_t kDroneTemp_L16  = 34000;  // 340 K hot spot
  static constexpr uint16_t kHeatThreshold  = 31000;  // 310 K detect gate
  static constexpr int      kHotSpotRadius  = 6;      // pixels

  // NETD noise: σ=0.7 L16 counts ≈ 0.007 K → <20 mK spec
  static constexpr double kNoiseSigma = 0.7;

  // Number of enemy drones to track
  static constexpr int kMaxDrones = 4;

  // Drone state cache
  std::mutex              drone_mutex_;
  std::map<int,DroneState> drone_states_;

  // RNG for NETD noise
  mutable std::mt19937 rng_{std::random_device{}()};
  mutable std::normal_distribution<double> noise_dist_{0.0, kNoiseSigma};

  // V4L2 device index (HW mode)
  int v4l2_device_index_{0};

  // Stats
  int frame_count_{0};
  int detection_count_{0};
};

}  // namespace acsdg_sensors_hw
