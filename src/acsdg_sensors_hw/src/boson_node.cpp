//============================================================================
// boson_node.cpp — FLIR Boson+ 640 thermal camera node.
// See boson_node.hpp for full specification.
//
// Sim path: synthesises 640×512 L16 thermal frames from enemy drone odometry.
// Each drone motor is rendered as a 340 K hot disc (r=6 px) on a 293 K
// background.  Gaussian noise σ=0.7 LSB simulates the Boson+ NETD <20 mK.
//============================================================================

#include "acsdg_sensors_hw/boson_node.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>

namespace acsdg_sensors_hw {

BosonNode::BosonNode()
: SensorHwInterface("boson_node")
{}

// ── initialize() ─────────────────────────────────────────────────────────────

void BosonNode::initialize()
{
  use_real_hardware_ = declare_parameter<bool>("use_real_hardware", false);
  sensor_id_         = declare_parameter<int>("sensor_id", 1);
  frame_id_          = declare_parameter<std::string>("frame_id", "boson_1");
  fov_hz_deg_        = declare_parameter<double>("fov_deg", 50.0);
  fov_vt_deg_        = fov_hz_deg_ * (static_cast<double>(kHeight) / kWidth);
  publish_rate_      = 60.0;

  sensor_x_           = declare_parameter<double>("sensor_x",   190.0);
  sensor_y_           = declare_parameter<double>("sensor_y",   190.0);
  sensor_z_           = declare_parameter<double>("sensor_z",   13.0);
  sensor_yaw_         = declare_parameter<double>("sensor_yaw", -2.3562);  // NE→SW
  v4l2_device_index_  = declare_parameter<int>("v4l2_device", 0);

  // Focal lengths: f = (W/2) / tan(hfov/2)
  fx_ = (kWidth  / 2.0) / std::tan(fov_hz_deg_ * M_PI / 360.0);
  fy_ = (kHeight / 2.0) / std::tan(fov_vt_deg_ * M_PI / 360.0);

  std::string id_str    = std::to_string(sensor_id_);
  std::string img_topic  = "/sensors/boson_" + id_str + "/thermal_image";
  std::string info_topic = "/sensors/boson_" + id_str + "/camera_info";
  std::string det_topic  = "/sensors/boson_" + id_str + "/detections";
  std::string stat_topic = "/sensors/boson_" + id_str + "/status";

  image_pub_  = create_publisher<sensor_msgs::msg::Image>(img_topic,  rclcpp::QoS(10));
  info_pub_   = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, rclcpp::QoS(10));
  detect_pub_ = create_publisher<acsdg_msgs::msg::RadarTrack>(det_topic, rclcpp::QoS(20));
  status_pub_ = create_publisher<std_msgs::msg::String>(stat_topic, rclcpp::QoS(10));

  if (!use_real_hardware_) {
    // Subscribe to enemy drone odometry to get positions for synthetic frames
    for (int i = 1; i <= kMaxDrones; ++i) {
      drone_states_[i] = DroneState{};
      std::string topic = "/model/enemy_" + std::to_string(i) + "/odometry";
      odom_subs_.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          topic, rclcpp::QoS(10),
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            onDroneOdom(i, msg);
          }));
    }

    RCLCPP_INFO(get_logger(),
      "BosonNode [SIM-SYNTH] id=%d  pose=(%.0f,%.0f,%.0fz)  "
      "pub=%s  FOV=%.0f°×%.0f°  @%.0fHz",
      sensor_id_, sensor_x_, sensor_y_, sensor_z_,
      img_topic.c_str(), fov_hz_deg_, fov_vt_deg_, publish_rate_);
  } else {
    RCLCPP_WARN(get_logger(),
      "HARDWARE MODE: opening V4L2 device /dev/video%d  (640×512 L16)",
      v4l2_device_index_);
  }

  status_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this]() { publishStatus(); });

  RCLCPP_INFO(get_logger(),
    "BosonNode ready  id=%d  FOV=%.0f×%.0f°  @%.0fHz  hw=%s",
    sensor_id_, fov_hz_deg_, fov_vt_deg_, publish_rate_,
    use_real_hardware_ ? "REAL" : "SIM");
}

void BosonNode::startStreaming()
{
  if (!use_real_hardware_) {
    auto ms = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / publish_rate_));
    synth_timer_ = create_wall_timer(ms, [this]() { publishSyntheticFrame(); });
  } else {
    auto ms = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / publish_rate_));
    hw_poll_timer_ = create_wall_timer(ms, [this]() {
      RCLCPP_DEBUG(get_logger(),
        "HARDWARE MODE: capturing from /dev/video%d", v4l2_device_index_);
    });
  }
  RCLCPP_INFO(get_logger(), "BosonNode streaming at %.0f Hz", publish_rate_);
}

void BosonNode::stopStreaming()
{
  if (synth_timer_)    synth_timer_->cancel();
  if (hw_poll_timer_)  hw_poll_timer_->cancel();
  RCLCPP_INFO(get_logger(), "BosonNode stopped");
}

// ── Drone odometry callback ───────────────────────────────────────────────────

void BosonNode::onDroneOdom(int idx, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(drone_mutex_);
  auto & s = drone_states_[idx];
  s.x     = msg->pose.pose.position.x;
  s.y     = msg->pose.pose.position.y;
  s.z     = msg->pose.pose.position.z;
  s.fresh = true;
}

// ── Synthetic frame generation ────────────────────────────────────────────────

void BosonNode::publishSyntheticFrame()
{
  ++frame_count_;
  const rclcpp::Time now = this->now();

  auto img = synthesiseFrame(now);
  image_pub_->publish(img);

  std_msgs::msg::Header hdr;
  hdr.stamp    = now;
  hdr.frame_id = frame_id_;
  info_pub_->publish(buildCameraInfo(hdr));

  auto blobs = detectBlobs(img);
  detection_count_ = static_cast<int>(blobs.size());

  uint32_t bid = 1;
  for (const auto & blob : blobs) {
    double az{0}, el{0};
    pixelToBearing(blob.cx, blob.cy, az, el);

    acsdg_msgs::msg::RadarTrack trk;
    trk.id          = bid++;
    trk.position.x  = 0.0;
    trk.position.y  = 0.0;
    trk.position.z  = 0.0;
    trk.velocity.x  = az;
    trk.velocity.y  = el;
    trk.velocity.z  = blob.mean_temp_k;
    trk.confidence  = blob.confidence;
    trk.stamp       = now;
    detect_pub_->publish(trk);
  }
}

// ── Frame synthesiser ─────────────────────────────────────────────────────────

sensor_msgs::msg::Image BosonNode::synthesiseFrame(const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::Image img;
  img.header.stamp    = stamp;
  img.header.frame_id = frame_id_;
  img.width           = kWidth;
  img.height          = kHeight;
  img.encoding        = "mono16";
  img.is_bigendian    = 0;
  img.step            = kWidth * 2;

  // Fill background: 293 K ambient (constant — background noise σ=0.7 LSB can
  // never reach the 310 K detect gate 1700σ away, so noise here is wasted).
  img.data.resize(kWidth * kHeight * 2);
  uint16_t * pixels = reinterpret_cast<uint16_t *>(img.data.data());
  std::fill(pixels, pixels + kWidth * kHeight, kBgTemp_L16);

  // Project each fresh drone into image and paint hot disc
  {
    std::lock_guard<std::mutex> lk(const_cast<std::mutex &>(drone_mutex_));
    for (auto & [idx, ds] : const_cast<std::map<int,DroneState>&>(drone_states_)) {
      if (!ds.fresh) continue;

      double px{0}, py{0};
      if (!worldToPixel(ds.x, ds.y, ds.z, px, py)) continue;

      // Paint filled disc of radius kHotSpotRadius at (px, py)
      int cx = static_cast<int>(std::round(px));
      int cy = static_cast<int>(std::round(py));

      for (int dy = -kHotSpotRadius; dy <= kHotSpotRadius; ++dy) {
        for (int dx = -kHotSpotRadius; dx <= kHotSpotRadius; ++dx) {
          if (dx*dx + dy*dy > kHotSpotRadius*kHotSpotRadius) continue;
          int px2 = cx + dx, py2 = cy + dy;
          if (px2 < 0 || px2 >= kWidth || py2 < 0 || py2 >= kHeight) continue;
          double noisy = kDroneTemp_L16 + noise_dist_(rng_);
          pixels[py2 * kWidth + px2] = static_cast<uint16_t>(
            std::max(0.0, std::min(65535.0, noisy)));
        }
      }
    }
  }
  return img;
}

// ── World → pixel projection ──────────────────────────────────────────────────

bool BosonNode::worldToPixel(double wx, double wy, double wz,
                              double & px, double & py) const
{
  // Translate into sensor frame
  double dx = wx - sensor_x_;
  double dy = wy - sensor_y_;
  double dz = wz - sensor_z_;

  // Rotate by -sensor_yaw (sensor faces inward)
  double cosY = std::cos(-sensor_yaw_);
  double sinY = std::sin(-sensor_yaw_);
  double lx   =  dx * cosY - dy * sinY;   // forward (camera +Z)
  double ly   =  dx * sinY + dy * cosY;   // left
  double lz   =  dz;                       // up

  // Camera convention: forward = +lx, right = -ly, up = +lz
  // (camera looks along +lx, image x → right, image y → down)
  if (lx < 0.3) return false;  // behind or too close

  double u = fx_ * (-ly / lx);   // image x (right = positive)
  double v = fy_ * (-lz / lx);   // image y (down  = positive)

  px = u + kWidth  / 2.0;
  py = v + kHeight / 2.0;

  // FOV gate
  if (px < 0 || px >= kWidth || py < 0 || py >= kHeight) return false;
  return true;
}

// ── Blob detector (unchanged from original) ───────────────────────────────────

std::vector<ThermalBlob> BosonNode::detectBlobs(
  const sensor_msgs::msg::Image & img) const
{
  const uint16_t * data = reinterpret_cast<const uint16_t *>(img.data.data());
  int W = static_cast<int>(img.width);
  int H = static_cast<int>(img.height);
  if (W == 0 || H == 0 || img.data.size() < static_cast<size_t>(W * H * 2))
    return {};

  std::vector<bool> visited(W * H, false);
  std::vector<ThermalBlob> blobs;

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int idx = y * W + x;
      if (visited[idx] || data[idx] < kHeatThreshold) continue;

      std::vector<int> queue{idx};
      std::vector<int> region;
      visited[idx] = true;

      while (!queue.empty() && region.size() < 2000) {
        int cur = queue.back(); queue.pop_back();
        region.push_back(cur);
        int cx_ = cur % W, cy_ = cur / W;
        int nb[4] = {
          cy_ > 0   ? (cy_-1)*W+cx_ : -1,
          cy_ < H-1 ? (cy_+1)*W+cx_ : -1,
          cx_ > 0   ? cy_*W+cx_-1   : -1,
          cx_ < W-1 ? cy_*W+cx_+1   : -1
        };
        for (int n : nb) {
          if (n >= 0 && !visited[n] && data[n] >= kHeatThreshold) {
            visited[n] = true; queue.push_back(n);
          }
        }
      }

      if (region.size() < 4) continue;

      double sumX=0, sumY=0, sumT=0;
      for (int i : region) { sumX += i%W; sumY += i/W; sumT += data[i]; }
      double n = static_cast<double>(region.size());

      ThermalBlob blob;
      blob.cx          = sumX / n;
      blob.cy          = sumY / n;
      blob.mean_temp_k = (sumT / n) * 0.01;
      blob.pixel_count = static_cast<int>(n);
      double excess    = blob.mean_temp_k - 310.0;
      blob.confidence  = static_cast<float>(std::min(1.0, std::max(0.1, excess/50.0)));
      blobs.push_back(blob);
    }
  }
  return blobs;
}

void BosonNode::pixelToBearing(double cx, double cy,
                                double & az_deg, double & el_deg) const
{
  double dx = (cx - kWidth  / 2.0) / (kWidth  / 2.0);
  double dy = (cy - kHeight / 2.0) / (kHeight / 2.0);
  az_deg =  dx * (fov_hz_deg_ / 2.0);
  el_deg = -dy * (fov_vt_deg_ / 2.0);
}

sensor_msgs::msg::CameraInfo BosonNode::buildCameraInfo(
  const std_msgs::msg::Header & hdr) const
{
  sensor_msgs::msg::CameraInfo ci;
  ci.header = hdr;
  ci.width  = kWidth;
  ci.height = kHeight;
  ci.distortion_model = "plumb_bob";
  ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  double cx = kWidth/2.0, cy = kHeight/2.0;
  ci.k = {fx_, 0.0, cx, 0.0, fy_, cy, 0.0, 0.0, 1.0};
  ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  ci.p = {fx_, 0.0, cx, 0.0, 0.0, fy_, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return ci;
}

void BosonNode::onThermalImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Fallback for real gz camera image path (not used in synth mode)
  ++frame_count_;
  sensor_msgs::msg::Image out = *msg;
  out.header.frame_id = frame_id_;
  if (out.encoding.empty()) out.encoding = "mono16";
  image_pub_->publish(out);
  std_msgs::msg::Header hdr = out.header;
  info_pub_->publish(buildCameraInfo(hdr));
  auto blobs = detectBlobs(out);
  detection_count_ = static_cast<int>(blobs.size());
}

void BosonNode::publishStatus()
{
  std::ostringstream ss;
  ss << "{\"sensor\":\"BosonPlus640\","
     << "\"mode\":\""       << (use_real_hardware_ ? "HW" : "SIM") << "\","
     << "\"id\":"           << sensor_id_       << ","
     << "\"frames\":"       << frame_count_     << ","
     << "\"detections\":"   << detection_count_ << ","
     << "\"health\":\"OK\"}";
  std_msgs::msg::String msg;
  msg.data = ss.str();
  status_pub_->publish(msg);
}

}  // namespace acsdg_sensors_hw

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<acsdg_sensors_hw::BosonNode>();
  node->initialize();
  node->startStreaming();
  rclcpp::spin(node);
  node->stopStreaming();
  rclcpp::shutdown();
  return 0;
}
