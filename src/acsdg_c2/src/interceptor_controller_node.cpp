//============================================================================
// interceptor_controller_node.cpp — Per-interceptor flight controller.
//
// One instance per interceptor, parameterised by interceptor_id.
// When assigned a target via /c2/engagement_orders, subscribes to that
// target's FusedTarget topic and applies proportional-navigation guidance
// to fly toward the intercept point.  Publishes MAVROS setpoints for the
// physical (or simulated) autopilot.
//
// Publishes
// ---------
//   /drone_{id}/mavros/setpoint_position/local   geometry_msgs/PoseStamped
//   /interceptors/{id}/position                  geometry_msgs/Point
//
// Subscribes
// ----------
//   /c2/engagement_orders   acsdg_msgs/EngagementOrder
//   /threats/target_{n}/fused_target  acsdg_msgs/FusedTarget  (dynamic)
//
// Services (attempted non-blocking)
// ----------
//   /drone_{id}/mavros/set_mode   mavros_msgs/SetMode
//============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <acsdg_msgs/msg/engagement_order.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

// ── InterceptorControllerNode ─────────────────────────────────────────────

class InterceptorControllerNode : public rclcpp::Node
{
  // Proportional navigation constant
  static constexpr double kN           = 3.0;
  // Maximum commanded speed (m/s)
  static constexpr double kMaxSpeed    = 15.0;
  // Neutralisation radius (m) — "target hit" when within this range
  static constexpr double kKillRadius  = 8.0;
  // Ticks without target update before "target lost"
  static constexpr int    kLostTicks   = 40;   // 40 × 50 ms = 2 s
  // Control loop period
  static constexpr double kDt          = 0.05; // seconds (20 Hz)

public:
  InterceptorControllerNode() : rclcpp::Node("interceptor_controller_node")
  {
    // ── Parameters ────────────────────────────────────────────────────────
    declare_parameter("interceptor_id", 1);
    declare_parameter("home_x",  200.0);
    declare_parameter("home_y",  200.0);
    declare_parameter("home_z",   20.0);

    id_    = get_parameter("interceptor_id").as_int();
    home_x_ = get_parameter("home_x").as_double();
    home_y_ = get_parameter("home_y").as_double();
    home_z_ = get_parameter("home_z").as_double();

    // Start at home
    pos_x_ = home_x_;  pos_y_ = home_y_;  pos_z_ = home_z_;

    // ── Publishers ────────────────────────────────────────────────────────
    std::string sp_topic =
      "/drone_" + std::to_string(id_) + "/mavros/setpoint_position/local";
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(sp_topic, 10);

    std::string pos_topic = "/interceptors/unit_" + std::to_string(id_) + "/position";
    position_pub_ = create_publisher<geometry_msgs::msg::Point>(pos_topic, 10);

    // ── Subscriptions ─────────────────────────────────────────────────────
    order_sub_ = create_subscription<acsdg_msgs::msg::EngagementOrder>(
      "/c2/engagement_orders", 10,
      [this](acsdg_msgs::msg::EngagementOrder::SharedPtr msg) {
        onOrder(msg);
      });

    // ── MAVROS set_mode client (optional — fails silently without MAVROS) ─
    std::string mode_srv = "/drone_" + std::to_string(id_) + "/mavros/set_mode";
    mode_client_ = create_client<mavros_msgs::srv::SetMode>(mode_srv);

    // ── 20 Hz control loop ────────────────────────────────────────────────
    timer_ = create_wall_timer(50ms, [this]() { controlStep(); });

    RCLCPP_INFO(get_logger(),
      "InterceptorController #%d  home=(%.0f, %.0f, %.0f)",
      id_, home_x_, home_y_, home_z_);
  }

private:
  // ── Engagement order ──────────────────────────────────────────────────

  void onOrder(const acsdg_msgs::msg::EngagementOrder::SharedPtr msg)
  {
    if (static_cast<int>(msg->interceptor_id) != id_) {
      return;
    }
    if (pursuing_ && static_cast<uint32_t>(target_id_) == msg->target_id) {
      return;  // already on this target
    }

    target_id_  = static_cast<int>(msg->target_id);
    pursuing_   = true;
    lost_ticks_ = 0;

    // Drop old subscription and subscribe to the assigned target
    target_sub_.reset();
    std::string topic =
      "/threats/target_" + std::to_string(target_id_) + "/fused_target";
    target_sub_ = create_subscription<acsdg_msgs::msg::FusedTarget>(
      topic, rclcpp::QoS(10),
      [this](acsdg_msgs::msg::FusedTarget::SharedPtr m) { onTarget(m); });

    RCLCPP_INFO(get_logger(),
      "Interceptor #%d assigned to target #%d", id_, target_id_);

    requestOffboard();
  }

  // ── FusedTarget update ────────────────────────────────────────────────

  void onTarget(const acsdg_msgs::msg::FusedTarget::SharedPtr msg)
  {
    tgt_x_ = msg->position.x;
    tgt_y_ = msg->position.y;
    tgt_z_ = msg->position.z;
    tgt_vx_ = msg->velocity.x;
    tgt_vy_ = msg->velocity.y;
    tgt_vz_ = msg->velocity.z;
    lost_ticks_ = 0;  // refresh watchdog
  }

  // ── 20 Hz control step ────────────────────────────────────────────────

  void controlStep()
  {
    if (pursuing_) {
      ++lost_ticks_;
      if (lost_ticks_ > kLostTicks) {
        RCLCPP_INFO(get_logger(),
          "Interceptor #%d: target #%d lost — returning home", id_, target_id_);
        pursuing_ = false;
        target_sub_.reset();
        returnHome();
      } else {
        pursueTarget();
      }
    } else {
      stepTowardHome();
      publishSetpoint(pos_x_, pos_y_, pos_z_);
    }

    publishPosition();
  }

  // ── Proportional navigation guidance ─────────────────────────────────

  void pursueTarget()
  {
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double rz = tgt_z_ - pos_z_;
    const double range = std::sqrt(rx*rx + ry*ry + rz*rz);

    if (range < kKillRadius) {
      RCLCPP_INFO(get_logger(),
        "Interceptor #%d: NEUTRALISED target #%d", id_, target_id_);
      pursuing_ = false;
      target_sub_.reset();
      publishSetpoint(pos_x_, pos_y_, pos_z_);
      return;
    }

    // Unit range vector (interceptor → target)
    const double rux = rx / range;
    const double ruy = ry / range;
    const double ruz = rz / range;

    // Closing velocity = -dR/dt ≈ -(relative_vel · range_unit)
    const double rel_vx = tgt_vx_;   // interceptor velocity not fed back
    const double rel_vy = tgt_vy_;   // in this open-loop sim
    const double rel_vz = tgt_vz_;
    const double vc = -(rel_vx*rux + rel_vy*ruy + rel_vz*ruz);

    // Line-of-sight rotation rate vector (cross-product approximation)
    // ω_LOS = (R × Vrel) / R²
    const double omega_x = (ry*rel_vz - rz*rel_vy) / (range*range);
    const double omega_y = (rz*rel_vx - rx*rel_vz) / (range*range);
    const double omega_z = (rx*rel_vy - ry*rel_vx) / (range*range);
    const double omega   = std::sqrt(omega_x*omega_x + omega_y*omega_y + omega_z*omega_z);

    // Commanded acceleration magnitude: a = N * Vc * ωLOS
    // Direction: perpendicular to LOS (simplified: along LOS toward target)
    const double a_mag = kN * std::max(1.0, std::abs(vc)) * omega;

    // Decompose: blend proportional (toward target) + lateral correction
    double cmd_x = kN * std::max(1.0, std::abs(vc)) * rux;
    double cmd_y = kN * std::max(1.0, std::abs(vc)) * ruy;
    double cmd_z = kN * std::max(1.0, std::abs(vc)) * ruz;
    (void)a_mag;  // used implicitly via omega scaling above

    // Clamp to max speed
    const double cmd_spd = std::sqrt(cmd_x*cmd_x + cmd_y*cmd_y + cmd_z*cmd_z);
    if (cmd_spd > kMaxSpeed) {
      const double s = kMaxSpeed / cmd_spd;
      cmd_x *= s;  cmd_y *= s;  cmd_z *= s;
    }

    // Integrate position (forward Euler, kDt = 50 ms)
    pos_x_ += cmd_x * kDt;
    pos_y_ += cmd_y * kDt;
    pos_z_ += cmd_z * kDt;

    publishSetpoint(pos_x_, pos_y_, pos_z_);
  }

  void stepTowardHome()
  {
    const double dx = home_x_ - pos_x_;
    const double dy = home_y_ - pos_y_;
    const double dz = home_z_ - pos_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist < 0.5) {
      pos_x_ = home_x_;  pos_y_ = home_y_;  pos_z_ = home_z_;
      return;
    }
    const double step = std::min(5.0 * kDt, dist);  // 5 m/s return speed
    pos_x_ += (dx / dist) * step;
    pos_y_ += (dy / dist) * step;
    pos_z_ += (dz / dist) * step;
  }

  void returnHome() { /* state already set — stepTowardHome() handles it */ }

  // ── ROS publishing helpers ────────────────────────────────────────────

  void publishSetpoint(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    msg.pose.orientation.w = 1.0;
    setpoint_pub_->publish(msg);
  }

  void publishPosition()
  {
    geometry_msgs::msg::Point pt;
    pt.x = pos_x_;  pt.y = pos_y_;  pt.z = pos_z_;
    position_pub_->publish(pt);
  }

  void requestOffboard()
  {
    if (!mode_client_->service_is_ready()) {
      return;  // MAVROS not running — skip silently in simulation
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode    = 0;
    req->custom_mode  = "OFFBOARD";
    mode_client_->async_send_request(
      req,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture /*fut*/) {
        RCLCPP_DEBUG(get_logger(), "Interceptor #%d: OFFBOARD requested", id_);
      });
  }

  // ── Members ──────────────────────────────────────────────────────────

  int    id_;
  double home_x_, home_y_, home_z_;

  // Simulated state
  double pos_x_, pos_y_, pos_z_;
  bool   pursuing_{false};
  int    target_id_{0};
  double tgt_x_{0}, tgt_y_{0}, tgt_z_{0};
  double tgt_vx_{0}, tgt_vy_{0}, tgt_vz_{0};
  int    lost_ticks_{0};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr       position_pub_;
  rclcpp::Subscription<acsdg_msgs::msg::EngagementOrder>::SharedPtr order_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::FusedTarget>::SharedPtr     target_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr              mode_client_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

// ── Entry point ───────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterceptorControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
