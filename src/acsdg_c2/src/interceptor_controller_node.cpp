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
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <acsdg_msgs/msg/engagement_order.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

// ── InterceptorControllerNode ─────────────────────────────────────────────

class InterceptorControllerNode : public rclcpp::Node
{
  // Maximum commanded speed (m/s)
  static constexpr double kMaxSpeed    = 15.0;
  // Neutralisation radius (m) — "target hit" when within this range
  static constexpr double kKillRadius  = 8.0;
  // Ticks without target update before "target lost"
  static constexpr int    kLostTicks   = 40;   // 40 × 50 ms = 2 s
  // Control loop period
  static constexpr double kDt          = 0.05; // seconds (20 Hz)
  // Cruise altitude — matches enemy_driver_node's TARGET_ALTITUDE so
  // pursuit collapses to a 2D problem and the controller doesn't have
  // to chase noisy enemy z. Altitude is held by an outer P loop, not
  // by the pursuit term.
  static constexpr double kCruiseZ     = 50.0;
  static constexpr double kAltKp       = 0.5;
  static constexpr double kMaxVz       = 3.0;

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

    // Engagement-outcome acknowledgement (consumed by mission_manager + c2_engine)
    ack_pub_ = create_publisher<std_msgs::msg::String>("/mission/engagement_ack", 10);

    // Gazebo velocity command — picked up by gz_bridge_shim and forwarded
    // to /model/interceptor_{id}/cmd_vel on the Gazebo transport side.
    std::string cmd_topic = "/interceptor_" + std::to_string(id_) + "/cmd_vel";
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    // ── Subscriptions ─────────────────────────────────────────────────────
    order_sub_ = create_subscription<acsdg_msgs::msg::EngagementOrder>(
      "/c2/engagement_orders", 10,
      [this](acsdg_msgs::msg::EngagementOrder::SharedPtr msg) {
        onOrder(msg);
      });

    // Self-odometry feedback. Without this the controller dead-reckons from
    // home via integration of its own commanded velocity, and the kill-
    // radius check compares two fictional positions (internal pos_ vs fused
    // target). In practice this caused NEUTRALIZED acks with the real
    // Gazebo body nowhere near the enemy. Overwriting pos_ from odom on
    // every message makes the kill check a real physical-proximity test.
    std::string odom_topic = "/model/interceptor_" + std::to_string(id_) + "/odometry";
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        pos_x_ = m->pose.pose.position.x;
        pos_y_ = m->pose.pose.position.y;
        pos_z_ = m->pose.pose.position.z;
        has_odom_ = true;
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

    target_id_        = static_cast<int>(msg->target_id);
    pursuing_         = true;
    lost_ticks_       = 0;
    // Cached target position is stale (may equal our own position if this
    // controller just killed the previous target). Block pursuit until the
    // first onTarget() from the new subscription lands, otherwise the kill-
    // radius check fires instantly and we spin in a neutralise loop.
    has_target_data_  = false;

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
    lost_ticks_      = 0;  // refresh watchdog
    has_target_data_ = true;
  }

  // ── 20 Hz control step ────────────────────────────────────────────────

  void controlStep()
  {
    // Hold until we know where we actually are — pos_ defaults to home,
    // which is not where the Gazebo body spawns, so commanding velocity
    // against that stale belief drifts the body off course.
    if (!has_odom_) {
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }

    if (pursuing_) {
      ++lost_ticks_;
      if (lost_ticks_ > kLostTicks) {
        RCLCPP_INFO(get_logger(),
          "Interceptor #%d: target #%d lost — returning home", id_, target_id_);
        publishAck(target_id_, "LOST");
        pursuing_        = false;
        has_target_data_ = false;
        target_sub_.reset();
        returnHome();
      } else if (has_target_data_) {
        pursueTarget();
      } else {
        // Assigned but no FusedTarget received yet — hold position.
        publishSetpoint(pos_x_, pos_y_, pos_z_);
        publishCmdVel(0.0, 0.0, 0.0);
      }
    } else {
      stepTowardHome();
      publishSetpoint(pos_x_, pos_y_, pos_z_);
    }

    publishPosition();
  }

  // ── Lead-pursuit guidance ────────────────────────────────────────────
  // Aim at where the target will be after t_go = range / max_speed seconds.
  // Simpler and better-behaved than the open-loop PN that preceded this:
  // without interceptor-velocity feedback the LOS-rate term was degenerate,
  // collapsing the guidance law to pure pursuit (tail-chase geometry).

  void pursueTarget()
  {
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double rz = tgt_z_ - pos_z_;
    const double range = std::sqrt(rx*rx + ry*ry + rz*rz);

    if (range < kKillRadius) {
      RCLCPP_INFO(get_logger(),
        "Interceptor #%d: NEUTRALISED target #%d at range=%.2fm "
        "int(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
        id_, target_id_, range,
        pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
      publishAck(target_id_, "NEUTRALIZED");
      pursuing_        = false;
      has_target_data_ = false;
      target_sub_.reset();
      publishSetpoint(pos_x_, pos_y_, pos_z_);
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }

    // 2D lead pursuit — predict target xy after t_go seconds, fly toward
    // that lead point at max speed. Altitude is handled separately by the
    // outer altitude-hold loop below.
    const double range_xy = std::sqrt(rx*rx + ry*ry);
    const double t_go     = range_xy / kMaxSpeed;
    const double lead_x   = tgt_x_ + tgt_vx_ * t_go;
    const double lead_y   = tgt_y_ + tgt_vy_ * t_go;

    const double dx   = lead_x - pos_x_;
    const double dy   = lead_y - pos_y_;
    const double dmag = std::sqrt(dx*dx + dy*dy);
    const double s    = (dmag > 1e-6) ? (kMaxSpeed / dmag) : 0.0;

    const double vx = dx * s;
    const double vy = dy * s;
    // Hold cruise altitude — clamped for safety even though kAltKp keeps
    // the loop tame.
    const double vz = std::clamp(kAltKp * (kCruiseZ - pos_z_),
                                 -kMaxVz, kMaxVz);

    publishSetpoint(pos_x_, pos_y_, pos_z_);
    publishCmdVel(vx, vy, vz);
  }

  void stepTowardHome()
  {
    const double dx = home_x_ - pos_x_;
    const double dy = home_y_ - pos_y_;
    const double dz = home_z_ - pos_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist < 0.5) {
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }
    const double kReturnSpeed = 5.0;  // m/s
    const double vx = (dx / dist) * kReturnSpeed;
    const double vy = (dy / dist) * kReturnSpeed;
    const double vz = (dz / dist) * kReturnSpeed;
    publishCmdVel(vx, vy, vz);
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

  void publishCmdVel(double vx, double vy, double vz)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.linear.z = vz;
    cmd_vel_pub_->publish(t);
  }

  void publishAck(int target_id, const std::string & outcome)
  {
    std::ostringstream oss;
    oss << "{\"target_id\":" << target_id
        << ",\"interceptor_id\":" << id_
        << ",\"outcome\":\"" << outcome << "\"}";
    std_msgs::msg::String msg;
    msg.data = oss.str();
    ack_pub_->publish(msg);
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
  bool   has_target_data_{false};

  bool   has_odom_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr       position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           ack_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub_;
  rclcpp::Subscription<acsdg_msgs::msg::EngagementOrder>::SharedPtr order_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::FusedTarget>::SharedPtr     target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
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
