//============================================================================
// coyote_controller_node.cpp — Per-Coyote flight controller.
//
// Phase-2 controller for the Raytheon Coyote Block 2 frag-warhead jet
// interceptor. Modelled on interceptor_controller_node.cpp but with
// Coyote-specific constants (max speed 160 m/s vs Anvil 15 m/s, frag-fuze
// kill radius 5 m vs Anvil 8 m collision, no fixed cruise altitude).
//
// Publishes
// ---------
//   /coyote_{id}/cmd_vel             geometry_msgs/Twist
//   /interceptors/unit_{id}/position geometry_msgs/Point
//   /mission/engagement_ack          std_msgs/String
//
// Subscribes
// ----------
//   /c2/engagement_orders        acsdg_msgs/EngagementOrder
//   /threats/target_{n}/fused_target acsdg_msgs/FusedTarget (dynamic)
//   /model/coyote_{id}/odometry  nav_msgs/Odometry
//============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <acsdg_msgs/msg/engagement_order.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class CoyoteControllerNode : public rclcpp::Node
{
  // ── Coyote spec values (spec §6.2) ──────────────────────────────────
  static constexpr double kMaxSpeed     = 160.0;  // m/s — Mach 0.45 sustained
  static constexpr double kKillRadius   = 5.0;    // m — frag p≥0.5 (spec §6.2)
  static constexpr int    kLostTicks    = 40;     // 40 × 50 ms = 2 s
  static constexpr double kDt           = 0.05;   // 20 Hz
  // Spec §6.2 also lists a 100 m proximity-fuze arming distance. Phase 2
  // doesn't enforce it (kill check fires regardless of distance from
  // launcher). Phase 3 base-class refactor should add `is_armed(range)`
  // as a virtual hook so each weapon's arming rule is uniform.
  // No fixed cruise altitude — Coyote pursues in 3D directly. Anvil's 2D-
  // pursuit + altitude-hold pattern was needed because radar-noisy tgt_vz
  // extrapolated over 10 s+ t_go was unstable; Coyote's 160 m/s closes
  // engagements in 5–10 s, so the noise window is too short to diverge.

public:
  CoyoteControllerNode() : rclcpp::Node("coyote_controller_node")
  {
    declare_parameter("interceptor_id", 1);
    declare_parameter("home_x",  177.0);
    declare_parameter("home_y",  177.0);
    declare_parameter("home_z",   20.0);

    id_     = get_parameter("interceptor_id").as_int();
    home_x_ = get_parameter("home_x").as_double();
    home_y_ = get_parameter("home_y").as_double();
    home_z_ = get_parameter("home_z").as_double();

    pos_x_ = home_x_;  pos_y_ = home_y_;  pos_z_ = home_z_;

    // ── Publishers ────────────────────────────────────────────────────
    std::string cmd_topic = "/coyote_" + std::to_string(id_) + "/cmd_vel";
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    std::string pos_topic = "/interceptors/unit_" + std::to_string(id_) + "/position";
    position_pub_ = create_publisher<geometry_msgs::msg::Point>(pos_topic, 10);

    ack_pub_ = create_publisher<std_msgs::msg::String>("/mission/engagement_ack", 10);

    // ── Subscriptions ─────────────────────────────────────────────────
    order_sub_ = create_subscription<acsdg_msgs::msg::EngagementOrder>(
      "/c2/engagement_orders", 10,
      [this](acsdg_msgs::msg::EngagementOrder::SharedPtr msg) { onOrder(msg); });

    std::string odom_topic = "/model/coyote_" + std::to_string(id_) + "/odometry";
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        pos_x_ = m->pose.pose.position.x;
        pos_y_ = m->pose.pose.position.y;
        pos_z_ = m->pose.pose.position.z;
        has_odom_ = true;
      });

    // ── 20 Hz control loop ────────────────────────────────────────────
    timer_ = create_wall_timer(50ms, [this]() { controlStep(); });

    RCLCPP_INFO(get_logger(),
      "CoyoteController #%d  home=(%.0f, %.0f, %.0f)  max_speed=%.0fm/s",
      id_, home_x_, home_y_, home_z_, kMaxSpeed);
  }

private:
  void onOrder(const acsdg_msgs::msg::EngagementOrder::SharedPtr msg)
  {
    if (static_cast<int>(msg->interceptor_id) != id_) return;
    if (pursuing_ && static_cast<uint32_t>(target_id_) == msg->target_id) return;

    target_id_       = static_cast<int>(msg->target_id);
    pursuing_        = true;
    lost_ticks_      = 0;
    has_target_data_ = false;

    target_sub_.reset();
    std::string topic =
      "/threats/target_" + std::to_string(target_id_) + "/fused_target";
    target_sub_ = create_subscription<acsdg_msgs::msg::FusedTarget>(
      topic, rclcpp::QoS(10),
      [this](acsdg_msgs::msg::FusedTarget::SharedPtr m) { onTarget(m); });

    RCLCPP_INFO(get_logger(),
      "Coyote #%d assigned to target #%d", id_, target_id_);
  }

  void onTarget(const acsdg_msgs::msg::FusedTarget::SharedPtr msg)
  {
    tgt_x_ = msg->position.x;
    tgt_y_ = msg->position.y;
    tgt_z_ = msg->position.z;
    tgt_vx_ = msg->velocity.x;
    tgt_vy_ = msg->velocity.y;
    tgt_vz_ = msg->velocity.z;
    lost_ticks_      = 0;
    has_target_data_ = true;
  }

  void controlStep()
  {
    if (!has_odom_) {
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }

    if (pursuing_) {
      ++lost_ticks_;
      if (lost_ticks_ > kLostTicks) {
        RCLCPP_INFO(get_logger(),
          "Coyote #%d: target #%d lost — returning home", id_, target_id_);
        publishAck(target_id_, "LOST");
        pursuing_        = false;
        has_target_data_ = false;
        target_sub_.reset();
      } else if (has_target_data_) {
        pursueTarget();
      } else {
        publishCmdVel(0.0, 0.0, 0.0);
      }
    } else {
      stepTowardHome();
    }

    publishPosition();
  }

  void pursueTarget()
  {
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double rz = tgt_z_ - pos_z_;
    const double range = std::sqrt(rx*rx + ry*ry + rz*rz);

    if (range < kKillRadius) {
      // Frag-fuze proximity kill — Coyote detonates within 5 m.
      RCLCPP_INFO(get_logger(),
        "Coyote #%d: NEUTRALISED target #%d at range=%.2fm "
        "[FRAG-FUZE] coy(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
        id_, target_id_, range,
        pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
      publishAck(target_id_, "NEUTRALIZED");
      pursuing_        = false;
      has_target_data_ = false;
      target_sub_.reset();
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }

    // 3D proportional pursuit toward predicted intercept point.
    const double t_go   = range / kMaxSpeed;
    const double lead_x = tgt_x_ + tgt_vx_ * t_go;
    const double lead_y = tgt_y_ + tgt_vy_ * t_go;
    const double lead_z = tgt_z_ + tgt_vz_ * t_go;

    const double dx   = lead_x - pos_x_;
    const double dy   = lead_y - pos_y_;
    const double dz   = lead_z - pos_z_;
    const double dmag = std::sqrt(dx*dx + dy*dy + dz*dz);
    const double s    = (dmag > 1e-6) ? (kMaxSpeed / dmag) : 0.0;

    publishCmdVel(dx * s, dy * s, dz * s);
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
    const double kReturnSpeed = 5.0;
    publishCmdVel((dx / dist) * kReturnSpeed,
                  (dy / dist) * kReturnSpeed,
                  (dz / dist) * kReturnSpeed);
  }

  // ── Publishing helpers ──────────────────────────────────────────────

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

  // ── Members ─────────────────────────────────────────────────────────

  int    id_;
  double home_x_, home_y_, home_z_;
  double pos_x_, pos_y_, pos_z_;
  bool   pursuing_{false};
  int    target_id_{0};
  double tgt_x_{0}, tgt_y_{0}, tgt_z_{0};
  double tgt_vx_{0}, tgt_vy_{0}, tgt_vz_{0};
  int    lost_ticks_{0};
  bool   has_target_data_{false};
  bool   has_odom_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr        position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            ack_pub_;
  rclcpp::Subscription<acsdg_msgs::msg::EngagementOrder>::SharedPtr order_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::FusedTarget>::SharedPtr     target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoyoteControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
