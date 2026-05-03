//============================================================================
// weapon_controller_base.cpp -- Shared implementation for per-weapon controllers.
// See weapon_controller_base.hpp for the public contract.
//============================================================================

#include "acsdg_c2/weapon_controller_base.hpp"

#include <cmath>
#include <sstream>

namespace acsdg_c2 {

using namespace std::chrono_literals;

WeaponControllerBase::WeaponControllerBase(const std::string & node_name,
                                           const std::string & topic_prefix,
                                           double max_speed_mps,
                                           double kill_radius_inner_m,
                                           double kill_radius_outer_m,
                                           const std::string & gz_model_kind)
  : rclcpp::Node(node_name),
    max_speed_(max_speed_mps),
    kill_radius_inner_(kill_radius_inner_m),
    kill_radius_outer_(kill_radius_outer_m > 0.0 ? kill_radius_outer_m : kill_radius_inner_m),
    topic_prefix_(topic_prefix)
{
  declare_parameter("interceptor_id", 1);
  declare_parameter("home_x", 0.0);
  declare_parameter("home_y", 0.0);
  declare_parameter("home_z", 20.0);

  id_     = get_parameter("interceptor_id").as_int();
  home_x_ = get_parameter("home_x").as_double();
  home_y_ = get_parameter("home_y").as_double();
  home_z_ = get_parameter("home_z").as_double();
  pos_x_ = home_x_;  pos_y_ = home_y_;  pos_z_ = home_z_;

  // ── Publishers ──────────────────────────────────────────────────────
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    topic_prefix_ + std::to_string(id_) + "/cmd_vel", 10);
  position_pub_ = create_publisher<geometry_msgs::msg::Point>(
    "/interceptors/unit_" + std::to_string(id_) + "/position", 10);
  ack_pub_ = create_publisher<std_msgs::msg::String>("/mission/engagement_ack", 10);

  // ── Subscriptions ───────────────────────────────────────────────────
  order_sub_ = create_subscription<acsdg_msgs::msg::EngagementOrder>(
    "/c2/engagement_orders", 10,
    [this](acsdg_msgs::msg::EngagementOrder::SharedPtr msg) { onOrder(msg); });

  // Resolve gz model kind: explicit override wins; otherwise derive from
  // topic_prefix by stripping leading '/' and trailing '_'. The explicit
  // override is the recommended form going forward — string surgery is a
  // fallback for back-compat with weapons that haven't been updated.
  std::string kind = gz_model_kind;
  if (kind.empty()) {
    kind = topic_prefix_;
    if (!kind.empty() && kind.front() == '/') kind.erase(0, 1);
    if (!kind.empty() && kind.back() == '_') kind.pop_back();
  }
  const std::string odom_topic = "/model/" + kind + "_" + std::to_string(id_) + "/odometry";
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::SharedPtr m) {
      pos_x_ = m->pose.pose.position.x;
      pos_y_ = m->pose.pose.position.y;
      pos_z_ = m->pose.pose.position.z;
      has_odom_ = true;
    });

  // ── 20 Hz control loop ──────────────────────────────────────────────
  timer_ = create_wall_timer(50ms, [this]() { controlStep(); });
}

void WeaponControllerBase::onOrder(const acsdg_msgs::msg::EngagementOrder::SharedPtr msg)
{
  if (static_cast<int>(msg->interceptor_id) != id_) return;
  if (pursuing_ && static_cast<uint32_t>(target_id_) == msg->target_id) return;

  target_id_       = static_cast<int>(msg->target_id);
  pursuing_        = true;
  lost_ticks_      = 0;
  has_target_data_ = false;

  target_sub_.reset();
  const std::string topic =
    "/threats/target_" + std::to_string(target_id_) + "/fused_target";
  target_sub_ = create_subscription<acsdg_msgs::msg::FusedTarget>(
    topic, rclcpp::QoS(10),
    [this](acsdg_msgs::msg::FusedTarget::SharedPtr m) { onTarget(m); });

  RCLCPP_INFO(get_logger(), "Controller #%d assigned to target #%d", id_, target_id_);
}

void WeaponControllerBase::onTarget(const acsdg_msgs::msg::FusedTarget::SharedPtr msg)
{
  tgt_x_  = msg->position.x; tgt_y_  = msg->position.y; tgt_z_  = msg->position.z;
  tgt_vx_ = msg->velocity.x; tgt_vy_ = msg->velocity.y; tgt_vz_ = msg->velocity.z;
  lost_ticks_      = 0;
  has_target_data_ = true;
}

void WeaponControllerBase::controlStep()
{
  if (!has_odom_) {
    publishCmdVel(0.0, 0.0, 0.0);
    return;
  }

  if (pursuing_) {
    ++lost_ticks_;
    if (lost_ticks_ > kLostTicks) {
      RCLCPP_INFO(get_logger(),
        "Controller #%d: target #%d lost — returning home", id_, target_id_);
      publishAck(target_id_, "LOST");
      pursuing_        = false;
      has_target_data_ = false;
      target_sub_.reset();
    } else if (has_target_data_) {
      // Check kill ring; on hit, let the subclass decide outcome.
      const double rx = tgt_x_ - pos_x_;
      const double ry = tgt_y_ - pos_y_;
      const double rz = tgt_z_ - pos_z_;
      const double range = std::sqrt(rx*rx + ry*ry + rz*rz);
      if (range < kill_radius_outer_) {
        const std::string outcome = onKill(range);
        if (!outcome.empty()) {
          publishAck(target_id_, outcome);
          pursuing_        = false;
          has_target_data_ = false;
          target_sub_.reset();
          publishCmdVel(0.0, 0.0, 0.0);
          publishPosition();
          return;
        }
        // empty → fall through to pursuit (subclass declined this tick)
      }
      double vx, vy, vz;
      computePursuitCmd(vx, vy, vz);
      publishCmdVel(vx, vy, vz);
    } else {
      publishCmdVel(0.0, 0.0, 0.0);
    }
  } else {
    stepTowardHome();
  }

  publishPosition();
}

void WeaponControllerBase::stepTowardHome()
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

void WeaponControllerBase::publishPosition()
{
  geometry_msgs::msg::Point pt;
  pt.x = pos_x_; pt.y = pos_y_; pt.z = pos_z_;
  position_pub_->publish(pt);
}

void WeaponControllerBase::publishCmdVel(double vx, double vy, double vz)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = vx; t.linear.y = vy; t.linear.z = vz;
  cmd_vel_pub_->publish(t);
}

void WeaponControllerBase::publishAck(int target_id, const std::string & outcome)
{
  std::ostringstream oss;
  oss << "{\"target_id\":" << target_id
      << ",\"interceptor_id\":" << id_
      << ",\"outcome\":\"" << outcome << "\"}";
  std_msgs::msg::String msg;
  msg.data = oss.str();
  ack_pub_->publish(msg);
}

}  // namespace acsdg_c2
