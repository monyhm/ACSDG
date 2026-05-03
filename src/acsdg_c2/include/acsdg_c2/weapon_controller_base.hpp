//============================================================================
// weapon_controller_base.hpp -- Abstract base for per-weapon flight controllers.
//
// Concrete subclasses (AnvilControllerNode, CoyoteControllerNode, and Phase 3+
// DroneHunterControllerNode etc.) override two virtuals:
//   - computePursuitCmd: write WORLD-frame (vx, vy, vz) for one tick
//   - onKill:            decide outcome when target enters kill_radius_outer
//
// The base class owns all topic plumbing, the watchdog, the return-to-home
// loop, and the 20 Hz timer. ~80% of the per-weapon controller code that
// previously lived in two near-identical files now lives once here.
//============================================================================

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <acsdg_msgs/msg/engagement_order.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace acsdg_c2 {

class WeaponControllerBase : public rclcpp::Node
{
public:
  WeaponControllerBase(const std::string & node_name,
                       const std::string & topic_prefix,
                       double max_speed_mps,
                       double kill_radius_inner_m,
                       double kill_radius_outer_m = 0.0);

protected:
  // ── Virtuals (subclasses MUST override) ──────────────────────────────
  virtual void computePursuitCmd(double & vx, double & vy, double & vz) = 0;

  /**
   * Called when range_m < kill_radius_outer_. Return:
   *   "NEUTRALIZED" or "MISS" → base publishes ack, ends pursuit, returns home
   *   ""                       → base continues pursuit (no-kill-this-tick)
   */
  virtual std::string onKill(double range_m) = 0;

  // ── Shared protected state subclasses can read ───────────────────────
  int     id_{0};
  double  pos_x_{0}, pos_y_{0}, pos_z_{0};
  double  tgt_x_{0}, tgt_y_{0}, tgt_z_{0};
  double  tgt_vx_{0}, tgt_vy_{0}, tgt_vz_{0};
  bool    pursuing_{false};
  bool    has_target_data_{false};
  bool    has_odom_{false};
  int     target_id_{0};
  double  home_x_{0}, home_y_{0}, home_z_{0};
  int     lost_ticks_{0};

  const double max_speed_;
  const double kill_radius_inner_;
  const double kill_radius_outer_;   // == inner if no outer ring

  // Constants shared across weapons
  static constexpr int    kLostTicks    = 40;     // 40 × 50 ms = 2 s
  static constexpr double kDt           = 0.05;   // 20 Hz

private:
  // ── Shared implementation (NOT virtual) ──────────────────────────────
  void onOrder(const acsdg_msgs::msg::EngagementOrder::SharedPtr msg);
  void onTarget(const acsdg_msgs::msg::FusedTarget::SharedPtr msg);
  void controlStep();
  void stepTowardHome();
  void publishCmdVel(double vx, double vy, double vz);
  void publishPosition();
  void publishAck(int target_id, const std::string & outcome);

  std::string                                                            topic_prefix_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr                position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                    ack_pub_;
  rclcpp::Subscription<acsdg_msgs::msg::EngagementOrder>::SharedPtr      order_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::FusedTarget>::SharedPtr          target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr               odom_sub_;
  rclcpp::TimerBase::SharedPtr                                           timer_;
};

}  // namespace acsdg_c2
