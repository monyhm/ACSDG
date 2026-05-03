//============================================================================
// interceptor_controller_node.cpp -- Anvil quadcopter flight controller.
//
// Phase 3 prep refactor: extends acsdg_c2::WeaponControllerBase. The 20% of
// behaviour unique to the Anvil — 2D pursuit + altitude hold at kCruiseZ —
// lives in computePursuitCmd; the kill check is binary collision-radius.
//============================================================================

#include "acsdg_c2/weapon_controller_base.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class AnvilControllerNode : public acsdg_c2::WeaponControllerBase
{
  // Anvil-specific constants
  static constexpr double kMaxSpeed     = 15.0;   // m/s
  static constexpr double kKillRadius   = 8.0;    // m — kinetic ramming collision
  static constexpr double kCruiseZ      = 50.0;   // m — fixed cruise altitude
  static constexpr double kAltKp        = 0.5;    // altitude-hold P gain
  static constexpr double kMaxVz        = 3.0;    // m/s — clamp on vertical velocity

public:
  AnvilControllerNode()
    : WeaponControllerBase("interceptor_controller_node", "/interceptor_",
                           kMaxSpeed, kKillRadius,
                           /*kill_radius_outer_m=*/0.0,
                           /*gz_model_kind=*/"interceptor")
  {
    RCLCPP_INFO(get_logger(),
      "InterceptorController #%d  home=(%.0f, %.0f, %.0f)  max_speed=%.0fm/s",
      id_, home_x_, home_y_, home_z_, kMaxSpeed);
  }

protected:
  void computePursuitCmd(double & vx, double & vy, double & vz) override
  {
    // 2D lead pursuit toward predicted intercept point at cruise altitude.
    // 3D was tried and rejected: radar-noisy tgt_vz extrapolated over a 10+ s
    // t_go blew the interceptor hundreds of metres above its target.
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double range_xy = std::sqrt(rx*rx + ry*ry);
    const double t_go = range_xy / kMaxSpeed;
    const double lead_x = tgt_x_ + tgt_vx_ * t_go;
    const double lead_y = tgt_y_ + tgt_vy_ * t_go;

    const double dx = lead_x - pos_x_;
    const double dy = lead_y - pos_y_;
    const double dmag = std::sqrt(dx*dx + dy*dy);
    const double s = (dmag > 1e-6) ? (kMaxSpeed / dmag) : 0.0;
    vx = dx * s;
    vy = dy * s;

    // Altitude-hold P-loop independent of xy pursuit
    const double dz = kCruiseZ - pos_z_;
    vz = std::clamp(kAltKp * dz, -kMaxVz, kMaxVz);
  }

  std::string onKill(double range_m) override
  {
    RCLCPP_INFO(get_logger(),
      "Interceptor #%d: NEUTRALISED target #%d at range=%.2fm "
      "int(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
      id_, target_id_, range_m,
      pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
    return "NEUTRALIZED";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AnvilControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
