//============================================================================
// dronehunter_controller_node.cpp -- Fortem DroneHunter F700 net-capture
// octocopter controller.
//
// Extends acsdg_c2::WeaponControllerBase. DroneHunter-specific behaviour:
//   - 2D lead pursuit + altitude hold at cruise z (same pattern as Anvil —
//     31 m/s closes typical engagements in ~30s, long enough that 3D pursuit
//     would amplify radar-noisy tgt_vz over t_go)
//   - Binary kill at 15 m net-deploy range (no probabilistic ring; net
//     deployment is deterministic at close range)
//   - [NET-CAPTURE] log tag distinguishes this weapon class in demo logs
//
// The 180 s multi-shot cooldown is enforced Python-side in DroneHunter's
// is_available() override — this controller has no cooldown logic.
//============================================================================

#include "acsdg_c2/weapon_controller_base.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class DroneHunterControllerNode : public acsdg_c2::WeaponControllerBase
{
  // DroneHunter spec values (spec §6.3)
  static constexpr double kMaxSpeed   = 31.0;   // m/s — post-2024 doubled-speed
  static constexpr double kKillRadius = 15.0;   // m — net-deploy range
  static constexpr double kCruiseZ    = 50.0;   // m — fixed cruise altitude
  static constexpr double kAltKp      = 0.5;    // altitude-hold P gain
  static constexpr double kMaxVz      = 3.0;    // m/s — vertical clamp

public:
  DroneHunterControllerNode()
    : WeaponControllerBase("dronehunter_controller_node", "/dronehunter_",
                           kMaxSpeed, kKillRadius,
                           /*kill_radius_outer_m=*/0.0,
                           /*gz_model_kind=*/"dronehunter")
  {
    RCLCPP_INFO(get_logger(),
      "DroneHunterController #%d  home=(%.0f, %.0f, %.0f)  max_speed=%.0fm/s",
      id_, home_x_, home_y_, home_z_, kMaxSpeed);
  }

protected:
  void computePursuitCmd(double & vx, double & vy, double & vz) override
  {
    // 2D lead pursuit toward predicted intercept point at cruise altitude.
    // Same pattern as Anvil — see interceptor_controller_node.cpp for the
    // rationale (3D pursuit was rejected; radar-noisy tgt_vz over a long
    // t_go blew interceptors hundreds of metres above their targets).
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
      "DroneHunter #%d: NEUTRALISED target #%d at range=%.2fm "
      "[NET-CAPTURE] dh(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
      id_, target_id_, range_m,
      pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
    return "NEUTRALIZED";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneHunterControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
