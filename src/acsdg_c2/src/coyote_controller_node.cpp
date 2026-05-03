//============================================================================
// coyote_controller_node.cpp -- Raytheon Coyote Block 2 frag-jet controller.
//
// Phase 3 prep refactor: extends acsdg_c2::WeaponControllerBase. The 20% of
// behaviour unique to the Coyote — 3D proportional pursuit (no altitude hold)
// and the [FRAG-FUZE] kill log — lives in computePursuitCmd / onKill. The
// 5–8 m probabilistic frag-fuze ring lands in Task 10; this task preserves
// the binary 5 m kill from Phase 2.
//============================================================================

#include "acsdg_c2/weapon_controller_base.hpp"

#include <cmath>
#include <random>
#include <sstream>

class CoyoteControllerNode : public acsdg_c2::WeaponControllerBase
{
  // Coyote spec values (spec §6.2)
  static constexpr double kMaxSpeed         = 160.0;  // m/s
  static constexpr double kKillRadiusInner  = 5.0;    // m — frag p≥0.5 guaranteed kill
  static constexpr double kKillRadiusOuter  = 8.0;    // m — frag p≥0.3 probabilistic

public:
  CoyoteControllerNode()
    : WeaponControllerBase("coyote_controller_node", "/coyote_",
                           kMaxSpeed, kKillRadiusInner, kKillRadiusOuter,
                           /*gz_model_kind=*/"coyote"),
      uniform_(0.0, 1.0)
  {
    declare_parameter("pkill_small_quad", 0.60);
    declare_parameter("rng_seed", -1);
    pkill_small_quad_ = get_parameter("pkill_small_quad").as_double();
    const int seed_param = get_parameter("rng_seed").as_int();
    const unsigned int seed = (seed_param < 0)
        ? std::random_device{}()
        : static_cast<unsigned int>(seed_param);
    rng_.seed(seed);

    RCLCPP_INFO(get_logger(),
      "CoyoteController #%d  home=(%.0f, %.0f, %.0f)  max_speed=%.0fm/s  "
      "pkill_sq=%.2f  rng_seed=%u",
      id_, home_x_, home_y_, home_z_, kMaxSpeed, pkill_small_quad_, seed);
  }

protected:
  void computePursuitCmd(double & vx, double & vy, double & vz) override
  {
    // 3D proportional pursuit toward predicted lead point.
    // No altitude hold: Coyote's 160 m/s closes engagements in 5–10 s, so the
    // radar-noise window on tgt_vz is too short to diverge over t_go.
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double rz = tgt_z_ - pos_z_;
    const double range = std::sqrt(rx*rx + ry*ry + rz*rz);

    const double t_go   = range / kMaxSpeed;
    const double lead_x = tgt_x_ + tgt_vx_ * t_go;
    const double lead_y = tgt_y_ + tgt_vy_ * t_go;
    const double lead_z = tgt_z_ + tgt_vz_ * t_go;

    const double dx = lead_x - pos_x_;
    const double dy = lead_y - pos_y_;
    const double dz = lead_z - pos_z_;
    const double dmag = std::sqrt(dx*dx + dy*dy + dz*dz);
    const double s = (dmag > 1e-6) ? (kMaxSpeed / dmag) : 0.0;

    vx = dx * s; vy = dy * s; vz = dz * s;
  }

  std::string onKill(double range_m) override
  {
    if (range_m < kKillRadiusInner) {
      // Inner ring (range < 5 m): guaranteed kill, p50 spec band.
      RCLCPP_INFO(get_logger(),
        "Coyote #%d: NEUTRALISED target #%d at range=%.2fm "
        "[FRAG-FUZE p50] coy(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
        id_, target_id_, range_m,
        pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
      return "NEUTRALIZED";
    }

    // Outer ring (5 m ≤ range < 8 m): probabilistic kill governed by Pkill table.
    const double roll = uniform_(rng_);
    const bool killed = (roll <= pkill_small_quad_);
    const char * tag = killed ? "[FRAG-FUZE p30]" : "[FRAG-FUZE MISS]";
    RCLCPP_INFO(get_logger(),
      "Coyote #%d frag roll=%.3f vs pkill=%.3f at range=%.2fm → %s",
      id_, roll, pkill_small_quad_, range_m, killed ? "KILL" : "MISS");
    RCLCPP_INFO(get_logger(),
      "Coyote #%d: %s target #%d at range=%.2fm %s",
      id_, killed ? "NEUTRALISED" : "MISSED", target_id_, range_m, tag);
    return killed ? "NEUTRALIZED" : "MISS";
  }

private:
  double pkill_small_quad_{0.60};
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoyoteControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
