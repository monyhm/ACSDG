//============================================================================
// echoguard_radar_plugin.cpp — Gazebo system plugin simulating the
// Echodyne EchoGuard phased-array radar.
//
// Attach as a model plugin on each sensor-post model.  The plugin scans
// for entities whose name starts with "enemy_", computes range/azimuth/
// elevation/Doppler, applies the EchoGuard detection probability model,
// adds Gaussian noise, and publishes JSON detections to:
//   /echoguard_{sensor_id}/radar_returns  (gz.msgs.StringMsg)
//
// Bridged via ros_gz_bridge to ROS2 as std_msgs/String for echoguard_node.
//
// SDF parameters:
//   <sensor_id>  integer  — unique index, used in topic names
//   <update_rate> float   — Hz (default 25)
//============================================================================

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>           // defines WorldPose
#include <gz/sim/components/LinearVelocity.hh>  // defines WorldLinearVelocity
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <cmath>
#include <random>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

namespace acsdg_sensors_hw {

// ── Sensor constants matching real EchoGuard specs ─────────────────────────
static constexpr double kMaxRange          = 1000.0;   // metres
static constexpr double kSureProbRange     = 200.0;    // guaranteed detect inside this
static constexpr double kAzHalfFovDeg      = 60.0;     // ±60° = 120° total
static constexpr double kElHalfFovDeg      = 40.0;     // ±40° = 80°  total
static constexpr double kRangeNoiseSigma   = 0.5;      // metres σ
static constexpr double kAzNoiseSignaDeg   = 0.3;      // degrees σ
static constexpr double kVelNoiseSigma     = 0.05;     // m/s σ

// ── EchoGuardRadarPlugin ───────────────────────────────────────────────────

class EchoGuardRadarPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  // ── ISystemConfigure ─────────────────────────────────────────────────────
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & /*ecm*/,
    gz::sim::EventManager & /*em*/) override
  {
    modelEntity_ = entity;

    // Parse SDF params
    if (sdf->HasElement("sensor_id")) {
      sensorId_ = sdf->Get<int>("sensor_id");
    }
    if (sdf->HasElement("update_rate")) {
      updateRateHz_ = sdf->Get<double>("update_rate");
    }

    // Advertise Gazebo transport topic
    std::string topic = "/echoguard_" + std::to_string(sensorId_) + "/radar_returns";
    pub_ = node_.Advertise<gz::msgs::StringMsg>(topic);

    gzmsg << "[EchoGuardRadarPlugin] sensor_id=" << sensorId_
          << "  topic=" << topic
          << "  rate=" << updateRateHz_ << " Hz\n";
  }

  // ── ISystemPostUpdate ────────────────────────────────────────────────────
  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    // Rate limiting
    double simSec = std::chrono::duration<double>(info.simTime).count();
    if (simSec - lastPublishSec_ < 1.0 / updateRateHz_) return;
    lastPublishSec_ = simSec;

    if (info.paused) return;

    // Get this sensor's world pose
    auto selfPoseComp = ecm.Component<gz::sim::components::WorldPose>(modelEntity_);
    if (!selfPoseComp) return;
    const gz::math::Pose3d selfPose = selfPoseComp->Data();
    const double selfYaw = selfPose.Rot().Yaw();

    // Collect detections from all enemy entities
    std::ostringstream json;
    json << std::fixed << std::setprecision(4) << "[";
    bool firstEntry = true;

    ecm.Each<gz::sim::components::WorldPose,
             gz::sim::components::WorldLinearVelocity,
             gz::sim::components::Name>(
      [&](const gz::sim::Entity & /*ent*/,
          const gz::sim::components::WorldPose * posComp,
          const gz::sim::components::WorldLinearVelocity * velComp,
          const gz::sim::components::Name * nameComp) -> bool
      {
        const std::string & name = nameComp->Data();
        // Only track enemy drones
        if (name.find("enemy_") == std::string::npos) return true;

        const gz::math::Pose3d &  dronePos = posComp->Data();
        const gz::math::Vector3d & droneVel = velComp->Data();

        // Vector from sensor to drone in world frame
        double dx = dronePos.Pos().X() - selfPose.Pos().X();
        double dy = dronePos.Pos().Y() - selfPose.Pos().Y();
        double dz = dronePos.Pos().Z() - selfPose.Pos().Z();

        double range = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (range < 0.5 || range > kMaxRange) return true;

        // Rotate into sensor body frame (subtract sensor yaw)
        double cosY = std::cos(-selfYaw);
        double sinY = std::sin(-selfYaw);
        double lx   = dx * cosY - dy * sinY;
        double ly   = dx * sinY + dy * cosY;

        double azDeg  = std::atan2(ly, lx) * 180.0 / M_PI;
        double elDeg  = std::atan2(dz, std::sqrt(lx*lx + ly*ly)) * 180.0 / M_PI;

        // FOV gate
        if (std::abs(azDeg) > kAzHalfFovDeg) return true;
        if (std::abs(elDeg) > kElHalfFovDeg) return true;

        // Detection probability
        double detectProb = 1.0;
        if (range > kSureProbRange) {
          detectProb = 1.0 - (range - kSureProbRange)
                           / (kMaxRange - kSureProbRange);
        }

        // Monte-Carlo detection gate
        std::uniform_real_distribution<double> uDist(0.0, 1.0);
        if (uDist(rng_) > detectProb) return true;

        // Doppler (radial velocity, positive = receding)
        double ux = dx / range, uy = dy / range, uz = dz / range;
        double doppler = droneVel.X()*ux + droneVel.Y()*uy + droneVel.Z()*uz;

        // Add noise
        std::normal_distribution<double> rangeDist(0.0, kRangeNoiseSigma);
        std::normal_distribution<double> azDist(0.0, kAzNoiseSignaDeg);
        std::normal_distribution<double> velDist(0.0, kVelNoiseSigma);

        double nRange   = range   + rangeDist(rng_);
        double nAz      = azDeg   + azDist(rng_);
        double nEl      = elDeg;   // elevation noise minor, skip for brevity
        double nDoppler = doppler  + velDist(rng_);

        // JSON entry
        if (!firstEntry) json << ",";
        firstEntry = false;
        json << "{"
             << "\"range\":"    << nRange   << ","
             << "\"az_deg\":"   << nAz      << ","
             << "\"el_deg\":"   << nEl      << ","
             << "\"doppler\":"  << nDoppler << ","
             << "\"prob\":"     << detectProb
             << "}";

        return true;  // continue iteration
      });

    json << "]";

    gz::msgs::StringMsg msg;
    msg.set_data(json.str());
    pub_.Publish(msg);
  }

private:
  gz::sim::Entity   modelEntity_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;

  int    sensorId_      {0};
  double updateRateHz_  {25.0};
  double lastPublishSec_{-1.0};

  std::mt19937 rng_{std::random_device{}()};
};

}  // namespace acsdg_sensors_hw

// ── Plugin registration ───────────────────────────────────────────────────
GZ_ADD_PLUGIN(acsdg_sensors_hw::EchoGuardRadarPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(acsdg_sensors_hw::EchoGuardRadarPlugin,
                    "EchoGuardRadarPlugin")
