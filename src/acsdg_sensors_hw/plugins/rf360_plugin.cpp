//============================================================================
// rf360_plugin.cpp — Gazebo system plugin simulating the Dedrone RF-360
// wideband RF direction-finding sensor.
//
// The RF-360 is omnidirectional; this plugin is attached at the world level
// (or per-instance model) and simulates:
//   • RF emissions from every "enemy_" entity (random drone control freq)
//   • Free-space path loss: RSSI = TxPower - 20·log10(range) - PathLoss
//   • AoA direction finding with ±5° Gaussian noise
//   • 2000 m detection range
//
// Publishes JSON to:
//   /rf360_{sensor_id}/rf_detections  (gz.msgs.StringMsg)
//
// SDF parameters:
//   <sensor_id>  integer  (default 1)
//   <update_rate> float   Hz (default 1)
//   <sensor_x>  <sensor_y>  <sensor_z>  — position override if needed
//============================================================================

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>    // defines WorldPose
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/math/Pose3.hh>
#include <sdf/sdf.hh>

#include <cmath>
#include <random>
#include <sstream>
#include <iomanip>
#include <string>
#include <array>

namespace acsdg_sensors_hw {

// ── RF simulation constants matching Dedrone RF-360 specs ─────────────────
static constexpr double kMaxDetectRange    = 2000.0;   // metres
static constexpr double kAoANoiseSigmaDeg  = 5.0;      // ±5° accuracy
static constexpr double kTxPowerDbm        = 20.0;     // typical drone TX power
static constexpr double kPathLossExponent  = 2.0;      // free-space

// Drone control frequencies (randomly assigned per drone per detection cycle)
static constexpr std::array<double, 4> kFreqsHz{
  433e6, 915e6, 2.4e9, 5.8e9
};

// ── RF360Plugin ────────────────────────────────────────────────────────────

class RF360Plugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & /*ecm*/,
    gz::sim::EventManager & /*em*/) override
  {
    modelEntity_ = entity;

    if (sdf->HasElement("sensor_id"))   sensorId_     = sdf->Get<int>("sensor_id");
    if (sdf->HasElement("update_rate")) updateRateHz_ = sdf->Get<double>("update_rate");
    if (sdf->HasElement("sensor_x"))    sensorX_      = sdf->Get<double>("sensor_x");
    if (sdf->HasElement("sensor_y"))    sensorY_      = sdf->Get<double>("sensor_y");
    if (sdf->HasElement("sensor_z"))    sensorZ_      = sdf->Get<double>("sensor_z");

    std::string topic = "/rf360_" + std::to_string(sensorId_) + "/rf_detections";
    pub_ = node_.Advertise<gz::msgs::StringMsg>(topic);

    gzmsg << "[RF360Plugin] sensor_id=" << sensorId_
          << "  topic=" << topic
          << "  rate=" << updateRateHz_ << " Hz\n";
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    double simSec = std::chrono::duration<double>(info.simTime).count();
    if (simSec - lastPublishSec_ < 1.0 / updateRateHz_) return;
    lastPublishSec_ = simSec;

    if (info.paused) return;

    // Determine sensor position: prefer model WorldPose, fall back to SDF params
    double rx = sensorX_, ry = sensorY_, rz = sensorZ_;
    if (modelEntity_ != gz::sim::kNullEntity) {
      auto selfPoseComp = ecm.Component<gz::sim::components::WorldPose>(modelEntity_);
      if (selfPoseComp) {
        const auto & p = selfPoseComp->Data().Pos();
        rx = p.X(); ry = p.Y(); rz = p.Z();
      }
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(4) << "[";
    bool firstEntry = true;

    std::uniform_int_distribution<size_t> freqPicker(0, kFreqsHz.size() - 1);
    std::normal_distribution<double> aoaNoise(0.0, kAoANoiseSigmaDeg);

    ecm.Each<gz::sim::components::WorldPose,
             gz::sim::components::Name>(
      [&](const gz::sim::Entity & /*ent*/,
          const gz::sim::components::WorldPose * posComp,
          const gz::sim::components::Name * nameComp) -> bool
      {
        if (nameComp->Data().find("enemy_") == std::string::npos) return true;

        const auto & dp = posComp->Data().Pos();
        double dx = dp.X() - rx;
        double dy = dp.Y() - ry;
        double dz = dp.Z() - rz;
        double range = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (range < 0.5 || range > kMaxDetectRange) return true;

        // Free-space path loss RSSI model
        double rssi = kTxPowerDbm
                      - 20.0 * std::log10(std::max(range, 1.0))
                      - 10.0 * kPathLossExponent * std::log10(4.0 * M_PI / 0.125);
        // Clamp to realistic receiver sensitivity floor
        rssi = std::max(rssi, -110.0);

        // AoA bearing with noise
        double bearing = std::atan2(dy, dx) * 180.0 / M_PI + aoaNoise(rng_);

        // Random frequency assignment this cycle
        double freq = kFreqsHz[freqPicker(rng_)];

        // Parse drone index from name for id field
        uint32_t droneId = 0;
        const std::string & nm = nameComp->Data();
        size_t sep = nm.rfind('_');
        if (sep != std::string::npos) {
          try { droneId = std::stoul(nm.substr(sep + 1)); }
          catch (...) {}
        }

        if (!firstEntry) json << ",";
        firstEntry = false;
        json << "{"
             << "\"id\":"      << droneId   << ","
             << "\"bearing_deg\":" << bearing << ","
             << "\"rssi_dbm\":" << rssi     << ","
             << "\"freq_hz\":" << freq      << ","
             << "\"range_m\":" << range
             << "}";

        return true;
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

  int    sensorId_      {1};
  double updateRateHz_  {1.0};
  double lastPublishSec_{-1.0};

  // Default sensor position (overridden by model WorldPose if available)
  double sensorX_{0.0}, sensorY_{0.0}, sensorZ_{3.0};

  std::mt19937 rng_{std::random_device{}()};
};

}  // namespace acsdg_sensors_hw

GZ_ADD_PLUGIN(acsdg_sensors_hw::RF360Plugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(acsdg_sensors_hw::RF360Plugin, "RF360Plugin")
