#pragma once
//============================================================================
// sensor_hw_interface.hpp — Abstract hardware abstraction base for all
// acsdg_sensors_hw sensor nodes.
//
// Key addition over acsdg_sensors::SensorInterface:
//   use_real_hardware (bool, default false)
//     false → use Gazebo simulation path (subscribe to bridged gz topics)
//     true  → activate physical hardware driver path
//
// Downstream topics are IDENTICAL in both modes — callers never need to know
// which backend is active.
//============================================================================

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace acsdg_sensors_hw {

class SensorHwInterface : public rclcpp::Node
{
public:
  explicit SensorHwInterface(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    node_name_(node_name)
  {}

  virtual ~SensorHwInterface() = default;

  /**
   * Declare ROS2 parameters, create publishers/subscribers, and optionally
   * open hardware resources.  Must be called once before startStreaming().
   */
  virtual void initialize() = 0;

  /** Begin publishing sensor data at the configured rate. */
  virtual void startStreaming() = 0;

  /** Cancel timers and release hardware resources. */
  virtual void stopStreaming() = 0;

  bool useRealHardware() const { return use_real_hardware_; }

protected:
  std::string node_name_;
  bool        use_real_hardware_{false};
  int         sensor_id_{1};
  std::string frame_id_{"world"};
  double      publish_rate_{10.0};
};

}  // namespace acsdg_sensors_hw
