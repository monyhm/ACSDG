#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace acsdg_sensors {

/**
 * @brief Hardware abstraction layer for all ACSDG sensor nodes.
 *
 * Concrete sensor drivers (radar, RF, camera, etc.) inherit this class
 * and implement the three lifecycle methods.  The class itself is a
 * fully-functional rclcpp::Node so subclasses get the full ROS2 API.
 */
class SensorInterface : public rclcpp::Node
{
public:
  explicit SensorInterface(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    node_name_(node_name),
    publish_rate_(10.0),
    frame_id_("world")
  {}

  virtual ~SensorInterface() = default;

  /**
   * @brief Initialise hardware resources and ROS publishers/subscribers.
   *        Must be called once before startStreaming().
   */
  virtual void initialize() = 0;

  /**
   * @brief Begin publishing sensor data at publish_rate_ Hz.
   */
  virtual void startStreaming() = 0;

  /**
   * @brief Stop publishing and release any acquired hardware resources.
   */
  virtual void stopStreaming() = 0;

protected:
  std::string node_name_;    ///< ROS node name, also used for logging
  double      publish_rate_; ///< Desired output frequency in Hz
  std::string frame_id_;     ///< TF reference frame for all measurements
};

}  // namespace acsdg_sensors
