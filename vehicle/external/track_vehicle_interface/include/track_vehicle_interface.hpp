#ifndef TRACK_VEHICLE_INTERFACE_HPP_
#define TRACK_VEHICLE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"
#include "tier4_vehicle_msgs/msg/shift_stamped.hpp"
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <fcntl.h>
#include <modbus/modbus.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#define TRACK_VEHICLE_TREAD 0.67
namespace autoware::track_vehocle_interface
{
enum Shift { NONE = 0, PARKING = 1, REVERSE = 2, NEUTRAL = 3, DRIVE = 4, LOW = 5, EMERGENCE = 6 };
using autoware_vehicle_msgs::msg::GearCommand;
class TrackVehicleInterface : public rclcpp::Node
{
public:
  explicit TrackVehicleInterface(const rclcpp::NodeOptions & node_options);
  ~TrackVehicleInterface();
  void convert_ackermann_to_differential(
    autoware_control_msgs::msg::Control & ackermann_msg, double & left, double & right);

private:
  double steering_zoom_ = 0;
  double velocity_zoom_ = 0;
  modbus_t * ctx;
  // from autoware
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr
    emergency_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr shift_cmd_sub_;
  // from vehicle
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr
    actuation_status_pub_;
  // autoware command messages
  autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr_;
  // callbacks
  void callback_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
  void callback_emergency_cmd(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  void callback_shift_cmd(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  // void to_vehicle();
  // void from_vehicle();
  Shift shift_ = NONE;
};
}  // namespace autoware::track_vehocle_interface
#endif
