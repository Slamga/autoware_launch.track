#ifndef TRACK_VEHICLE_INTERFACE_HPP_
#define TRACK_VEHICLE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <tier4_vehicle_msgs/msg/ActuationCommandStamped.hpp>
#include <tier4_vehicle_msgs/msg/ShiftStamped.hpp>
#include <tier4_vehicle_msgs/msg/VehicleEmergencyStamped.hpp>

#include <fcntl.h>
#include <modbus/modbus.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#define TRACK_VEHICLE_TREAD 0.67
class TrackVehicleInterface : public rclcpp::Node
{
public:
  explicit TrackVehicleInterface(const rclcpp::NodeOptions & node_options);
  ~TrackVehicleInterface();
  void convert_ackermann_to_differential(
    autoware_auto_control_msgs::msg::AckermannControlCommand & ackermann_msg, double & left,
    double & right);

private:
  modbus_t * ctx;
  // from autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_cmd_sub_;
  // from vehicle
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  // autoware command messages
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr_;
  // callbacks
  void callback_control_cmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void callback_emergency_cmd(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  // void to_vehicle();
  // void from_vehicle();
  bool energency = false;
}
#endif
