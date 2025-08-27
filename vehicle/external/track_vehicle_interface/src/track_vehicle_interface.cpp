#include <track_vehicle_interface.hpp>
TrackVehicleInterface::TrackVehicleInterface(const rclcpp::NodeOptions & node_options)
: Node("TrackVehicleInterface", node_options)
{
  ctx = modbus_new_tcp("192.168.1.2", 502);  // 改成目标 IP/端口
  if (modbus_connect(ctx) == -1) {
    std::cerr << "Connection failed: " << modbus_strerror(errno) << "\n";
    return;
  }
  uint16_t tab[10];
  int rc = modbus_read_registers(ctx, 10, 4, tab);
  if (rc == -1) {
    std::cerr << "Read failed: " << modbus_strerror(errno) << "\n";
  } else {
    for (int i = 0; i < rc; ++i) std::cout << "reg[" << i << "] = " << tab[i] << "\n";
  }
  uint16_t data[4];
  {
    data[0] = data[1] = data[2] = data[3] = 0;
    modbus_write_registers(ctx, 20, 4, data);
  }
  /* subscribers */
  using std::placeholders::_1;
  // from autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1,
    std::bind(&TrackVehicleInterface::callback_control_cmd, this, _1));
  emergency_cmd_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&TrackVehicleInterface::callback_emergency_cmd, this, _1));

  // to autoware
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
}

~TrackVehicleInterface::TrackVehicleInterface()
{
  uint16_t data[4];
  data[0] = data[1] = data[2] = data[3] = 0;
  modbus_write_registers(ctx, 20, 4, data);
  modbus_close(ctx);
  modbus_free(ctx);
}

void TrackVehicleInterface::callback_control_cmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  double left, right;
  convert_ackermann_to_differential(msg, left, right);
  uint16_t data[4];
  if (left > 0) {
    data[1] = data[3] = 2000;
  } else {
    data[1] = data[3] = 0;
  }
  data[0] = left;
  data[2] = right;
  if (!energency) modbus_write_registers(ctx, 20, 4, data);
  control_cmd_ptr_ = msg;
}

void TrackVehicleInterface::callback_emergency_cmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  if (msg.emergency) {
    energency = true;
    uint16_t data[4];
    data[0] = data[1] = data[2] = data[3] = 0;
    modbus_write_registers(ctx, 20, 4, data);
  } else {
    energency = false;
  }
}

// void TrackVehicleInterface::to_vehicle()
// {
//   // you should implement this structure according to your own vehicle design
//   //   control_command_to_vehicle(control_cmd_ptr_);
// }

// void TrackVehicleInterface::to_autoware()
// {
//   // you should implement this structure according to your own vehicle design
//   autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
//   convert_gear_status_to_autoware_msg(gear_report_msg);
//   gear_status_pub_->publish(gear_report_msg);
// }

void TrackVehicleInterface::convert_ackermann_to_differential(
  autoware_auto_control_msgs::msg::AckermannControlCommand & ackermann_msg, double & left,
  double & right)
{
  left = ackermann_msg.longitudinal.speed -
         (ackermann_msg.lateral.steering_tire_angle * TRACK_VEHICLE_TREAD) / 2;
  right = ackermann_msg.longitudinal.speed +
          (ackermann_msg.lateral.steering_tire_angle * TRACK_VEHICLE_TREAD) / 2;
}