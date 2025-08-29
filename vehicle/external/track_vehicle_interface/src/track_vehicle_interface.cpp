#include <track_vehicle_interface.hpp>
namespace autoware::track_vehocle_interface
{
TrackVehicleInterface::TrackVehicleInterface(const rclcpp::NodeOptions & node_options)
: Node("TrackVehicleInterface", node_options)
{
  steering_zoom_ = declare_parameter<double>("steering_zoom", 10.0);
  velocity_zoom_ = declare_parameter<double>("velocity_zoom", 250.0);
  RCLCPP_INFO(
    this->get_logger(), "zoom: steering: %f. velocity:%f ", steering_zoom_, velocity_zoom_);
  ctx = modbus_new_tcp("192.168.1.2", 502);  // 改成目标 IP/端口
  if (modbus_connect(ctx) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Connection failed: %s.", modbus_strerror(errno));
    std::cerr << "Connection failed: " << modbus_strerror(errno) << "\n";
    return;
  }
  uint16_t tab[10];
  int rc = modbus_read_registers(ctx, 10, 4, tab);
  if (rc == -1) {
    RCLCPP_ERROR(this->get_logger(), "Read failed: %s.", modbus_strerror(errno));
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
  control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1,
    std::bind(&TrackVehicleInterface::callback_control_cmd, this, _1));
  emergency_cmd_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&TrackVehicleInterface::callback_emergency_cmd, this, _1));
  shift_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1,
    std::bind(&TrackVehicleInterface::callback_shift_cmd, this, _1));
  // to autoware
  actuation_status_pub_ = create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status", rclcpp::QoS{1});
}

TrackVehicleInterface::~TrackVehicleInterface()
{
  uint16_t data[4];
  data[0] = data[1] = data[2] = data[3] = 0;
  modbus_write_registers(ctx, 20, 4, data);
  modbus_close(ctx);
  modbus_free(ctx);
}

void TrackVehicleInterface::callback_control_cmd(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  double left = 0, right = 0;
  auto contrl_data = *msg;  // 拷贝出一个非 const 对象
  convert_ackermann_to_differential(contrl_data, left, right);
  uint16_t data[4];
  if (shift_ == REVERSE) {
    data[1] = data[3] = 0;
  } else {
    data[1] = data[3] = 1000;
  }
  // if (shift_ == REVERSE || shift_ == DRIVE) {
  data[0] = left * velocity_zoom_;
  data[2] = right * velocity_zoom_;
  // } else {
  //   data[0] = data[2] = 0;
  // }
  // RCLCPP_INFO(this->get_logger(), "conver msg: l: %f. r:%f ", left, right);
  modbus_write_registers(ctx, 20, 4, data);
  control_cmd_ptr_ = msg;
}

void TrackVehicleInterface::callback_emergency_cmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  if ((*msg).emergency) {
    shift_ = EMERGENCE;
    uint16_t data[4];
    data[0] = data[1] = data[2] = data[3] = 0;
    modbus_write_registers(ctx, 20, 4, data);
  } else {
    shift_ = NONE;
  }
}

void TrackVehicleInterface::callback_shift_cmd(
  const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "shift msg: v: %d", msg->command);
  if (msg->command == GearCommand::REVERSE || msg->command == GearCommand::REVERSE_2) {
    shift_ = REVERSE;
  } else if (
    msg->command == GearCommand::PARK || msg->command == GearCommand::NONE ||
    msg->command == GearCommand::NEUTRAL) {
    shift_ = PARKING;
  } else {
    shift_ = DRIVE;
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
  autoware_control_msgs::msg::Control & ackermann_msg, double & left, double & right)
{
  RCLCPP_INFO(
    this->get_logger(), "cmd msg: v: %f. r:%f ", ackermann_msg.longitudinal.velocity,
    ackermann_msg.lateral.steering_tire_angle);
  left = ackermann_msg.longitudinal.velocity -
         (ackermann_msg.lateral.steering_tire_angle * TRACK_VEHICLE_TREAD * steering_zoom_) / 2;
  right = ackermann_msg.longitudinal.velocity +
          (ackermann_msg.lateral.steering_tire_angle * TRACK_VEHICLE_TREAD * steering_zoom_) / 2;
  // if (ackermann_msg.actuation.brake_cmd > 0) {
  //   left = right = 0;
  // }
}
}  // namespace autoware::track_vehocle_interface