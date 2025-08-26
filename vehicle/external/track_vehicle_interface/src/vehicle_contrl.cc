#include "rclcpp/rclcpp.hpp"
// #include "vehicle_contrl/msg/contrl.hpp"

#include "std_msgs/msg/string.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>

#include <fcntl.h>
#include <modbus/modbus.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

// Discrete Coils/Flags

// 0x01 - Read Coils
// 0x02 - Read Discrete Inputs
// 0x05 - Write Single Coil
// 0x0F - Write Multiple Coils
// Registers

// 0x03 - Read Holding Registers
// 0x04 - Read Input Registers
// 0x06 - Write Single Register
// 0x10 - Write Multiple Registers
// 0x16 - Mask Write Register
// 0x17 - Read Write Multiple Registers

char getch()
{
  termios oldt, newt;
  char c;

  tcgetattr(STDIN_FILENO, &oldt);  // 获取旧设置
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);         // 关闭缓冲与回显
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 应用新设置

  c = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复旧设置
  return c;
}

void setNonBlockingInput()
{
  termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag &= ~(ICANON | ECHO);  // 非规范模式+无回显
  tcsetattr(STDIN_FILENO, TCSANOW, &t);

  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // 非阻塞
}

void resetInputMode()
{
  termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

class VehicleContrl : public rclcpp::Node
{
public:
  VehicleContrl() : Node("vehicle_contrl")
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
    vehicle_contrl_sub_ = this->create_subscription<std_msgs::msg::String>(
      "vehicle_contrl", 10,
      std::bind(&VehicleContrl::vehicle_contrl_callback, this, std::placeholders::_1));
  }

  void vehicle_contrl_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    uint16_t data[4];
    std::string cmd = msg->data;
    // data[0] 左电机
    // data[1] 左正反
    // data[2] 右电机
    // data[3] 右正反
    if (cmd == "goAhend") {
      data[0] = data[2] = 500;
      data[1] = 2000;
      data[3] = 2000;
    } else if (cmd == "goBack") {
      data[0] = data[2] = 500;
      data[1] = 0;
      data[3] = 0;
    } else if (cmd == "stop") {
      data[0] = data[2] = 0;
      data[1] = 0;
      data[3] = 0;
    } else if (cmd == "turnLeft") {
      data[0] = 500;
      data[2] = 700;
      data[1] = 2000;
      data[3] = 2000;
    } else if (cmd == "turnAright") {
      data[0] = 700;
      data[2] = 500;
      data[1] = 2000;
      data[3] = 2000;
    } else if (cmd == "turnAboutL") {
      data[0] = 500;
      data[2] = 500;
      data[1] = 0;
      data[3] = 2000;
    } else if (cmd == "turnAboutR") {
      data[0] = 500;
      data[2] = 500;
      data[1] = 2000;
      data[3] = 0;
    }
    modbus_write_registers(ctx, 20, 4, data);
  }

  ~VehicleContrl()
  {
    uint16_t data[4];
    data[0] = data[1] = data[2] = data[3] = 0;
    modbus_write_registers(ctx, 20, 4, data);
    modbus_close(ctx);
    modbus_free(ctx);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_contrl_sub_;
  modbus_t * ctx;
  // from autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  // from vehicle
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  // autoware command messages
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  // callbacks
  void callback_control_cmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void to_vehicle();
  void from_vehicle();
};

int main(int argc, char * argv[])

{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleContrl>());
  rclcpp::shutdown();
  return 0;
}
