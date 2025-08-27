#include "track_vehicle_interface.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackVehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
