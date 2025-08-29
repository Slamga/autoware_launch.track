#include "track_vehicle_interface.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<autoware::track_vehocle_interface::TrackVehicleInterface>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
