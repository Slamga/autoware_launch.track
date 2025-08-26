cmake_minimum_required(VERSION 3.5)
project(track_vehicle_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_library(MODBUS_LIB modbus PATHS /usr/local/lib)
find_path(MODBUA_INCLUDE_DIR modbus.h PATHS /usr/local/include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 自定义 msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Contrl.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
)

# 节点
add_executable(track_vehicle_interface_node src/vehicle_contrl.cc)
ament_target_dependencies(track_vehicle_interface_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)
target_include_directories(track_vehicle_interface_node PRIVATE ${MODBUS_INCLUDE_DIR})
# 确保 msg 先生成，再编译 node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(track_vehicle_interface_node ${cpp_typesupport_target} ${MODBUS_LIB})
add_dependencies(track_vehicle_interface_node ${PROJECT_NAME}__cpp)

install(DIRECTORY launch
  	DESTINATION share/${PROJECT_NAME}
  	)

# install(DIRECTORY config/
#   	DESTINATION share/${PROJECT_NAME}/config
#   	)

install(TARGETS
  track_vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
