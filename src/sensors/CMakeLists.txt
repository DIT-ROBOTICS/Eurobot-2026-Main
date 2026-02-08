cmake_minimum_required(VERSION 3.8)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(utils REQUIRED)

# Include directories
include_directories(
  include
)

# Create library for sensors nodes
add_library(sensors_nodes SHARED
  src/CamReceiver.cpp
  src/LocReceiver.cpp
)

ament_target_dependencies(sensors_nodes
  rclcpp
  rclcpp_action
  rclcpp_components
  behaviortree_ros2
  behaviortree_cpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  btcpp_ros2_interfaces
  opennav_docking_msgs
  utils
)

# Install library
install(TARGETS sensors_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config and params
# install(DIRECTORY
#   params
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(sensors_nodes)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_ros2
  behaviortree_cpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  btcpp_ros2_interfaces
  opennav_docking_msgs
  utils
)

ament_package()