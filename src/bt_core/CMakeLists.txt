cmake_minimum_required(VERSION 3.8)
project(bt_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(sensors REQUIRED)
find_package(navigation REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../sensors/include
  ${CMAKE_SOURCE_DIR}/../navigation/include
  ${CMAKE_SOURCE_DIR}/../utils/include
)

# Main executable - bt_engine
add_executable(bt_engine 
  src/bt_engine.cpp
  src/decision_core.cpp
  src/MissionPublisher.cpp
  src/MissionChecker.cpp
  src/FieldUpdater.cpp
)
ament_target_dependencies(bt_engine
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "nav2_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "jsoncpp"
  "btcpp_ros2_interfaces"
  "opennav_docking_msgs"
  "behaviortree_cpp"
  "behaviortree_ros2"
)
target_link_libraries(bt_engine
  jsoncpp
  ${sensors_LIBRARIES}
  ${navigation_LIBRARIES}
)

install(TARGETS
  bt_engine
  DESTINATION lib/${PROJECT_NAME}
)

# Install params, bt trees, and launch files
install(DIRECTORY
  params
  bt
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
