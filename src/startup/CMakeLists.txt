cmake_minimum_required(VERSION 3.8)
project(startup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(utils REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../main/include
)

add_executable(startup_new src/startup_new.cpp)
ament_target_dependencies(startup_new
  ${THIS_PACKAGE_DEPS}
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "btcpp_ros2_interfaces"
  "jsoncpp"
  "utils"
)
target_link_libraries(startup_new
  jsoncpp
)

install(TARGETS
  startup_new
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  params
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Launch test — verify startup_new node starts and advertises topics
  add_launch_test(
    test/test_startup_launch.py
    TIMEOUT 30
  )

  # Integration test — full startup pipeline (startup + bt_engine)
  add_launch_test(
    test/test_startup_pipeline.py
    TIMEOUT 60
  )
endif()

ament_package()