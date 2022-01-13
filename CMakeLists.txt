cmake_minimum_required(VERSION 3.5)
project(raibo-smd_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(realsense2_camera      REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(velodyne               REQUIRED)
find_package(velodyne_driver        REQUIRED)
find_package(velodyne_laserscan     REQUIRED)
find_package(velodyne_pointcloud    REQUIRED)

find_package(rclcpp                 REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(sensor_msgs            REQUIRED)
find_package(pcl_ros                REQUIRED)
find_package(tf2_sensor_msgs        REQUIRED)
find_package(message_filters        REQUIRED)


install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(raibo_pcl2_bridge src/raibo_pcl2_bridge.cpp)
add_executable(syncer_bridge     src/sync_bridge.cpp)

install(TARGETS
  raibo_pcl2_bridge
  syncer_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(raibo_pcl2_bridge
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

ament_target_dependencies(syncer_bridge
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()