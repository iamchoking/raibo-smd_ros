cmake_minimum_required(VERSION 3.5)
project(raibo-smd_ros)

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

# sensors base packages
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


install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)


add_executable(raibo_pcl2_bridge src/raibo_pcl2_bridge)

install(TARGETS
  raibo_pcl2_bridge
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

# add_executable(listener src/subscriber_member_function.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# install(TARGETS
#   listener
#   DESTINATION lib/${PROJECT_NAME})

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