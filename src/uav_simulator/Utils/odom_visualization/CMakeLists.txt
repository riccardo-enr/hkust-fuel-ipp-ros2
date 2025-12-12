cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(odom_visualization src/odom_visualization.cpp)

ament_target_dependencies(odom_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  pose_utils
)

target_link_libraries(odom_visualization
  ${ARMADILLO_LIBRARIES}
)

# Install
install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
