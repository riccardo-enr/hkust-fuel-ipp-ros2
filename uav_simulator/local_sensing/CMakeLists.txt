cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CUDA support disabled by default (can be enabled if needed)
set(ENABLE_CUDA false)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include 
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if(ENABLE_CUDA)
  # CUDA support - would require additional setup
  message(WARNING "CUDA support not yet fully implemented for ROS 2")
else()
  add_executable(pcl_render_node
    src/depth_render_node.cpp
  )
  
  ament_target_dependencies(pcl_render_node
    rclcpp
    cv_bridge
    pcl_ros
    sensor_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
  )
  
  target_link_libraries(pcl_render_node
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
  )
endif()

# Install
install(TARGETS pcl_render_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
