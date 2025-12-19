cmake_minimum_required(VERSION 3.8)
project(traj_utils)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories( 
  include 
  ${EIGEN3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(traj_utils
  src/planning_visualization.cpp 
)

ament_target_dependencies(traj_utils
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  bspline
  bspline_opt
  path_searching
  poly_traj
  pcl_conversions
  quadrotor_msgs
)

target_link_libraries(traj_utils
  ${PCL_LIBRARIES}
)

target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(process_msg 
  src/process_msg.cpp
)

ament_target_dependencies(process_msg
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  quadrotor_msgs
)

target_link_libraries(process_msg
  ${PCL_LIBRARIES}
)

add_executable(circle_publisher 
  src/circle_publisher.cpp
)

ament_target_dependencies(circle_publisher
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
)

# Install
install(TARGETS traj_utils process_msg circle_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_dependencies(rclcpp std_msgs sensor_msgs visualization_msgs cv_bridge bspline bspline_opt path_searching poly_traj quadrotor_msgs)

ament_package()  