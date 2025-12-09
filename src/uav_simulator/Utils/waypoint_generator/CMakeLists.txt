cmake_minimum_required(VERSION 3.5)
project(waypoint_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -march=native")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(waypoint_generator src/waypoint_generator.cpp)
ament_target_dependencies(waypoint_generator
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  Eigen3
)

install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
