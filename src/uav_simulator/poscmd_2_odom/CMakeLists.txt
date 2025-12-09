cmake_minimum_required(VERSION 3.5)
project(poscmd_2_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)
ament_target_dependencies(poscmd_2_odom
  rclcpp
  nav_msgs
  quadrotor_msgs
  Eigen3
)

install(TARGETS poscmd_2_odom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
