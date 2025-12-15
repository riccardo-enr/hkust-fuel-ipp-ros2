cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(cmake_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

ament_target_dependencies(quadrotor_dynamics
  rclcpp
  quadrotor_msgs
  uav_utils
  cmake_utils
)

add_executable(quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp
)

ament_target_dependencies(quadrotor_simulator_so3
  rclcpp
  quadrotor_msgs
  uav_utils
  cmake_utils
  nav_msgs
  sensor_msgs
  geometry_msgs
)

target_link_libraries(quadrotor_simulator_so3 
  quadrotor_dynamics
)

# Install
install(TARGETS 
  quadrotor_dynamics
  quadrotor_simulator_so3
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
ament_export_libraries(quadrotor_dynamics)
ament_export_dependencies(rclcpp quadrotor_msgs uav_utils cmake_utils)

ament_package()
