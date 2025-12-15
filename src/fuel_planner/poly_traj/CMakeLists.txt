cmake_minimum_required(VERSION 3.8)
project(poly_traj)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(swarmtal_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(poly_traj
  src/polynomial_traj.cpp
)

ament_target_dependencies(poly_traj
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  swarmtal_msgs
)

target_include_directories(poly_traj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS poly_traj
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(traj_generator
  src/traj_generator.cpp
)

target_link_libraries(traj_generator
  poly_traj
)

ament_target_dependencies(traj_generator
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  swarmtal_msgs
)

install(TARGETS traj_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(poly_traj)
ament_export_dependencies(rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs swarmtal_msgs)

ament_package()  
