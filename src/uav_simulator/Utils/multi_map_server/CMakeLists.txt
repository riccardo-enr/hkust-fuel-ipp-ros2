cmake_minimum_required(VERSION 3.8)
project(multi_map_server)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Armadillo REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiOccupancyGrid.msg"
  "msg/MultiSparseMap3D.msg"
  "msg/SparseMap3D.msg"
  "msg/VerticalOccupancyGridList.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

# Include directories
include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

# Build executable
add_executable(multi_map_visualization src/multi_map_visualization.cc)

ament_target_dependencies(multi_map_visualization
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
  laser_geometry
  pose_utils
  quadrotor_msgs
)

target_link_libraries(multi_map_visualization
  ${ARMADILLO_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(multi_map_visualization "${cpp_typesupport_target}")

# Install
install(TARGETS
  multi_map_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
