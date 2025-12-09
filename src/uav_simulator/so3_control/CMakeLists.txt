cmake_minimum_required(VERSION 3.5)
project(so3_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cmake_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(SO3Control src/SO3Control.cpp)
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

add_library(so3_control_nodelet SHARED src/so3_control_nodelet.cpp)
target_include_directories(so3_control_nodelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(so3_control_nodelet SO3Control)
ament_target_dependencies(so3_control_nodelet
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  nav_msgs
  quadrotor_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

rclcpp_components_register_node(so3_control_nodelet
  PLUGIN "SO3ControlNodelet"
  EXECUTABLE so3_control_nodelet_exe
)

install(TARGETS SO3Control so3_control_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
