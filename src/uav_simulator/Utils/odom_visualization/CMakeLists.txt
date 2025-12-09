cmake_minimum_required(VERSION 3.5)
project(odom_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(odom_visualization src/odom_visualization.cpp)
target_include_directories(odom_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ARMADILLO_INCLUDE_DIRS}
)
ament_target_dependencies(odom_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf2_ros
  tf2_geometry_msgs
  pose_utils
)
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})

install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()