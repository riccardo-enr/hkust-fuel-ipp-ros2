cmake_minimum_required(VERSION 3.5)
project(local_sensing_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pcl_render_node src/depth_render_node.cpp)
ament_target_dependencies(pcl_render_node
  rclcpp
  pcl_conversions
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(pcl_render_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(TARGETS pcl_render_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()