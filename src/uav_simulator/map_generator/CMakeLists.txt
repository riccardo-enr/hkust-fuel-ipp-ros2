cmake_minimum_required(VERSION 3.5)
project(map_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(random_forest src/random_forest_sensing.cpp)
target_link_libraries(random_forest
  ${PCL_LIBRARIES}
)
ament_target_dependencies(random_forest
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  Eigen3
)

add_executable(map_recorder src/map_recorder.cpp)
target_link_libraries(map_recorder
  ${PCL_LIBRARIES}
)
ament_target_dependencies(map_recorder
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  Eigen3
)

add_executable(map_pub src/map_publisher.cpp)
target_link_libraries(map_pub
  ${PCL_LIBRARIES}
)
ament_target_dependencies(map_pub
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  Eigen3
)

add_executable(click_map src/click_map.cpp)
target_link_libraries(click_map
  ${PCL_LIBRARIES}
)
ament_target_dependencies(click_map
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  Eigen3
)

install(TARGETS random_forest map_recorder map_pub click_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()