cmake_minimum_required(VERSION 3.5)
project(uav_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cmake_utils REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(uav_utils_test src/uav_utils_test.cpp)
  target_include_directories(uav_utils_test PUBLIC include)
  ament_target_dependencies(uav_utils_test rclcpp nav_msgs geometry_msgs)
  target_link_libraries(uav_utils_test Eigen3::Eigen)
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()