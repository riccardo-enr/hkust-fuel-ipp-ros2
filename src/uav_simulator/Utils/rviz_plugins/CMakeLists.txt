cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(multi_map_server REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Eigen3 REQUIRED)

# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

# Include directories
include_directories(
  src
  ${Eigen3_INCLUDE_DIRS}
)

# Source files
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/probmap_display.cpp
  src/aerialmap_display.cpp
  src/multi_probmap_display.cpp
)

# Library
add_library(${PROJECT_NAME} ${SOURCE_FILES})

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  multi_map_server
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${Eigen3_LIBRARIES}
)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()