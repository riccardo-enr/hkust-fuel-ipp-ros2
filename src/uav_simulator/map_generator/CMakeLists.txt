cmake_minimum_required(VERSION 3.8)
project(map_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler optimization flags for Jetson (Release mode)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED) # Not needed for C++ compilation
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# --- Global Includes and Definitions ---
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# CRITICAL: Ensures Eigen vectorization matches PCL's expectations
# Prevents segfaults on ARM64/Jetson
add_definitions(${PCL_DEFINITIONS})

# --- Executables ---

# 1. Random Forest
add_executable(random_forest src/random_forest_sensing.cpp)
ament_target_dependencies(random_forest
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(random_forest 
  ${PCL_LIBRARIES} 
  OpenMP::OpenMP_CXX
)

# 2. Empty World (Your MPPI map generator)
add_executable(empty_world src/empty_world.cpp)
ament_target_dependencies(empty_world
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(empty_world 
  ${PCL_LIBRARIES} 
  OpenMP::OpenMP_CXX
)

# 3. Map Recorder
add_executable(map_recorder src/map_recorder.cpp)
ament_target_dependencies(map_recorder
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(map_recorder ${PCL_LIBRARIES})

# 4. Map Publisher
add_executable(map_pub src/map_publisher.cpp)
ament_target_dependencies(map_pub
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(map_pub ${PCL_LIBRARIES})

# 5. Click Map
add_executable(click_map src/click_map.cpp)
ament_target_dependencies(click_map
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(click_map ${PCL_LIBRARIES})

# --- Install ---
install(TARGETS 
  random_forest
  empty_world 
  map_recorder 
  map_pub 
  click_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude copyright/flake8 checks if they are annoying during dev
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
