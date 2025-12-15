cmake_minimum_required(VERSION 3.8)
project(plan_manage)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(active_perception REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(plan_manage 
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)

ament_target_dependencies(plan_manage
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  cv_bridge
  pcl_conversions
)

target_include_directories(plan_manage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(fast_planner_node
  src/fast_planner_node.cpp 
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  test/local_explore_fsm.cpp
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)

ament_target_dependencies(fast_planner_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  cv_bridge
  pcl_conversions
)

# add_executable(traj_server src/traj_server.cpp)

# ament_target_dependencies(traj_server
#   rclcpp
#   std_msgs
#   geometry_msgs
#   quadrotor_msgs
# )

# add_executable(proc_msg test/process_msg.cpp)

# ament_target_dependencies(proc_msg
#   rclcpp
#   std_msgs
# )

# target_link_libraries(proc_msg
#   ${PCL_LIBRARIES}
# )

# add_executable(proc_msg2 test/process_msg2.cpp)

# ament_target_dependencies(proc_msg2
#   rclcpp
#   std_msgs
# )

# target_link_libraries(proc_msg2
#   ${PCL_LIBRARIES}
# )

# Install
install(TARGETS 
  plan_manage
  fast_planner_node 
  # traj_server
  # proc_msg
  # proc_msg2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(plan_manage)
ament_export_dependencies(rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs quadrotor_msgs poly_traj plan_env path_searching bspline bspline_opt traj_utils active_perception cv_bridge pcl_conversions)

ament_package()