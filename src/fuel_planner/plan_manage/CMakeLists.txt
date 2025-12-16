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
find_package(nav_msgs REQUIRED)
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
find_library(NLOPT_LIBRARY
  NAMES nlopt
  PATHS /usr/local/lib /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)
if(NOT NLOPT_LIBRARY)
  find_library(NLOPT_LIBRARY
    NAMES nlopt
  )
endif()
if(NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "Failed to locate the NLopt shared library required by plan_manage.")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
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
  nav_msgs
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
  nav_msgs
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

add_executable(traj_server
  src/traj_server.cpp
)

ament_target_dependencies(traj_server
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  bspline
  active_perception
)

add_executable(traj_server_backup
  src/traj_server_backup.cpp
)

ament_target_dependencies(traj_server_backup
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  bspline
  poly_traj
  path_searching
  active_perception
)

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

set(plan_manage_third_party
  ${bspline_LIBRARIES}
  ${NLOPT_LIBRARY}
  ${PCL_LIBRARIES}
)

foreach(target_name IN ITEMS plan_manage fast_planner_node traj_server traj_server_backup)
  target_link_libraries(${target_name} ${plan_manage_third_party})
endforeach()

add_executable(process_msg test/process_msg.cpp)
ament_target_dependencies(process_msg
  rclcpp
  geometry_msgs
  quadrotor_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(process_msg ${plan_manage_third_party})

add_executable(process_msg2 test/process_msg2.cpp)
ament_target_dependencies(process_msg2
  rclcpp
  geometry_msgs
  visualization_msgs
)

add_executable(pos_vel_acc test/pos_vel_acc.cpp)
ament_target_dependencies(pos_vel_acc
  rclcpp
  visualization_msgs
  bspline
)
target_link_libraries(pos_vel_acc ${plan_manage_third_party})

add_executable(rotation test/rotation.cpp)
ament_target_dependencies(rotation
  rclcpp
  nav_msgs
  visualization_msgs
)

add_executable(compare_topo test/compare_topo.cpp)
ament_target_dependencies(compare_topo
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)
target_link_libraries(compare_topo ${PCL_LIBRARIES})

add_executable(opti_node test/opti_node.cpp)
ament_target_dependencies(opti_node
  rclcpp
  visualization_msgs
  geometry_msgs
  poly_traj
)

add_executable(test_collision_cost test/test_collision_cost.cpp)
ament_target_dependencies(test_collision_cost
  rclcpp
  sensor_msgs
  visualization_msgs
  plan_env
  bspline
  bspline_opt
  traj_utils
  pcl_conversions
)
target_link_libraries(test_collision_cost ${plan_manage_third_party})

# Install
install(TARGETS 
  plan_manage
  fast_planner_node 
  traj_server
  traj_server_backup
  process_msg
  process_msg2
  pos_vel_acc
  rotation
  compare_topo
  opti_node
  test_collision_cost
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

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(plan_manage)
ament_export_dependencies(rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs geometry_msgs quadrotor_msgs poly_traj plan_env path_searching bspline bspline_opt traj_utils active_perception cv_bridge pcl_conversions)

ament_package()
