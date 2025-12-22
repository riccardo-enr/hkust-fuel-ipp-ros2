cmake_minimum_required(VERSION 3.8)
project(mppi_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable CUDA
enable_language(CUDA)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)

# CUDA dependencies
find_package(CUDAToolkit REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${CUDAToolkit_INCLUDE_DIRS}
)

add_library(mppi_control_component SHARED
  src/mppi_control_node.cpp
  src/mppi_kernels.cu
)

set_target_properties(mppi_control_component PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

# Use ament_target_dependencies first, it might use plain signatures
ament_target_dependencies(mppi_control_component
  rclcpp
  rclcpp_components
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  plan_env
)

# Then add CUDA libs. If ament_target_dependencies used plain, we must use plain.
# Usually ament_target_dependencies uses plain.
target_link_libraries(mppi_control_component
  CUDA::curand
  CUDA::cudart
)

target_compile_definitions(mppi_control_component
  PRIVATE "MPPI_CONTROL_CPP_BUILDING_DLL")

rclcpp_components_register_node(mppi_control_component
  PLUGIN "mppi_control::MPPIControlNode"
  EXECUTABLE mppi_control_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()