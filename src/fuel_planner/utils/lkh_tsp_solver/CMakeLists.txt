cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# LKH upstream still relies on common symbols; keep GCC 13 happy.
add_compile_options($<$<COMPILE_LANGUAGE:C>:-fcommon>)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories( 
  include 
)

file(GLOB SRCS src/*.c)

add_library(lkh_tsp_solver 
  ${SRCS}
  src/lkh_interface.cpp
)

ament_target_dependencies(lkh_tsp_solver
  rclcpp
)

target_link_libraries(lkh_tsp_solver -lm)

target_include_directories(lkh_tsp_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lkh_tsp 
  ${SRCS}
)

ament_target_dependencies(lkh_tsp
  rclcpp
)

target_link_libraries(lkh_tsp -lm)

# Install
install(TARGETS lkh_tsp_solver lkh_tsp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(lkh_tsp_solver)
ament_export_dependencies(rclcpp)

ament_package()
