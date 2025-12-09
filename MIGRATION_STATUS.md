# ROS 1 to ROS 2 Migration Status

## Project: FUEL - Fast UAV Exploration

This document tracks the progress of porting the FUEL project from ROS 1 to ROS 2.

## Overall Progress

**Packages Total:** 25  
**Completed:** 5 packages  
**In Progress:** 20 packages  

### Phase Status

| Phase | Status | Progress |
|-------|--------|----------|
| 1. Nodelet to Component Migration | ✅ Complete | 1/1 packages |
| 2. Message Package Migration | ✅ Complete | 3/3 packages |
| 3. Package Manifest Updates | ✅ Complete | 25/25 packages |
| 4. Documentation Updates | ✅ Complete | README updated |
| 5. CMakeLists.txt Migration | 🔄 In Progress | 4/25 packages |
| 6. Source Code Migration | ⏳ Pending | 0/25 packages |
| 7. Launch File Migration | ⏳ Pending | 0 packages |
| 8. Testing & Validation | ⏳ Pending | - |

## Detailed Package Status

### ✅ Fully Migrated Packages (5)

1. **so3_control** - Nodelet converted to component, package.xml and CMakeLists.txt updated
2. **quadrotor_msgs** - Message package fully migrated to rosidl
3. **cmake_utils** - Utility package migrated
4. **bspline** - Message and library package migrated
5. **multi_map_server** - Package.xml updated (CMakeLists.txt pending)

### 🔄 Partially Migrated Packages (20)

All of the following packages have updated package.xml files but require CMakeLists.txt updates and source code migration:

#### Fuel Planner Packages
- **active_perception**
- **bspline_opt**
- **exploration_manager** (has launch files to convert)
- **path_searching**
- **plan_env**
- **plan_manage** (has launch files to convert)
- **poly_traj**
- **traj_utils**
- **lkh_tsp_solver**

#### UAV Simulator Packages
- **pose_utils**
- **uav_utils**
- **odom_visualization**
- **waypoint_generator**
- **rviz_plugins**
- **map_generator**
- **local_sensing**
- **poscmd_2_odom**
- **so3_disturbance_generator**
- **so3_quadrotor_simulator** (has launch files to convert)

## Remaining Work

### 1. CMakeLists.txt Migration (21 packages)

Each CMakeLists.txt file needs to be updated to:
- Replace `find_package(catkin ...)` with `find_package(ament_cmake REQUIRED)`
- Update `find_package()` calls for ROS 2 packages (roscpp → rclcpp, etc.)
- Remove `catkin_package()` and replace with `ament_package()`
- Update message generation to use `rosidl_generate_interfaces()`
- Add proper `install()` commands for targets and files
- Update `ament_target_dependencies()` instead of manual `target_link_libraries()`
- Add test dependencies with `ament_lint_auto`

### 2. Source Code Migration (All packages with C++ code)

For each C++ source file:
- Replace `#include <ros/ros.h>` with `#include <rclcpp/rclcpp.hpp>`
- Replace `ros::NodeHandle` with `rclcpp::Node::SharedPtr` or inherit from `rclcpp::Node`
- Update publishers: `ros::Publisher` → `rclcpp::Publisher<>::SharedPtr`
- Update subscribers: `ros::Subscriber` → `rclcpp::Subscription<>::SharedPtr`
- Update callback signatures to use `::SharedPtr` instead of `::ConstPtr`
- Replace `ros::Time::now()` with `this->now()` or `node->now()`
- Replace `ros::Duration` with `rclcpp::Duration`
- Replace parameter API:
  - `nh.param()` → `this->declare_parameter()` and `this->get_parameter()`
  - `nh.getParam()` → `this->get_parameter()`
- Replace logging:
  - `ROS_INFO()` → `RCLCPP_INFO(this->get_logger(), ...)`
  - `ROS_WARN()` → `RCLCPP_WARN(this->get_logger(), ...)`
  - `ROS_ERROR()` → `RCLCPP_ERROR(this->get_logger(), ...)`
- Update TF:
  - `#include <tf/transform_datatypes.h>` → `#include <tf2/utils.h>`
  - `tf::getYaw()` → `tf2::getYaw()` or use tf2::Matrix3x3
  - `tf::TransformListener` → `tf2_ros::Buffer` and `tf2_ros::TransformListener`
- Update message includes:
  - `#include <package/MessageType.h>` → `#include <package/msg/message_type.hpp>`
  - e.g., `quadrotor_msgs/SO3Command.h` → `quadrotor_msgs/msg/so3_command.hpp`

### 3. Launch File Migration (3+ packages with launch files)

For packages with XML launch files:
- Create Python launch files in `launch/` directory
- Convert XML syntax to Python using `launch`, `launch_ros` APIs
- Update node declarations:
  ```python
  from launch_ros.actions import Node
  Node(
      package='package_name',
      executable='executable_name',
      name='node_name',
      parameters=[...],
      remappings=[...]
  )
  ```
- Remove nodelet manager references (components can be loaded differently in ROS 2)
- Convert parameter files from YAML to ROS 2 format if needed
- Install launch files in CMakeLists.txt:
  ```cmake
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  ```

### 4. Testing & Validation

Once migration is complete:
1. Build with `colcon build` in workspace root
2. Source the workspace: `source install/setup.bash`
3. Test individual nodes
4. Test full system integration
5. Verify sensor data flow
6. Test exploration functionality
7. Update README with final ROS 2 instructions

## Migration Pattern Examples

### Package.xml Pattern
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>package_name</name>
  <version>0.0.0</version>
  <description>...</description>
  <maintainer email="...">...</maintainer>
  <license>...</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <!-- other dependencies -->
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt Pattern (Basic)
```cmake
cmake_minimum_required(VERSION 3.8)
project(package_name)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# other packages...

# Targets
add_library(lib_name src/file.cpp)
ament_target_dependencies(lib_name rclcpp ...)

add_executable(node_name src/node.cpp)
ament_target_dependencies(node_name rclcpp ...)
target_link_libraries(node_name lib_name)

# Install
install(TARGETS lib_name node_name
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(lib_name)
ament_export_dependencies(rclcpp)

ament_package()
```

## Key Challenges

1. **Large codebase:** 25 packages with complex dependencies
2. **Launch files:** Multiple XML launch files need Python conversion
3. **TF dependencies:** Many packages use TF which requires careful conversion to TF2
4. **Custom messages:** Multiple message packages that need rosidl migration
5. **Nodelet architecture:** Nodelet-based design needs component conversion

## Next Steps

1. Continue CMakeLists.txt migration for remaining packages
2. Begin source code migration for core packages
3. Test building individual packages
4. Convert launch files
5. End-to-end integration testing

## References

- [ROS 2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
- [ament_cmake User Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [Composition in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
- [rosidl Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)

---

**Last Updated:** 2024-12-09  
**Migration Started:** 2024-12-09
