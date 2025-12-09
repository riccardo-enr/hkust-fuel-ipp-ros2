# ROS 1 to ROS 2 Migration Status

## Project: FUEL - Fast UAV Exploration

This document tracks the progress of porting the FUEL project from ROS 1 to ROS 2.

## Overall Progress

**Packages Total:** 25  
**Completed:** 24 packages (96%)  
**In Progress:** 1 package (4%)  

### Phase Status

| Phase | Status | Progress |
|-------|--------|----------|
| 1. Nodelet to Component Migration | ✅ Complete | 1/1 packages |
| 2. Message Package Migration | ✅ Complete | 4/4 packages |
| 3. Package Manifest Updates | ✅ Complete | 25/25 packages |
| 4. Documentation Updates | ✅ Complete | README + MIGRATION_STATUS + SOURCE_MIGRATION_GUIDE |
| 5. CMakeLists.txt Migration | ✅ Nearly Complete | 24/25 packages (96%) |
| 6. Source Code Migration | 🔄 In Progress | 6 files migrated |
| 7. Launch File Migration | ⏳ Pending | 0 packages |
| 8. Testing & Validation | ⏳ Pending | - |

## Source Code Migration Progress

**Files Migrated (6):**
1. **poscmd_2_odom.cpp** - Position command to odometry converter
2. **map_recorder.cpp** - Map recording utility
3. **map_publisher.cpp** - Map publishing utility
4. **click_map.cpp** - Interactive map generation
5. **exploration_node.cpp** - Main exploration FSM node
6. **fast_planner_node.cpp** - Fast planner main node

**Patterns Established:**
- Global state → Class-based design (inherit from `rclcpp::Node`)
- Message includes: `/msg/` subdirectory, `.hpp` extension
- Publishers/Subscribers use `SharedPtr` and `std::bind`
- Parameters: `declare_parameter()` + `get_parameter()` pattern
- Rate loops → Timers (`create_wall_timer()`)
- Logging: `ROS_INFO` → `RCLCPP_INFO(this->get_logger(), ...)`

See `SOURCE_MIGRATION_GUIDE.md` for detailed migration patterns and examples.

## Detailed Package Status

### ✅ Fully Migrated Packages (24)

**Core Planning Libraries (10):**
1. **bspline** - B-spline representation with messages
2. **bspline_opt** - B-spline optimization with NLopt
3. **path_searching** - Kinodynamic A* and topology PRM
4. **plan_env** - SDF mapping and environment
5. **active_perception** - Frontier finding and trajectory visibility
6. **poly_traj** - Polynomial trajectory
7. **traj_utils** - Planning visualization utilities
8. **lkh_tsp_solver** - TSP solver
9. **exploration_manager** - Exploration FSM and manager
10. **plan_manage** - Planner management library

**Message Packages (4):**
11. **quadrotor_msgs** - Custom quadrotor messages (main package, rosidl)
12. **bspline** - (includes Bspline.msg)
13. **multi_map_server** - (includes map messages)
14. **multi_map_server/quadrotor_msgs** - (duplicate, marked for consolidation)

**Simulator Packages (7):**
15. **so3_control** - SO3 control component (nodelet → component)
16. **so3_quadrotor_simulator** - Quadrotor dynamics
17. **so3_disturbance_generator** - Disturbance generation
18. **map_generator** - Map generation and recording
19. **local_sensing** - Depth/PCL rendering
20. **poscmd_2_odom** - Position command converter
21. **multi_map_server** - Multi-map visualization

**Utility Packages (6):**
22. **cmake_utils** - CMake utilities
23. **pose_utils** - Pose utilities with Armadillo
24. **uav_utils** - Header-only UAV utilities
25. **odom_visualization** - Odometry visualization
26. **waypoint_generator** - Waypoint generation

### 🔄 Partially Migrated Packages (1)

**Pending CMakeLists.txt:**
1. **rviz_plugins** - RViz visualization plugins (requires Qt5/RViz2 specific updates)

Note: rviz_plugins is complex and requires specialized RViz2 and Qt5 knowledge. It can be addressed separately or considered optional for initial ROS 2 port.

## Remaining Work

### 1. CMakeLists.txt Migration (1 package)

**rviz_plugins** - This package requires:
- Qt5 (instead of Qt4) configuration for RViz2
- RViz2-specific plugin API updates
- Can be considered optional for initial port or addressed separately

### 2. Source Code Migration (ALL packages with C++ code - PRIMARY FOCUS)

This is the next major phase. For each C++ source file:
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
