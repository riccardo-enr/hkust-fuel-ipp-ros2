# ROS 1 to ROS 2 (Jazzy) Porting Progress

This document tracks the progress of porting the `hkust-fuel-ipp-ros2` repository from ROS 1 (Noetic) to ROS 2 (Jazzy).

## Environment Setup

The development environment has been updated to use ROS 2 Jazzy. This involved modifying the `.devcontainer/Dockerfile` to base on `ros:jazzy` and install `ros-jazzy-desktop`.

## Completed Packages (ROS 2 Ported)

The following packages have been successfully ported to ROS 2 and compile without errors in the Jazzy environment:

-   **`quadrotor_msgs`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool, and replaced `message_generation`/`message_runtime` with `rosidl_default_generators`/`rosidl_default_runtime`. Added `std_msgs` dependency.
    -   `CMakeLists.txt`: Rewritten to use `ament_cmake` and `rosidl_generate_interfaces`. Explicitly linked generated typesupport libraries.
    -   `.msg` files (e.g., `Gains.msg`, `SO3Command.msg`): Renamed fields (e.g., `Kp` to `kp`, `kR` to `kr`) to adhere to ROS 2 message field naming conventions (snake_case).
    -   `.cpp` files (`encode_msgs.cpp`, `decode_msgs.cpp`): Updated message type namespaces (e.g., `quadrotor_msgs::SO3Command` to `quadrotor_msgs::msg::SO3Command`). Modified `encode_msgs.cpp` to replace `header.seq` with a static counter due to `seq` being removed from `std_msgs/Header` in ROS 2. Added `#include <cstring>`.
    -   `.h` files (`encode_msgs.h`, `decode_msgs.h`): Updated message type includes (e.g., `quadrotor_msgs/SO3Command.h` to `quadrotor_msgs/msg/so3_command.hpp`).

-   **`cmake_utils`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Removed `roscpp` and `cmake_modules` dependencies.
    -   `CMakeLists.txt`: Rewritten to use `ament_cmake`. CMake utility files (`arch.cmake`, `color.cmake`) and `cmake_modules` directory (`FindEigen.cmake`, `FindGSL.cmake`, `FindmvIMPACT.cmake`) are now installed and exported via `ament_package` with `CONFIG_EXTRAS`. A `cmake_utils-extras.cmake.in` file was created and processed with `configure_file(COPYONLY)` to correctly reference installed paths.

-   **`uav_utils`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp`/`rospy` with `rclcpp`. Added `nav_msgs`, `geometry_msgs`, `eigen` dependencies.
    -   `CMakeLists.txt`: Rewritten to use `ament_cmake`. Uses `ament_add_gtest` for tests.
    -   `include/uav_utils/converters.h`: Updated message type includes and namespaces (e.g., `<nav_msgs/Odometry.h>` to `<nav_msgs/msg/odometry.hpp>`, `nav_msgs::OdometryConstPtr` to `nav_msgs::msg::Odometry::ConstSharedPtr`).
    -   `include/uav_utils/utils.h`: Replaced `#include <ros/ros.h>` with `#include <cassert>` and `#include <sstream>`. `ROS_ASSERT_MSG` replaced with `assert` statements.
    -   `src/uav_utils_test.cpp`: Updated ROS 1 message pointers (`nav_msgs::OdometryPtr`) to ROS 2 shared pointers (`std::make_shared<nav_msgs::msg::Odometry>`).

-   **`pose_utils`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Removed `roscpp` dependency.
    -   `CMakeLists.txt`: Rewritten to use `ament_cmake`. Builds `pose_utils` library and links `Armadillo`.

-   **`odom_visualization`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp` with `rclcpp`, `tf` with `tf2`/`tf2_ros`/`tf2_geometry_msgs`.
    -   `CMakeLists.txt`: Rewritten for `ament_cmake`.
    -   `src/odom_visualization.cpp`: Completely rewritten as an `rclcpp::Node` class. Converted ROS 1 publishers/subscribers/parameters to ROS 2 equivalents. Updated message types and TF2 usage.

-   **`so3_control`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `nodelet` with `rclcpp_components`.
    -   `CMakeLists.txt`: Rewritten to build as a `rclcpp_components` plugin.
    -   `src/SO3Control.cpp`: Removed `#include <ros/ros.h>`.
    -   `src/so3_control_nodelet.cpp`: Ported to an `rclcpp::Node` component, including changes for publishers, subscribers, parameters, and TF2 conversions.

-   **`so3_quadrotor_simulator`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp` with `rclcpp`.
    -   `CMakeLists.txt`: Rewritten for `ament_cmake`.
    -   `src/dynamics/Quadrotor.cpp`: Removed `#include <ros/ros.h>`.
    -   `src/quadrotor_simulator_so3.cpp`: Converted to an `rclcpp::Node` class. Updated publishers, subscribers, parameters, time handling, and message types. Corrected field access for `quadrotor_msgs::msg::SO3Command` (e.g., `cmd->kR[0]` to `cmd->kr[0]`).

-   **`local_sensing`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced ROS 1 specific dependencies with ROS 2 equivalents (`rclcpp`, `pcl_conversions`, `cv_bridge`, `image_transport`).
    -   `CMakeLists.txt`: Rewritten for `ament_cmake` (non-CUDA branch). Set C++ standard to C++17.
    -   `src/depth_render_node.cpp`: Ported to an `rclcpp::Node` class. Updated publishers, subscribers, parameters, time handling, and message types. Changed `#include <cv_bridge/cv_bridge.h>` to `#include <cv_bridge/cv_bridge.hpp>`. Removed `backward.hpp` dependency.

-   **`map_generator`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp` with `rclcpp`. Added `pcl_conversions`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `eigen`, and `pcl` dependencies.
    -   `CMakeLists.txt`: Rewritten for `ament_cmake`. Uses `ament_target_dependencies` for all executables.
    -   All source files (`random_forest_sensing.cpp`, `map_recorder.cpp`, `map_publisher.cpp`, `click_map.cpp`): Ported to `rclcpp::Node` classes. Updated message type includes (e.g., `<nav_msgs/msg/odometry.hpp>`, `<sensor_msgs/msg/point_cloud2.hpp>`). Converted publishers, subscribers, parameters, and timers to ROS 2 equivalents.

-   **`poscmd_2_odom`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp` with `rclcpp`. Added `eigen` dependency.
    -   `CMakeLists.txt`: Rewritten for `ament_cmake`. Updated C++ standard to C++17.
    -   `src/poscmd_2_odom.cpp`: Converted to `rclcpp::Node` class. Updated message includes to ROS 2 format (`<nav_msgs/msg/odometry.hpp>`, `<quadrotor_msgs/msg/position_command.hpp>`). Replaced timer loop with `create_wall_timer`. Converted publishers/subscribers to ROS 2 API.

-   **`waypoint_generator`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp` with `rclcpp`, `tf` with `tf2`/`tf2_geometry_msgs`. Added `geometry_msgs`, `nav_msgs`, and `eigen` dependencies.
    -   `CMakeLists.txt`: Rewritten for `ament_cmake`. Updated C++ standard to C++17.
    -   `src/sample_waypoints.h`: Updated message type includes to ROS 2 format. Created helper function `createQuaternionMsgFromYaw()` to replace `tf::createQuaternionMsgFromYaw()` using TF2.
    -   `src/waypoint_generator.cpp`: Completely rewritten as an `rclcpp::Node` class. Converted all ROS 1 APIs to ROS 2 (publishers, subscribers, parameters, time handling). Replaced TF1 functions (`tf::getYaw()`, `tf::createQuaternionMsgFromYaw()`) with TF2 equivalents. Updated message types to use `::msg::` namespace. Fixed timestamp comparisons by converting to `rclcpp::Time`.

-   **`so3_disturbance_generator`**:
    -   `package.xml`: Updated to format 3, `ament_cmake` buildtool. Replaced `roscpp` with `rclcpp`. Removed `dynamic_reconfigure` dependency (not needed in ROS 2).
    -   `CMakeLists.txt`: Rewritten for `ament_cmake`. Updated C++ standard to C++17. Removed dynamic_reconfigure build dependencies.
    -   `src/so3_disturbance_generator.cpp`: Completely rewritten as an `rclcpp::Node` class. Replaced `dynamic_reconfigure` with ROS 2 parameter declarations using `declare_parameter()` and `get_parameter()`. Converted all message types to ROS 2 format (`geometry_msgs::msg::Vector3`, `nav_msgs::msg::Odometry`, etc.). Replaced ROS 1 time handling (`ros::Time`, `toSec()`) with ROS 2 equivalents (`rclcpp::Time`, `seconds()`). Converted publishers/subscribers to ROS 2 API. Replaced spin loop with `create_wall_timer()` for disturbance generation at 100 Hz.


## Next Steps
1.  Continue porting the remaining `uav_simulator` packages (`so3_disturbance_generator`, `multi_map_server`, `rviz_plugins`).
2.  Port the `fuel_planner` packages.

## Status Update (2025-12-09)

- **Progress snapshot:** Most core packages have been ported to ROS 2 (Jazzy) and there are no reported compile errors for the packages listed in the "Completed Packages" section. The workspace devcontainer has been updated to use `ros:jazzy`.
- **What's done:** Consistent `package.xml`/`CMakeLists.txt` migrations to `ament_cmake`; message renames and `::msg::` namespace updates; TF1→TF2 conversions; node conversions to `rclcpp` and `rclcpp_components`; sensible fixes for `Header.seq` removal and shared pointer usage.
- **Remaining high-level work:**
    - Port the `uav_simulator` subpackages: `so3_disturbance_generator`, `multi_map_server`, and `rviz_plugins` (the latter is highest effort because rviz2 plugin API differs significantly from rviz1).
    - Port the `fuel_planner` packages (likely the largest group; requires careful dependency and behavior testing).
    - Verify and update CI/build scripts and run full workspace builds and runtime smoke tests.

## Risks and Notes

- **rviz plugins:** Porting `rviz_plugins` is high-risk and often requires reimplementation against the `rviz2` API and C++ ABI differences — treat as a separate effort with its own design/PR.
- **API/Field changes:** Renamed message fields (snake_case) and removal of `Header.seq` can cause runtime mismatches if any remaining nodes expect ROS 1 semantics — add integration tests to catch these.
- **Hidden dependencies:** Some packages may still rely on ROS 1 utilities or third-party libraries that need ROS 2-compatible wrappers (check `cmake_utils` exports and `Find*` modules).

## Package Inventory and Dependency Map

### Remaining UAV Simulator Packages (3 packages)

1. **`so3_disturbance_generator`** (Low complexity)
   - Dependencies: `roscpp`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `pose_utils` (✓ ported), `dynamic_reconfigure` (needs ROS 2 equivalent)
   - Effort: Small — straightforward node conversion
   - Blocker: `dynamic_reconfigure` → migrate to ROS 2 parameters/services

2. **`multi_map_server`** (Medium complexity)
   - Dependencies: `roscpp`, `visualization_msgs`, `geometry_msgs`, `tf` → `tf2`, `nav_msgs`, `std_srvs`, `laser_geometry`, `pose_utils` (✓ ported), `message_generation`, `quadrotor_msgs` (✓ ported)
   - Effort: Medium — has custom messages/services, server node
   - Blocker: Custom message definitions need rosidl migration

3. **`rviz_plugins`** (High complexity - **DEFER**)
   - Dependencies: `rviz` → `rviz2`, `roscpp`, `multi_map_server`, `qtbase5-dev`
   - Effort: High — rviz2 plugin API is completely different from rviz1
   - Blocker: Depends on `multi_map_server`; requires plugin API rewrite
   - **Recommendation:** Defer to separate PR or skip if not critical

### Remaining FUEL Planner Packages (10 packages)

**Dependency order (build from bottom up):**

**Tier 1 - Foundation (no internal dependencies):**
1. **`plan_env`** — Environment representation
   - Dependencies: `roscpp`, `rospy`, `std_msgs`
   - Effort: Small-Medium

2. **`poly_traj`** — Polynomial trajectory utilities
   - Dependencies: `roscpp`, `std_msgs`, `swarmtal_msgs` (⚠️ **external dependency - may not exist**)
   - Effort: Small
   - **Risk:** `swarmtal_msgs` not in workspace — may need to remove or stub

3. **`lkh_tsp_solver`** — TSP solver utility
   - Dependencies: `roscpp` only
   - Effort: Small

**Tier 2 - Core algorithms (depend on Tier 1):**
4. **`bspline`** — B-spline utilities
   - Dependencies: `roscpp`, `rospy`, `std_msgs`, `plan_env`
   - Effort: Small-Medium

5. **`path_searching`** — Path search algorithms (A*, etc.)
   - Dependencies: `roscpp`, `rospy`, `std_msgs`, `plan_env`
   - Effort: Small-Medium

**Tier 3 - Advanced (depend on Tier 2):**
6. **`active_perception`** — Active perception utilities
   - Dependencies: `roscpp`, `rospy`, `std_msgs`, `plan_env`, `bspline`, `path_searching`
   - Effort: Medium

7. **`bspline_opt`** — B-spline optimization
   - Dependencies: `roscpp`, `rospy`, `std_msgs`, `plan_env`, `active_perception`
   - Effort: Medium

**Tier 4 - Integration (depend on Tier 3):**
8. **`traj_utils`** — Trajectory utilities
   - Dependencies: `bspline`, `bspline_opt`, `path_searching`, `poly_traj`, `roscpp`, `std_msgs`
   - Effort: Medium

9. **`plan_manage`** — Plan manager
   - Dependencies: `poly_traj`, `plan_env`, `path_searching`, `bspline`, `bspline_opt`, `active_perception`, `traj_utils`, `roscpp`, `std_msgs`
   - Effort: Medium-Large

**Tier 5 - Top-level (integration package):**
10. **`exploration_manager`** — Exploration manager (top-level node)
    - Dependencies: All other fuel_planner packages + `quadrotor_msgs` (✓ ported)
    - Effort: Large — main node with many integrations

## Prioritized Action Plan (short-term)

### Phase 1: Quick Wins (UAV Simulator)
1. ✅ **Inventory remaining packages** — COMPLETED
2. **Port `so3_disturbance_generator`** — straightforward node, replace `dynamic_reconfigure` with ROS 2 parameters. (Est: 2-3 hours)
3. **Port `multi_map_server`** — migrate custom messages, update server node. (Est: 4-6 hours)
4. **Skip/Defer `rviz_plugins`** — defer to separate effort or omit if not critical for core functionality. (Defer)

### Phase 2: FUEL Planner Foundation (Tier 1)
5. **Port `plan_env`** — core environment package. (Est: 4-6 hours)
6. **Port `lkh_tsp_solver`** — minimal dependencies. (Est: 1-2 hours)
7. **Port `poly_traj`** — check `swarmtal_msgs` dependency; stub or remove if needed. (Est: 2-3 hours)

### Phase 3: FUEL Planner Core (Tier 2-3)
8. **Port `bspline`** and **`path_searching`** in parallel (depend on `plan_env`). (Est: 6-8 hours)
9. **Port `active_perception`** and **`bspline_opt`**. (Est: 6-8 hours)

### Phase 4: FUEL Planner Integration (Tier 4-5)
10. **Port `traj_utils`**. (Est: 4-6 hours)
11. **Port `plan_manage`**. (Est: 6-8 hours)
12. **Port `exploration_manager`** (final integration node). (Est: 8-10 hours)

### Phase 5: Verification
13. **Run full `colcon build`** — resolve all compile-time errors. (Est: 2-4 hours)
14. **Update CI and devcontainer** configs for Jazzy. (Est: 1-2 hours)
15. **Run runtime smoke tests** — launch representative scenarios. (Est: 2-3 hours)
16. **Update documentation** — finalize `PORTING_PROGRESS.md` and package READMEs. (Est: 1-2 hours)

**Total estimated effort:** ~50-70 hours for complete port

## Suggested Immediate Next Step

✅ **Inventory completed** — See detailed package dependency map above.
✅ **`so3_disturbance_generator` ported and building** — Successfully converted to ROS 2 with parameter-based configuration.

**Status:** `multi_map_server` port is in progress. Completed:
- `package.xml` updated to ament_cmake and rosidl
- `CMakeLists.txt` updated with rosidl_generate_interfaces
- `Map2D.h` updated (TF1→TF2, nav_msgs types)
- `multi_map_visualization.cc` rewritten as rclcpp::Node

**Remaining work for `multi_map_server`:**
- `Map3D.h` needs ros::Time → rclcpp::Time conversion and TF2 updates (lines 319, 321, 328, 509-510)
- Build and test the package
- Estimated: 1-2 hours

**Next recommended action:** Complete `multi_map_server` port, then proceed with FUEL planner packages starting with Tier 1 (`plan_env`, `lkh_tsp_solver`, `poly_traj`).

---
_Updated on 2025-12-09 — contact the maintainer or open a PR per package for review and CI checks._
