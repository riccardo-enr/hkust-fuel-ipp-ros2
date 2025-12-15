# multi_map_server ROS 2 Port Plan

## 1. Baseline & Inventory
- Enumerate all sources under `src/uav_simulator/Utils/multi_map_server/` (Map2D, Map3D, helper libs, duplicate `quadrotor_msgs`).
- Note the existing ROS 1 dependencies (`roscpp`, `tf`, `tf::TransformBroadcaster`, latched publishers, etc.) and how the Catkin message package is referenced.
- Attempt `colcon build --packages-select multi_map_server` (after sourcing ROS 2) to capture current build failures for reference.

## 2. Message Package Cleanup
- Decide whether the duplicate Catkin `multi_map_server/quadrotor_msgs` contains unique definitions; if so, port them into the primary ROS 2 `quadrotor_msgs`, otherwise remove the duplicate.
- Update includes across `multi_map_server` to consume the ROS 2 message package and drop Catkin-only dependencies.

## 3. Core Node Conversion
- For every executable (Map2D, Map3D, auxiliary tools):
  - Wrap logic in an `rclcpp::Node`; replace `ros::NodeHandle`, global publishers/subscribers, and `ros::spin()` loops with executor-driven timers/callbacks.
  - Substitute `ros::Time`/`ros::Duration` with `node->now()` / `rclcpp::Time`.
  - Migrate TF usage to `tf2_ros::Buffer`, `tf2_ros::TransformListener`, and `tf2_ros::TransformBroadcaster`.
  - Replace `nh.param`/`getParam` with `declare_parameter` + `get_parameter`, and configure QoS profiles for latched topics (e.g., use `transient_local()` for map publishers).

## 4. Visualization & RViz2 Interop
- Inspect visualization outputs (markers, occupancy grids, custom RViz displays); ensure message types and frame IDs follow ROS 2 conventions.
- Update any RViz-specific hooks (OGRE/Qt) to RViz2 plugin APIs if custom displays exist, or confirm existing visualization topics work in RViz2.

## 5. Build System Migration
- Rewrite `CMakeLists.txt` to pure `ament_cmake`: `find_package(rclcpp nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen ... )`.
- Add each executable via `add_executable` + `ament_target_dependencies` and install into `lib/multi_map_server`.
- Update `package.xml` (format 3) to list the ROS 2 dependencies and remove Catkin macros or `roscpp` buildtool requirements.

## 6. Testing & Validation
- Rebuild with `colcon build --packages-select multi_map_server`.
- Create or update a ROS 2 launch file that brings up Map2D/Map3D against a mock map or recorded bag; verify topics (`ros2 topic list`), TF frames, and RViz2 visualization.
- Document new parameters or behavioral changes discovered during testing.

## 7. Documentation & Tracking
- Update `docs/porting/PORTING_PROGRESS.md` once the package is fully ROS 2 compliant, removing it from the “Build System Ready” list.
- If the duplicate `quadrotor_msgs` is removed, add a migration note (README or release notes) so downstream users know to depend on the ROS 2 `quadrotor_msgs` package instead.
