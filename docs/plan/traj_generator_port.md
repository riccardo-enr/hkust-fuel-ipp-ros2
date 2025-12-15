# traj_generator ROS 2 Port Plan

## Background

`poly_traj/src/traj_generator.cpp` is still a standalone ROS 1 node that:
- Subscribes to `/uwb_vicon_odom` for odometry.
- Publishes visualization markers for the generated polynomial trajectory and instantaneous state.
- Publishes `swarmtal_msgs::drone_onboard_command` commands (position/velocity/acceleration encoded as scaled integers).
- Builds a hard-coded closed-form minimum-jerk trajectory using `generateTraj` and the legacy `PolynomialTraj` helper from `plan_env/polynomial_traj.hpp`.
- Relies on global publishers, `ros::NodeHandle`, `ros::Time`/`ros::Duration`, `ros::spinOnce()` loops, and ROS 1 message headers (`nav_msgs/Odometry.h`, etc.).

No executable for this node is registered in the ROS 2 `CMakeLists.txt`, so the source currently never builds.

## Porting Goals

1. Rewrite `traj_generator.cpp` as an `rclcpp::Node` class (or `rclcpp::NodeOptions` entry point) with:
   - Subscription to `nav_msgs::msg::Odometry`.
   - Publishers for `visualization_msgs::msg::Marker` (trajectory + state) and `swarmtal_msgs::msg::DroneOnboardCommand`.
   - A wall timer to stream commands at 100 Hz instead of the manual `while(ros::ok())` loop.
2. Replace all ROS 1 headers/APIs (`ros/ros.h`, `ros::Time`, `ros::Duration`, `ros::Publisher`) with their ROS 2 equivalents.
3. Modernize helper functions (`displayPathWithColor`, `drawState`) to accept publishers/time stamps from the node (no globals).
4. Resolve the `generateTraj`/`PolynomialTraj` helpers:
   - Confirm whether `generateTraj` should come from `plan_env/polynomial_traj.hpp` (function currently missing) or replace it using `fast_planner::PolynomialTraj::waypointsTraj`.
   - Ensure we include the correct header (`plan_env/polynomial_traj.hpp`) instead of the non-existent `<traj_generator/polynomial_traj.hpp>`.
5. Add an executable target to `poly_traj/CMakeLists.txt` and link it against the `poly_traj` library plus required dependencies (Eigen, rclcpp, nav_msgs, visualization_msgs, geometry_msgs, swarmtal_msgs).
6. Update `package.xml`/`CMakeLists.txt` dependency lists (nav_msgs, visualization_msgs, geometry_msgs, swarmtal_msgs) so builds succeed.
7. Provide launch argument hooks for topic names/frame IDs if needed (optional, after ROS 2 skeleton works).

## Key Technical Considerations

- **Message API updates:** Replace includes with `#include <nav_msgs/msg/odometry.hpp>`, `#include <visualization_msgs/msg/marker.hpp>`, and, if available, the ROS 2 port of `swarmtal_msgs` (`swarmtal_msgs/msg/drone_onboard_command.hpp`). If that ROS 2 package does not exist we’ll need to decide whether to stub commands or guard compilation.
- **Time utilities:** Use `node->now()` / `node->create_wall_timer()` rather than `ros::Time::now()` / sleep loops.
- **Multithreading:** The ROS 1 code uses `ros::spinOnce()` inside a blocking loop. In ROS 2, the timer callback should live in the executor; avoid manual sleeps except for visualization throttling.
- **Trajectory helper parity:** ROS 1 code expects `PolynomialTraj` to expose `evaluate`, `evaluateVel`, and `evaluateAcc`. The ROS 2 library version (in `include/poly_traj/polynomial_traj.h`) currently only implements `evaluate(ts, k)` rather than separate `evaluateVel/Acc`. We must either add those helpers back (preferred) or adjust the node to call `evaluate(t, 1/2)` to get derivatives.
- **Dependency cleanup:** Remove the unused/incorrect header `#include <traj_generator/polynomial_traj.hpp>`; rely on `plan_env` or the local `poly_traj` headers instead.

## Work Plan

1. **Refactor Helpers and Node Skeleton**
   - Create a `TrajGeneratorNode` class inheriting from `rclcpp::Node`.
   - Move publishers/subscribers/timers into class members; replace global state with class fields (`odom_`, `have_odom_`).
   - Convert `displayPathWithColor` and `drawState` into member functions or lambdas using `rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr`.


2. **Port ROS APIs to ROS 2**
   - Update includes to ROS 2 message headers and `rclcpp`.
   - Replace `ros::Time`/`ros::Duration` with `rclcpp::Clock` and `rclcpp::Duration` (or `std::chrono`).
   - Replace `ros::spinOnce()` loop with a state machine backed by timers: wait for odometry in constructor (or via callback) and only start timer once `have_odom_` is true.

3. **Trajectory Generation Clean-up**
   - Fix the include path for `PolynomialTraj` and ensure we can compute position/velocity/acceleration (either by extending `PolynomialTraj` or using existing derivative evaluation API).
   - Audit the mysterious `generateTraj` helper; if it does not exist, implement or import the minimum-jerk generator (likely available in other branches) or re-use the `PolynomialTraj::waypointsTraj` implementation.

4. **Command Publishing Timer**
   - Implement a `create_wall_timer(10ms, ...)` callback that computes elapsed time from a stored start time, evaluates the trajectory, fills `swarmtal_msgs::msg::DroneOnboardCommand`, and publishes markers.
   - Guard against running past the trajectory duration by looping or holding the final waypoint.

5. **CMake & Package Updates**
   - Add `add_executable(traj_generator src/traj_generator.cpp)` and link it with `poly_traj` target plus `rclcpp`, `nav_msgs`, `visualization_msgs`, `geometry_msgs`, `swarmtal_msgs`.
   - Install the executable (`install(TARGETS traj_generator DESTINATION lib/${PROJECT_NAME})`).
   - Ensure `ament_target_dependencies` includes the added packages.

6. **Testing Plan**
   - Build `poly_traj` with `colcon build --packages-select poly_traj`.
   - Run `ros2 run poly_traj traj_generator` in a test environment with simulated odometry to verify markers and commands publish.
   - (Optional) Add a lightweight unit/integration test that instantiates the node and feeds fake odometry to confirm it starts the timer.

7. **Follow-up Items**
   - Parameterize waypoint list and command scaling (currently hard-coded).
   - Consider exposing the generated trajectory via `nav_msgs::msg::Path` or a service for other nodes.
