# Step 3 Plan – plan_manage / fast_planner & traj_server

## Objective

Finalize the migration of `plan_manage` (the high-level planning logic and FSMs), ensuring `fast_planner_node` and `traj_server` are fully ROS 2–native, buildable, and launchable.

## Current State

- **Source Code:** Core nodes (`fast_planner_node`, `traj_server`, `kino_replan_fsm`, etc.) have been largely ported to `rclcpp`.
- **Dependencies:** Build failures indicate missing dependencies in `package.xml` (specifically `quadrotor_msgs`, `geometry_msgs`, `cv_bridge`).
- **Legacy Components:** Several test/benchmark files (`process_msg*.cpp`, `rotation.cpp`) in `test/` still use ROS 1 APIs.
- **Launch/Config:** No ROS 2 launch files or parameter configurations exist yet.

## Tasks

- [x] **Dependency Resolution**
  - Added the missing runtime dependencies (`quadrotor_msgs`, `geometry_msgs`, `cv_bridge`) to `plan_manage/package.xml` and propagated the include paths + NLopt linkage through `CMakeLists.txt`.
  - `colcon build --packages-select plan_manage` now completes cleanly (Dec 16, 2025).

- [ ] **Legacy Code Cleanup**
  - Identify non-critical legacy tests/benchmarks in `test/`.
  - Disable or port them based on immediate utility. If they are not critical for the runtime, exclude them from the ROS 2 build to unblock the package.

- [x] **Runtime Configuration**
  - Added `config/fast_planner.yaml` covering planner/FSM/map parameters plus the companion `launch/fast_planner.launch.py` with remaps + optional waypoint generator.
  - `launch/traj_server.launch.py` already exists and both launches now ingest the new YAML parameters via `ros2 launch`.

- [ ] **Integration Verification**
  - Verify the nodes start up correctly and advertise/subscribe to the expected topics using `ros2 node info` and `ros2 topic list`.
  - Validate the `traj_server` effectively communicates with `poly_traj` and `so3_control` (mocked if necessary).

## Deliverables

- Fully building `plan_manage` package.
- Functional ROS 2 launch files and YAML configurations.
- Successful startup of `fast_planner_node` and `traj_server` in a ROS 2 environment.
