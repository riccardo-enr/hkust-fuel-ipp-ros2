# Step 2 Plan – poly_traj / traj_generator

## Objective

Finish the core-planning migration by ensuring `poly_traj`'s `traj_generator` node is fully ROS 2–native and production ready.

## Current State

- Source already uses `rclcpp`, ROS 2 message headers, QoS profiles, and timers.
- Package builds via ament (`poly_traj/CMakeLists.txt`) and exports the shared library plus executable.
- Integration and testing coverage for the ROS 2 node is minimal; launch files still ROS 1.

## Tasks

- [x] **Code Audit**
  - Traj node moved into `include/poly_traj/traj_generator_node.hpp` for reuse/tests; confirmed only ROS 2 APIs remain.
  - Parameter defaults verified and documented.
- [x] **Runtime Validation**
  - Added ROS 2 launch file (`launch/traj_generator.launch.py`) plus default YAML (`config/traj_generator_sim.yaml`) for sim bring-up.
  - Manual rosbag2 playback still recommended in the field; hook documented in README.
- [x] **Documentation & Samples**
  - Authored `src/fuel_planner/poly_traj/README.md` with parameter table, launch instructions, and testing notes.
  - YAML example provides topic remaps + sim-time toggle.
- [x] **Testing**
  - Added `ament_add_gtest` target exercising the node end-to-end with synthetic odometry and ensuring command publication.
  - Integrated into `colcon test --packages-select poly_traj` (auto-skips when RMW transports are unavailable in sandboxed CI).
- [x] **Upstream Consumers**
  - No additional dependencies required; downstream planners already consume the `traj_generator` executable name that remains unchanged.

## Deliverables

- Updated source/tests/docs per tasks above.
- Evidence of `colcon test --packages-select poly_traj` passing on ROS 2 Jazzy.
