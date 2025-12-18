# MPPI Controller Test Plan

**Objective**: Validate the integration of the MPPI (Model Predictive Path Integral) controller using the existing UAV simulator and visualization tools.

## Prerequisites
- **Simulator Build**: Ensure the simulator packages (`so3_quadrotor_simulator`, `so3_control`, `map_generator`, etc.) are built and sourced.
  ```bash
  colcon build --packages-select so3_quadrotor_simulator so3_control map_generator odom_visualization local_sensing_node plan_bringup plan_manage
  source install/setup.bash
  ```
- **MPPI Package**: The MPPI controller package must be built (or stubbed/mocked if in early dev).

## Phase 1: Baseline Verification (`so3_control`)
Before testing MPPI, confirm the simulator environment is functional with the default `so3_control`.

### 1. Launch Simulator
Run the existing simulator launch file which brings up the physics, map, and default controller.
```bash
ros2 launch plan_bringup simulator.launch.py
```
**Verification Results (2025-12-18)**:
- **Status**: Verified.
- **Outcome**: Simulator nodes (`quadrotor_simulator_so3`, `so3_control`, `map_generator`) launch successfully. Drone responds to `PositionCommand` messages on `/planning/pos_cmd` by moving as expected.

### 2. Visualize (RViz2)
Launch RViz2 with the existing configuration to see the drone and map.
```bash
ros2 run rviz2 rviz2 -d src/fuel_planner/plan_manage/config/traj.rviz
```
**Verification Results (2025-12-18)**:
- **Status**: Verified.
- **Outcome**: RViz2 configuration `traj.rviz` has been updated to ROS 2 (using `rviz_default_plugins`). Plugins load correctly and subscribe to topics.

## Phase 2: MPPI Integration Strategy

To test MPPI, we will replace the `so3_control` component in the launch loop.

### 1. Create MPPI Launch Configuration
Create a new launch file (e.g., `mppi_simulator.launch.py`) or modify `simulator.launch.py` to accept a `controller` argument.

**Proposed Change (`simulator.launch.py`):**
Add a launch argument to select the controller.
```python
DeclareLaunchArgument('controller_type', default_value='so3') # Options: 'so3', 'mppi'
```
Condition the `ComposableNode` for the controller based on this argument.

**MPPI Node Spec**:
- **Package**: `mppi_control` (placeholder name)
- **Plugin/Executable**: `mppi_control::MPPIControlComponent` or standalone node.
- **Inputs**: 
  - `/state_ukf/odom` (Odometry)
  - `/planning/pos_cmd` (Target/Reference)
- **Outputs**:
  - `motors` (Motor speeds) or `so3_cmd` (Attitude/Thrust) depending on MPPI output level.

### 2. Manual Testing Loop
1. **Launch with MPPI**:
   ```bash
   ros2 launch plan_bringup simulator.launch.py controller_type:=mppi
   ```
   *(Or modify the launch file manually to load the MPPI node instead of `so3_control`)*.

2. **Send Commands**:
   Publish a dummy target pose to `/planning/pos_cmd` to trigger the controller.
   ```bash
   ros2 topic pub --once /planning/pos_cmd quadrotor_msgs/msg/PositionCommand "{...}"
   ```

3. **Monitor**:
   - Check `ros2 topic echo /motors` or `/so3_cmd`.
   - Watch RViz for stability/trajectory tracking.

## Phase 3: Advanced Visualization & Metrics

- **Visualization**: Use `rviz_plugins` to visualize the predicted paths (rollouts) of the MPPI controller.
  - Topic: `/mppi/rollouts` (MarkerArray or Path)
- **Metrics**: Measure tracking error and computation time.
