# ROS 1 to ROS 2 Porting Progress

**Last Updated:** Tuesday, December 16, 2025 11:15 AM

## 1. Executive Summary

**Project:** FUEL - Fast UAV Exploration
**Goal:** Port entire codebase from ROS 1 (Noetic) to ROS 2 (Jazzy).

| Metric           | Status      | Details                                                                       |
| :--------------- | :---------- | :---------------------------------------------------------------------------- |
| **Packages**     | 25/25       | All packages now use package format 3; the duplicate ROS 1 `multi_map_server/quadrotor_msgs` copy has been removed. |
| **Build System** | 24/25       | Only `rviz_plugins` remains Catkin; `multi_map_server` builds natively with ament after the visualization node rewrite. |
| **Source Code**  | In Progress | Outstanding ROS 1 nodes: `multi_map_server` visualization tools and `local_sensing/pcl_render_node`; runtime nodes/launches for `plan_manage` now run on ROS 2 (Dec 16, 2025). |
| **Launch Files** | Pending     | To be addressed after source code migration.                                  |

## 2. Detailed Status

### ✅ Fully Migrated Packages (Build & Source)

The following packages build and run on ROS 2 without ROS 1 dependencies:

**Core Utilities & Messages:**
1.  `poscmd_2_odom`
2.  `waypoint_generator`
3.  `quadrotor_msgs` *(primary copy under `src/uav_simulator/Utils/quadrotor_msgs`)*
4.  `pose_utils`
5.  `uav_utils`
6.  `cmake_utils`
7.  `odom_visualization`

**Simulation & Control:**

8.  `so3_control`
9.  `so3_disturbance_generator`
10. `map_generator`
11. `plan_env`
12. `path_searching`
13. `poly_traj` *(traj_generator node + ROS 2 launch/test harness)*
14. `plan_manage` *(FSM entry point + traj_server ROS 2 nodes; benchmark/test helpers still ROS 1, see “Pending Source”)*
15. `active_perception`
16. `exploration_manager`
17. `so3_quadrotor_simulator`
18. `local_sensing`
    - [x] `pointcloud_render_node.cpp`
    - [x] `depth_render_node.cpp`
18. `multi_map_server` *(Map2D/Map3D libraries and visualization node now rclcpp/tf2-only)*

### 🔄 Build System Ready (Source Pending)

These packages have ROS 2 manifests/CMake but still contain ROS 1 executables or headers that must be ported:

- `plan_manage`
  - Legacy benchmarking/test utilities under `test/` (`process_msg*.cpp`, `rotation.cpp`, etc.) still use `ros::` APIs and must be rewritten or replaced.
- `local_sensing`
  - `pcl_render_node.cpp` (CUDA-based) is still a ROS 1 node.

> Legacy note: The duplicate Catkin `src/uav_simulator/Utils/multi_map_server/quadrotor_msgs` package has been deleted; depend on the ROS 2 `quadrotor_msgs` instead.

### ⏳ Pending Build System

- `rviz_plugins` (Requires Qt5/RViz2 specific updates and full ament port).

---

## 3. Migration Guide & Patterns

**Reference for converting C++ source code.**

| Concept      | ROS 1                         | ROS 2                                        |
| :----------- | :---------------------------- | :------------------------------------------- |
| **Headers**  | `#include <ros/ros.h>`        | `#include <rclcpp/rclcpp.hpp>`               |
| **Messages** | `#include <pkg/Msg.h>`        | `#include <pkg/msg/msg.hpp>`                 |
| **Node**     | `ros::NodeHandle nh;`         | Inherit `rclcpp::Node`                       |
| **Logging**  | `ROS_INFO(...)`               | `RCLCPP_INFO(this->get_logger(), ...)`       |
| **Time**     | `ros::Time::now()`            | `this->now()`                                |
| **Params**   | `nh.param("key", val, def)`   | `val = this->declare_parameter("key", def);` |
| **Rate**     | `ros::Rate r(10); r.sleep();` | `create_wall_timer(100ms, callback);`        |
| **Pub**      | `pub.publish(msg)`            | `pub->publish(msg)` (Use SharedPtr)          |
| **Sub**      | `ConstPtr` in callback        | `SharedPtr` in callback                      |

**Key Rules:**
1.  **Class-based Nodes:** Avoid global state. Wrap logic in a class inheriting from `rclcpp::Node`.
2.  **Timers:** Replace `while(ros::ok())` loops with wall timers.
3.  **TF2:** Replace `tf` with `tf2` and `tf2_ros`. Use `tf2::toMsg` and `tf2::fromMsg`.
4.  **Parameters:** Always declare parameters before getting them.

---

## 4. Step-by-Step Plan

### Step 1: Simple Utility Packages (Priority 1)
*Goal: Migrate independent utilities to establish patterns.*

- [x] **1.1 poscmd_2_odom**
    - [x] `poscmd_2_odom.cpp`
- [x] **1.2 waypoint_generator**
    - [x] `waypoint_generator.cpp`
    - [x] `sample_waypoints.h`
- [x] **1.3 odom_visualization**
    - [x] `odom_visualization.cpp` (Complex TF & Visualization)
- [x] **1.4 map_generator**
    - [x] `map_recorder.cpp`
    - [x] `map_publisher.cpp`
    - [x] `click_map.cpp`
    - [x] `random_forest_sensing.cpp`
- [x] **1.5 so3_disturbance_generator**
    - [x] `so3_disturbance_generator.cpp`

### Step 2: Core Planning Libraries (Priority 2)
*Goal: Migrate computational cores. Minimal ROS dependency.*

- [x] **2.1 bspline**
    - [x] `non_uniform_bspline.cpp`
- [x] **2.2 poly_traj**
    - [x] `polynomial_traj.cpp`
    - [x] `traj_generator.cpp` *(ROS 2 node + launch + CI test harness)*
- [x] **2.3 path_searching**
    - [x] `astar.cpp`
    - [x] `kinodynamic_astar.cpp`
    - [x] `topo_prm.cpp`
- [x] **2.4 bspline_opt**
    - [x] `bspline_optimizer.cpp`
- [x] **2.5 plan_env**
    - [x] `sdf_map.cpp`
    - [x] `obj_predictor.cpp`
    - [x] `edt_environment.cpp`
    - [x] `map_ros.cpp` (Complex)
- [x] **2.6 traj_utils**
    - [x] `planning_visualization.cpp`
    - [x] `process_msg.cpp`

### Step 3: Planning Management (Priority 3)
*Goal: Migrate the high-level Logic and FSMs.*

- [ ] **3.1 plan_manage**
    - [x] `fast_planner_node.cpp` (Entry point)
    - [x] `kino_replan_fsm.cpp`
    - [x] `topo_replan_fsm.cpp`
    - [x] `planner_manager.cpp`
    - [x] `traj_server.cpp` & `traj_server_backup.cpp` (ROS 2 nodes + launch/config)
    - [x] ROS 2 runtime artifacts landed (`config/traj_server.yaml`, `config/fast_planner.yaml`, `launch/traj_server.launch.py`, `launch/fast_planner.launch.py`), plus dependency fixes so `colcon build --packages-select plan_manage` succeeds as of Dec 16, 2025.
    - [x] Legacy benchmarking/test tools under `test/` (`process_msg*`, `rotation`, etc.) migrated to standalone ROS 2 nodes (Dec 16, 2025). *Note:* `compare_topo`, `opti_node`, and `test_collision_cost` now run in self-contained benchmarking modes that no longer rely on the old ROS 1 Fast Planner harness.
- [x] **3.2 active_perception**
    - [x] `frontier_finder.cpp`
    - [x] `perception_utils.cpp`
- [x] **3.3 exploration_manager**
    - [x] `exploration_node.cpp` (Entry point)
    - [x] `fast_exploration_fsm.cpp`
    - [x] `fast_exploration_manager.cpp`
    - [x] `expl_data.cpp`

### Step 4: Simulator Components (Priority 4)
*Goal: Physics and Rendering.*

- [x] **4.1 so3_control**
    - [x] `so3_control_component.cpp`
    - [x] `SO3Control.cpp`
- [x] **4.2 so3_quadrotor_simulator**
    - [x] `quadrotor_simulator_so3.cpp`
- [ ] **4.3 local_sensing**
    - [x] `pointcloud_render_node.cpp`
    - [x] `depth_render_node.cpp`
    - [x] `pcl_render_node.cpp` (CUDA path, ROS 2 w/ optional ENABLE_CUDA build)

### Step 5: Finalization
- [ ] **5.1 Launch Files:** Convert all XML launch files to Python.
- [ ] **5.2 Integration Test:** Build all and run full simulation.
