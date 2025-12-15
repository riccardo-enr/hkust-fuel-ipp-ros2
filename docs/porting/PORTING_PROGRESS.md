# ROS 1 to ROS 2 Porting Progress

**Last Updated:** Monday, December 15, 2025 05:30 PM

## 1. Executive Summary

**Project:** FUEL - Fast UAV Exploration
**Goal:** Port entire codebase from ROS 1 (Noetic) to ROS 2 (Jazzy).

| Metric           | Status      | Details                                                                       |
| :--------------- | :---------- | :---------------------------------------------------------------------------- |
| **Packages**     | 24/25       | All but `src/uav_simulator/Utils/multi_map_server/quadrotor_msgs` are format 3; that duplicate message pkg is still Catkin/format 1. Newly added `swarmtal_msgs` already uses `rosidl_default_generators`. |
| **Build System** | 23/25       | `rviz_plugins` remains Catkin and `multi_map_server` targets still rely on ROS 1 APIs despite the ament build. |
| **Source Code**  | In Progress | Outstanding ROS 1 nodes: `multi_map_server` visualization tools, `local_sensing/pcl_render_node`, and legacy `plan_manage` tools/tests (`traj_server`, etc.). |
| **Launch Files** | Pending     | To be addressed after source code migration.                                  |

## 2. Detailed Status

### ✅ Fully Migrated Packages (Build & Source)

The following packages build and run on ROS 2 without ROS 1 dependencies:

**Core Utilities & Messages:**
1.  `poscmd_2_odom`
2.  `waypoint_generator`
3.  `quadrotor_msgs` *(primary copy under `src/uav_simulator/Utils/quadrotor_msgs`)*
4.  `swarmtal_msgs`
5.  `pose_utils`
6.  `uav_utils`
7.  `cmake_utils`
8.  `odom_visualization`

**Simulation & Control:**

9.   `so3_control`
10.  `so3_disturbance_generator`
11.  `map_generator`
12.  `plan_env`
13.  `path_searching`
14.  `plan_manage` *(ROS 2 entry point only; legacy ROS 1 tools remain, see “Pending Source”)*
15.  `active_perception`
16.  `exploration_manager`
17.  `poly_traj`
18.  `so3_quadrotor_simulator`
19.  `local_sensing`
    - [x] `pointcloud_render_node.cpp`
    - [x] `depth_render_node.cpp`

### 🔄 Build System Ready (Source Pending)

These packages have ROS 2 manifests/CMake but still contain ROS 1 executables or headers that must be ported:

- `plan_manage`
  - `traj_server.cpp`, `traj_server_backup.cpp`, and legacy tests under `test/` still use `ros::` APIs.
- `local_sensing`
  - `pcl_render_node.cpp` (CUDA-based) is still a ROS 1 node.
- `multi_map_server`
  - Visualization binary and headers (`Map2D/Map3D`) depend on `roscpp`, `tf`, and `ros::Time`.
- `multi_map_server/quadrotor_msgs`
  - Duplicate Catkin message package; remove or migrate to ROS 2 (still Catkin build).

### ⏳ Pending Build System

- `rviz_plugins` (Requires Qt5/RViz2 specific updates and full ament port).

### ⏳ Pending Build System

- `rviz_plugins` (Requires Qt5/RViz2 specific updates).

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
- [x] **1.6 swarmtal_msgs**
    - [x] `DroneOnboardCommand.msg`

### Step 2: Core Planning Libraries (Priority 2)
*Goal: Migrate computational cores. Minimal ROS dependency.*

- [x] **2.1 bspline**
    - [x] `non_uniform_bspline.cpp`
- [x] **2.2 poly_traj**
    - [x] `polynomial_traj.cpp`
    - [x] `traj_generator.cpp` (ROS 2 node + `swarmtal_msgs` dependency)
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
    - [ ] `traj_server.cpp` & tools/tests (ROS 1)
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
    - [ ] `pcl_render_node.cpp` (CUDA ROS 1 path)

### Step 5: Finalization
- [ ] **5.1 Launch Files:** Convert all XML launch files to Python.
- [ ] **5.2 Integration Test:** Build all and run full simulation.
