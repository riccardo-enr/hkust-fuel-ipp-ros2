# ROS 1 to ROS 2 Porting Progress

**Last Updated:** Monday, December 15, 2025

## 1. Executive Summary

**Project:** FUEL - Fast UAV Exploration
**Goal:** Port entire codebase from ROS 1 (Noetic) to ROS 2 (Jazzy).

| Metric           | Status      | Details                                                                       |
| :--------------- | :---------- | :---------------------------------------------------------------------------- |
| **Packages**     | 25/25       | All package manifests (`package.xml`) migrated.                               |
| **Build System** | 24/25       | `CMakeLists.txt` migrated for 96% of packages. (Only `rviz_plugins` pending). |
| **Source Code**  | In Progress | Ported path_searching, plan_env, and plan_manage.                             |
| **Launch Files** | Pending     | To be addressed after source code migration.                                  |

## 2. Detailed Status

### ✅ Fully Migrated Packages (Build & Source)

**Core Utilities & Messages:**
1.  `poscmd_2_odom` - Position command to odometry converter.
2.  `waypoint_generator` - Waypoint generation with TF2.
3.  `quadrotor_msgs` - Custom quadrotor messages.
4.  `pose_utils` - Pose utilities (Armadillo).
5.  `uav_utils` - Header-only utilities.
6.  `cmake_utils` - CMake utilities.
7.  `odom_visualization` - Odometry and trajectory visualization.

**Simulation & Control:**

8.  `so3_control` - SO3 control (migrated to Component).
9.  `so3_disturbance_generator` - Disturbance generation (migrated `dynamic_reconfigure`).
10. `map_generator` - Map generation suite (`map_recorder`, `map_publisher`, `click_map`, `random_forest_sensing`).
11. `plan_env`
    - `sdf_map.cpp`
    - `obj_predictor.cpp`
    - `edt_environment.cpp`
    - `map_ros.cpp` (Complex)
    - `obj_generator.cpp`
12. `path_searching`
    - `astar.cpp`
    - `kinodynamic_astar.cpp`
    - `topo_prm.cpp`
    - `astar2.cpp`
    - `astar2.cpp`
13. `plan_manage`
    - `fast_planner_node.cpp` (Entry point)
    - `kino_replan_fsm.cpp`
    - `topo_replan_fsm.cpp`
    - `planner_manager.cpp`


### 🔄 Build System Ready (Source Pending)

**Planning Libraries:**

- `bspline` (Partially source migrated)
- `bspline_opt`
- `active_perception`
- `poly_traj`
- `traj_utils`
- `lkh_tsp_solver`
- `exploration_manager` (Entry node migrated)

**Visualization & Simulation:**
- `multi_map_server`
- `local_sensing`
- `so3_quadrotor_simulator`

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

### Step 2: Core Planning Libraries (Priority 2)
*Goal: Migrate computational cores. Minimal ROS dependency.*

- [x] **2.1 bspline**
    - [x] `non_uniform_bspline.cpp`
- [x] **2.2 poly_traj**
    - [x] `polynomial_traj.cpp`
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

- [x] **3.1 plan_manage**
    - [x] `fast_planner_node.cpp` (Entry point)
    - [x] `kino_replan_fsm.cpp`
    - [x] `topo_replan_fsm.cpp`
    - [x] `planner_manager.cpp`
- [ ] **3.2 active_perception**
    - [ ] `frontier_finder.cpp`
    - [ ] `perception_utils.cpp`
- [ ] **3.3 exploration_manager**
    - [x] `exploration_node.cpp` (Entry point)
    - [ ] `fast_exploration_fsm.cpp`
    - [ ] `fast_exploration_manager.cpp`
    - [ ] `expl_data.cpp`

### Step 4: Simulator Components (Priority 4)
*Goal: Physics and Rendering.*

- [x] **4.1 so3_control**
    - [x] `so3_control_component.cpp`
    - [x] `SO3Control.cpp`
- [ ] **4.2 so3_quadrotor_simulator**
    - [ ] `quadrotor_simulator_so3.cpp`
- [ ] **4.3 local_sensing**
    - [ ] `pointcloud_render_node.cpp`
    - [ ] `depth_render.cu` (CUDA)

### Step 5: Finalization
- [ ] **5.1 Launch Files:** Convert all XML launch files to Python.
- [ ] **5.2 Integration Test:** Build all and run full simulation.