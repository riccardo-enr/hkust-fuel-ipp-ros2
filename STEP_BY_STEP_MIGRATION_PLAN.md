# ROS 1 to ROS 2 Migration - Step-by-Step Plan

## Overview

This document provides a systematic, step-by-step plan for completing the ROS 1 to ROS 2 migration. The work is organized by package complexity and dependencies.

## Current Status

### ✅ Completed (Phase 1: Build System)
- **Package manifests**: 25/25 packages (100%)
- **CMakeLists.txt**: 24/25 packages (96%)
  - Only rviz_plugins remains (optional, requires specialized Qt5/RViz2 knowledge)

### 🔄 In Progress (Phase 2: Source Code)
- **Files migrated**: 11/~100+ files
- **Patterns established**: ✅ All core patterns documented

## Step-by-Step Migration Order

### Step 1: Simple Utility Packages (Priority 1) 🎯

These packages have minimal dependencies and simple node structures.

#### 1.1 poscmd_2_odom ✅ COMPLETE
- [x] poscmd_2_odom.cpp - Position command to odometry converter

#### 1.2 odom_visualization
- [ ] odom_visualization.cpp - Odometry visualization with Armadillo

#### 1.3 waypoint_generator ✅ COMPLETE
- [x] waypoint_generator.cpp - Waypoint generation with TF2
- [x] sample_waypoints.h - Waypoint patterns

#### 1.4 map_generator (Partially Complete)
- [x] map_recorder.cpp - Map recording
- [x] map_publisher.cpp - Map publishing
- [x] click_map.cpp - Interactive map generation
- [ ] random_forest.cpp - Forest map generation
- [ ] random_forest_sensing.cpp - Sensing simulation

#### 1.5 so3_disturbance_generator
- [ ] so3_disturbance_generator.cpp - Disturbance generation for testing

---

### Step 2: Core Planning Libraries (Priority 2)

These are library files with minimal ROS dependencies (mostly parameters and logging).

#### 2.1 bspline (Partially Complete)
- [x] non_uniform_bspline.cpp - B-spline implementation
- [ ] polynomial_traj.cpp - Polynomial trajectory

#### 2.2 poly_traj
- [ ] polynomial_traj.cpp - Polynomial trajectory utilities

#### 2.3 path_searching (Partially Complete)
- [x] astar.cpp - A* search algorithm
- [x] kinodynamic_astar.cpp - Kinodynamic A* search
- [ ] topo_prm.cpp - Topology PRM

#### 2.4 bspline_opt
- [ ] bspline_optimizer.cpp - B-spline optimization (uses timers, NodeHandle)
- [ ] gradient_descent_optimizer.cpp - Gradient descent

#### 2.5 plan_env
- [ ] sdf_map.cpp - Signed distance field map
- [ ] obj_predictor.cpp - Object prediction
- [ ] edt_environment.cpp - EDT environment
- [ ] map_ros.cpp - Map ROS interface (complex: subscribers, NodeHandle)

#### 2.6 traj_utils
- [ ] planning_visualization.cpp - Planning visualization

---

### Step 3: Planning Management (Priority 3)

Main planning nodes and FSM implementations. Entry points are done, FSM classes need migration.

#### 3.1 plan_manage (Entry Complete)
- [x] fast_planner_node.cpp - Planner selection node
- [ ] kino_replan_fsm.cpp - Kinodynamic replanning FSM (NodeHandle, timers, subscribers)
- [ ] topo_replan_fsm.cpp - Topology replanning FSM
- [ ] planner_manager.cpp - Planner manager

#### 3.2 active_perception
- [ ] frontier_finder.cpp - Frontier detection
- [ ] perception_utils.cpp - Perception utilities

#### 3.3 exploration_manager (Entry Complete)
- [x] exploration_node.cpp - Exploration initialization
- [ ] fast_exploration_fsm.cpp - Exploration FSM (complex: NodeHandle, many subscribers/publishers)
- [ ] fast_exploration_manager.cpp - Exploration management
- [ ] expl_data.cpp - Exploration data structures

---

### Step 4: Simulator Components (Priority 4)

Simulation components with physics and rendering.

#### 4.1 so3_control ✅ COMPLETE
- [x] so3_control_component.cpp - Component architecture
- [x] SO3Control.cpp - SO3 control implementation

#### 4.2 so3_quadrotor_simulator
- [ ] quadrotor_simulator_so3.cpp - Quadrotor dynamics simulation (complex)

#### 4.3 local_sensing
- [ ] pointcloud_render_node.cpp - Point cloud rendering
- [ ] depth_render.cu - CUDA depth rendering (optional, very complex)

#### 4.4 pose_utils ✅ COMPLETE
- Header-only library (no migration needed)

#### 4.5 uav_utils ✅ COMPLETE
- Header-only library (no migration needed)

---

### Step 5: Optional/Advanced Packages (Priority 5)

These can be deferred or are very complex.

#### 5.1 multi_map_server
- [ ] multi_map_visualization.cpp - Multi-map visualization

#### 5.2 lkh_tsp_solver
- Mostly C code with minimal ROS interface
- May need minimal changes

#### 5.3 rviz_plugins (DEFERRED)
- Requires Qt5 and RViz2 API knowledge
- Can be addressed separately

---

## Phase 3: Launch Files (Future)

After source code migration:
- [ ] Convert XML launch files to Python
- [ ] Update node declarations
- [ ] Update parameter loading

---

## Phase 4: Integration Testing (Future)

- [ ] Build all packages with `colcon build`
- [ ] Resolve compilation errors
- [ ] Test individual nodes
- [ ] Test full system integration

---

## Migration Patterns Reference

All patterns are documented in `SOURCE_MIGRATION_GUIDE.md`:

1. **Node Structure**: Global → Class-based (`rclcpp::Node`)
2. **Headers**: `ros/ros.h` → `rclcpp/rclcpp.hpp`
3. **Messages**: `/msg/` subdirectory, `.hpp` extension
4. **Publishers/Subscribers**: `SharedPtr` with `std::bind`
5. **Parameters**: `declare_parameter()` + `get_parameter()`
6. **Timers**: `ros::Rate` → `create_wall_timer()`
7. **Time**: `ros::Time::now()` → `this->now()`
8. **TF**: `tf::*` → `tf2::*` with `tf2_geometry_msgs`
9. **Logging**: `ROS_*` → `RCLCPP_*(get_logger(), ...)`
10. **Library Classes**: `ros::NodeHandle&` → `rclcpp::Node::SharedPtr`

---

## Progress Tracking

| Category | Total | Complete | Percentage |
|----------|-------|----------|------------|
| Package Manifests | 25 | 25 | 100% ✅ |
| CMakeLists.txt | 25 | 24 | 96% ✅ |
| Simple Utilities | ~10 | 7 | 70% 🔄 |
| Planning Libraries | ~15 | 3 | 20% 🔄 |
| Planning Management | ~8 | 2 | 25% 🔄 |
| Simulators | ~5 | 2 | 40% 🔄 |
| **Overall** | **~100** | **~43** | **~43%** |

---

## Next Actions

**Current Focus**: Step 1 - Complete Simple Utility Packages

**Next File**: odom_visualization.cpp

**Strategy**: 
1. Complete all of Step 1 (simple utilities)
2. Move to Step 2 (core libraries with minimal ROS)
3. Tackle Step 3 (complex FSMs and managers)
4. Address Step 4 (simulators)
5. Optional Step 5 as needed

This organized approach ensures:
- Dependencies are satisfied
- Patterns are reused
- Progress is measurable
- Testing can begin early with completed packages
