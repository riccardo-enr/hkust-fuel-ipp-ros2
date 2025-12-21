# Plan: Enable Autonomous Map Exploration

## Objective
Enable the `exploration_manager` to autonomously guide the drone to explore the unknown map.

## Background
- **Package:** `exploration_manager`
- **Node:** `exploration_node`
- **Launch File:** `fuel_pipeline.launch.py` controls the mode via the `exploration` argument.
- **Current State:** `exploration` is set to `false` (default). `fast_planner_node` handles manual goals.

## Plan

### 1. Analyze Configuration
- Check `src/fuel_planner/exploration_manager/config/exploration.yaml`.
- Ensure `exploration_node` parameters are compatible with the current simulator setup.

### 2. Verify TSP Solver Path
- The `exploration.launch.py` sets `exploration.tsp_dir` using `FindPackageShare("lkh_tsp_solver")`.
- We need to ensure `lkh_tsp_solver` is installed and has the necessary resources (`par` files, etc.).
- Check `.gitignore`: `fuel_planner/utils/lkh_tsp_solver/resource/*.par` was listed. We need to make sure they exist in the *install* directory.

### 3. Launch Test
Run `fuel_pipeline.launch.py` with `exploration:=true`.

**Command:**
```bash
ros2 launch plan_bringup fuel_pipeline.launch.py exploration:=true
```

### 4. Verification Steps
1.  **Node Startup:** Confirm `exploration_node` starts without crashing.
2.  **Triggering:** Autonomous exploration often requires a trigger to start (e.g., a "Start" waypoint or service call).
    - `exploration_node` usually listens to `waypoint_generator/waypoints`.
    - `waypoint_generator` is launched with `waypoint_type:=point` in `exploration.launch.py`.
    - Sending a goal to `/move_base_simple/goal` might trigger the first exploration step.
3.  **Behavior:**
    - Drone should generate a global tour (TSP) or local frontier goals.
    - Drone should move and cover the map.
    - Check topics: `/planning/bspline`, `/position_cmd`.

### 5. Visualization Setup (New)
*Goal: Show the explored portion of the map in RViz.*
1.  **Map Topic:** Identify the topic publishing the known map. `MapROS` publishes `/sdf_map/occupancy_all` (global known cells) if `show_all_map` is true.
2.  **Configuration:**
    - Check `fast_planner.yaml` for `map_ros/show_all_map`. Set to `true` if needed.
    - Update RViz config (`traj_viz_dev.rviz` or new `exploration.rviz`) to include a PointCloud2 display subscribing to `/sdf_map/occupancy_all`.
    - Topic: `/sdf_map/occupancy_all` (or `occupancy_local` for just the sensor range).

### 6. Version Control
*Goal: Manage changes in a feature branch.*
1.  Create branch `feat/autonomous-exploration` from `test/fuel-ros2`.
    ```bash
    git checkout -b feat/autonomous-exploration
    ```
2.  Commit changes (yaml updates, launch fixes) to this branch.

### 7. Troubleshooting
- If the drone doesn't move:
    - Check if `exploration_node` is publishing goals/paths.
    - Check if `exploration_node` received the initial trigger.
    - Check logs for "TSP" or "Frontier" errors.

## Action Items
1.  **Git:** Create/Checkout `feat/autonomous-exploration`.
2.  **Config:** Inspect `exploration.yaml` & `fast_planner.yaml` (for map visualization).
3.  **Resources:** Verify `lkh_tsp_solver` resources.
4.  **Execute:** Launch with `exploration:=true`.
5.  **Trigger:** Trigger exploration.
