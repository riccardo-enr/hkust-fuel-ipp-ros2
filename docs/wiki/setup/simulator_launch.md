# Simulator & Launch Recipes

Document reproducible launch sequences that stitch together the simulator, planners, and visualization tools.

## Example Flow
1. `ros2 launch so3_quadrotor_simulator simulator.launch.py world:=office`
2. `ros2 launch plan_bringup traj_server.launch.py world:=office`
3. `ros2 launch plan_bringup fast_planner.launch.py use_sim_time:=true`

## Combined Pipeline
Use `ros2 launch plan_bringup fuel_pipeline.launch.py` to bring up the simulator, trajectory server, and fast planner from a single entry point. Override any of the shared launch arguments (e.g., `map_type`, `planner_mode`, `launch_waypoint_generator`, `use_backup`) to tune the behavior for your run.

The planner-specific Python launch files now live in the `plan_bringup` package so they can be invoked from a single bringup workspace share.

Consider adding troubleshooting sections for TF mismatches or QoS incompatibilities.
