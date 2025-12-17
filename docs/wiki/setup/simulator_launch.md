# Simulator & Launch Recipes

Document reproducible launch sequences that stitch together the simulator, planners, and visualization tools.

## Example Flow
1. `ros2 launch so3_quadrotor_simulator simulator.launch.py world:=office`
2. `ros2 launch plan_bringup traj_server.launch.py world:=office`
3. `ros2 launch plan_bringup fast_planner.launch.py use_sim_time:=true`

The planner-specific Python launch files now live in the `plan_bringup` package so they can be invoked from a single bringup workspace share.

Consider adding troubleshooting sections for TF mismatches or QoS incompatibilities.
