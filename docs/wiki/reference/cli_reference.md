# CLI Reference

List frequently used commands with short explanations.

## Launch Commands

| Command | Purpose |
|---------|---------|
| `ros2 launch plan_bringup traj_server.launch.py world:=office` | Bring up trajectory server with office world |
| `ros2 launch plan_bringup fuel_pipeline.launch.py controller_type:=so3 world:=office exploration:=true` | Full pipeline with SO3 controller and exploration |
| `ros2 launch plan_bringup fuel_pipeline.launch.py controller_type:=mppi world:=office exploration:=true` | Full pipeline with MPPI thrust+quaternion controller |
| `ros2 launch plan_bringup fuel_pipeline.launch.py controller_type:=mppi_acc world:=office exploration:=true` | Full pipeline with MPPI acceleration controller |

## Topic Inspection

| Command | Purpose |
|---------|---------|
| `ros2 topic echo /traj_server/command` | Inspect trajectory server commands |
| `ros2 topic echo /odom` | Inspect UAV odometry |
| `ros2 topic echo /planning/pos_cmd` | Inspect reference trajectory from planner |
| `ros2 topic echo /so3_cmd` | Inspect SO3 control commands |
| `ros2 topic hz /so3_cmd` | Check control command rate |

## Visualization

| Command | Purpose |
|---------|---------|
| `ros2 run tf2_tools view_frames` | Visualize TF tree |
| `rviz2 -d src/fuel_planner/plan_bringup/rviz/fuel.rviz` | Launch RViz with FUEL config |

## Development

| Command | Purpose |
|---------|---------|
| `colcon build --packages-select mppi_control` | Build MPPI controller package |
| `ros2 run mppi_control mppi_acc_node --ros-args --params-file src/fuel_planner/plan_bringup/config/mppi.yaml` | Run MPPI acceleration node with config |
| `ros2 run mppi_control mppi_tq_node --ros-args --params-file src/fuel_planner/plan_bringup/config/mppi.yaml` | Run MPPI thrust+quaternion node with config |

Expand as new workflows stabilize.
