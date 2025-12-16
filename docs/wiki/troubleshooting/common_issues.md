# Common Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| `ros2 launch plan_manage traj_server.launch.py` hangs | `install/setup.bash` not sourced | Source `install/setup.bash` and relaunch |
| Missing `quadrotor_msgs` | Workspace not rebuilt after dependency change | Run `colcon build --packages-select plan_manage quadrotor_msgs` |

Add rows as new issues arise.
