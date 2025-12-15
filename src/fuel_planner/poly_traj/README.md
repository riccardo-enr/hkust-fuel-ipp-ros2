# poly_traj

`poly_traj` provides polynomial trajectory utilities plus a ROS 2 `traj_generator` node that converts odometry into onboard waypoint commands and visualization markers.

## Parameters
| Name | Default | Description |
| --- | --- | --- |
| `odom_topic` | `/uwb_vicon_odom` | Odometry topic used to seed trajectories. |
| `traj_marker_topic` | `/traj_generator/traj_vis` | Marker topic for full planned path. |
| `state_marker_topic` | `/traj_generator/cmd_vis` | Marker topic for instantaneous velocity/acc vectors. |
| `command_topic` | `/drone_commander/onboard_command` | Output command topic (`swarmtal_msgs/DroneOnboardCommand`). |
| `frame_id` | `world` | Frame used for visualization messages. |
| `command_rate_hz` | `100.0` | Rate of command publishing while a trajectory is active. |

You can override any of the above via a parameter file; see `config/traj_generator_sim.yaml` for an example.

## Launching
```
ros2 launch poly_traj traj_generator.launch.py \
  odom_topic:=/sim/odom \
  command_topic:=/drone_commander/onboard_command \
  frame_id:=map
```
This launch file declares arguments for every exposed parameter plus `use_sim_time`. Combine it with `config/traj_generator_sim.yaml` if you want to keep overrides in YAML:
```
ros2 launch poly_traj traj_generator.launch.py params_file:=/path/to/traj_generator_sim.yaml
```

## Testing
Run the package tests (including the integration-style `traj_generator` check) with:
```
colcon test --packages-select poly_traj
```
The new test publishes synthetic odometry, spins rclcpp, and asserts that the node emits at least one onboard command message.
