# Parameter Tables

Track essential parameters for planners and simulators.

## Planning

| Package | Parameter | Default | Description |
|---------|-----------|---------|-------------|
| `plan_manage` | `use_sim_time` | `true` | Enable simulation clock when running with simulator |
| `plan_manage` | `traj_server/horizon` | `5.0` | Command horizon in seconds |

## Controllers

### MPPI Acceleration Variant (`mppi_acc_node`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `K` | 9000 | Number of control samples |
| `H` | 50 | Horizon length in steps |
| `dt` | 0.02 | Time step per horizon step (seconds) |
| `sigma` | 0.2 | Noise standard deviation |
| `lambda` | 0.01 | Temperature parameter (lower=exploit, higher=explore) |
| `Q_pos_x/y/z` | 50.0 | Position tracking cost weight |
| `Q_vel_x/y/z` | 2.0 | Velocity tracking cost weight |
| `R_x/y/z` | 0.1 | Control effort regularization |
| `R_rate_x/y/z` | 10.0 | Control rate change penalty |
| `w_obs` | 20.0 | Obstacle avoidance cost weight |
| `a_max` | 10.0 | Maximum acceleration (m/s²) |
| `tilt_max` | 0.6 | Maximum tilt angle (radians) |

### MPPI Thrust+Quaternion Variant (`mppi_tq_node`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `K` | 9000 | Number of control samples |
| `H` | 20 | Horizon length in steps |
| `dt` | 0.05 | Time step per horizon step (seconds) |
| `lambda` | 0.1 | Temperature parameter |
| `sigma_thrust` | 1.0 | Thrust noise standard deviation |
| `sigma_quat` | 0.1 | Quaternion noise (tangent space) |
| `Q_pos_x/y/z` | 10.0 | Position tracking cost weight |
| `Q_vel_x/y/z` | 1.0 | Velocity tracking cost weight |
| `Q_thrust` | 0.1 | Thrust tracking cost weight |
| `R_thrust` | 0.0 | Thrust effort regularization |
| `Q_quat` | 10.0 | Orientation tracking cost weight |
| `R_quat` | 0.0 | Quaternion effort regularization |
| `Q_omega` | 0.5 | Angular velocity smoothing penalty |
| `w_obs` | 100.0 | Obstacle avoidance cost weight |
| `thrust_max` | 20.0 | Maximum thrust (N) |
| `thrust_min` | 1.0 | Minimum thrust (N) |

Pull actual defaults from YAML files when documenting.
