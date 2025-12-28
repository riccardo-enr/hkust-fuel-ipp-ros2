# MPPI Controller

The MPPI (Model Predictive Path Integral) controller is a sampling-based Model Predictive Control (MPC) algorithm that uses GPU acceleration for real-time trajectory tracking. FUEL includes two MPPI variants optimized for different control representations.

## Overview

MPPI solves optimal control problems by:
1. Sampling thousands of control trajectories (K samples over H horizon steps)
2. Rolling out dynamics predictions for each trajectory on GPU
3. Computing costs based on tracking error, control effort, and obstacles
4. Computing exponentially-weighted average of samples based on costs
5. Applying first control command, shifting horizon, and repeating

The algorithm is highly parallelizable on GPU, making it suitable for real-time control at ~100Hz.

## Controller Variants

### 1. MPPI Acceleration (`mppi_acc_node`)

Optimizes **acceleration deltas** (3 DOF: x, y, z) and converts to force/orientation.

| Property | Value |
|----------|-------|
| Control Space | 3 DOF (acceleration vector) |
| Output Mapping | Acceleration → Force vector → Orientation (from force direction) |
| Use Case | When trajectory planner provides acceleration commands |
| Source | `src/i_mppi/mppi_control/include/mppi_control/mppi_acc_node.hpp` |

**Control Flow:**
```
Optimize: [ax, ay, az] → Force = mass * (a + g) → SO3Command
```

### 2. MPPI Thrust+Quaternion (`mppi_tq_node`)

Optimizes **thrust magnitude** (1 DOF) and **orientation quaternion** (4 DOF) = 5 DOF total.

| Property | Value |
|----------|-------|
| Control Space | 5 DOF (thrust + quaternion) |
| Output Mapping | Direct thrust+quaternion → SO3Command |
| Use Case | Direct control over attitude, explicit orientation constraints |
| Source | `src/i_mppi/mppi_control/include/mppi_control/mppi_tq_node.hpp` |

**Control Flow:**
```
Optimize: [thrust, qx, qy, qz, qw] → SO3Command (direct mapping)
```

**Key Differences:**
- Explicit orientation control (full attitude, not just yaw)
- Thrust constraints more natural than acceleration bounds
- Proper manifold handling for quaternions (tangent space noise)
- Higher dimensionality requires more samples for same coverage

## Parameters

### Common Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `K` | 9000 | Number of control samples (higher = better exploration, more compute) |
| `H` | 20-50 | Horizon length in steps (longer = better foresight, more compute) |
| `dt` | 0.02-0.05 | Time step per horizon step (seconds) |
| `lambda` | 0.01-0.1 | Temperature parameter (lower = more exploitation, higher = exploration) |
| `g` | 9.81 | Gravity acceleration (m/s²) |

### Cost Weights (Acceleration Variant)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Q_pos_x/y/z` | 50.0 | Position tracking cost weight |
| `Q_vel_x/y/z` | 2.0 | Velocity tracking cost weight |
| `R_x/y/z` | 0.1 | Control effort regularization weight |
| `R_rate_x/y/z` | 10.0 | Control rate change penalty (smoothness) |
| `w_obs` | 20.0 | Obstacle avoidance cost weight |

### Cost Weights (Thrust+Quaternion Variant)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Q_pos_x/y/z` | 10.0 | Position tracking cost weight |
| `Q_vel_x/y/z` | 1.0 | Velocity tracking cost weight |
| `Q_thrust` | 0.1 | Thrust tracking cost weight |
| `R_thrust` | 0.0 | Thrust effort regularization |
| `R_rate_thrust` | 0.1 | Thrust rate change penalty |
| `Q_quat` | 10.0 | Orientation tracking cost weight |
| `R_quat` | 0.0 | Quaternion effort regularization |
| `R_rate_quat` | 0.1 | Quaternion rate change penalty |
| `Q_omega` | 0.5 | Angular velocity smoothing penalty |
| `w_obs` | 100.0 | Obstacle avoidance cost weight |

### Control Constraints

| Parameter | Acc Variant | TQ Variant | Description |
|-----------|-------------|------------|-------------|
| `a_max` | 10.0 | N/A | Maximum acceleration magnitude (m/s²) |
| `tilt_max` | 0.6 | N/A | Maximum tilt angle (radians, ~34°) |
| `thrust_max` | N/A | 20.0 | Maximum thrust (N, ~2g for 1kg drone) |
| `thrust_min` | N/A | 1.0 | Minimum thrust (N, prevents singularities) |
| `sigma` | 0.2 | N/A | Noise standard deviation (acceleration) |
| `sigma_thrust` | N/A | 1.0 | Noise standard deviation (thrust) |
| `sigma_quat` | N/A | 0.1 | Noise standard deviation (quaternion tangent space) |

## Usage

### Launch with MPPI Controller

```bash
# Thrust+Quaternion variant (recommended)
ros2 launch plan_bringup fuel_pipeline.launch.py \
  controller_type:=mppi \
  world:=office \
  exploration:=true

# Acceleration variant
ros2 launch plan_bringup fuel_pipeline.launch.py \
  controller_type:=mppi_acc \
  world:=office \
  exploration:=true
```

### Configuration File

Located at `src/fuel_planner/plan_bringup/config/mppi.yaml`:

```yaml
mppi_control:
  ros__parameters:
    mppi:
      K: 9000
      H: 50
      dt: 0.02
      sigma: 0.2
      lambda: 0.01
      Q_pos_x: 50.0
      Q_pos_y: 50.0
      Q_pos_z: 50.0
      Q_vel_x: 2.0
      Q_vel_y: 2.0
      Q_vel_z: 2.0
      R_x: 0.1
      R_y: 0.1
      R_z: 0.1
      R_rate_x: 10.0
      R_rate_y: 10.0
      R_rate_z: 10.0
      w_obs: 20.0
      a_max: 10.0
      tilt_max: 0.6
      g: 9.81
    mass: 0.98
```

## Topics

### Subscribed Topics

| Topic | Message | Description |
|-------|---------|-------------|
| `odom` | `nav_msgs/msg/Odometry` | Current UAV state (position, velocity, orientation) |
| `planning/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | Reference trajectory (position, velocity, acceleration, yaw) |

### Published Topics

| Topic | Message | Description |
|-------|---------|-------------|
| `so3_cmd` | `quadrotor_msgs/msg/SO3Command` | Control command (force + orientation + gains) |

## Algorithm Details

### Dynamics Model

The MPPI controller uses a predictive model to simulate the quadrotor's future state. The state vector is defined as $x = [p, v]^T \in \mathbb{R}^6$, where $p$ is position and $v$ is velocity.

**Continuous Dynamics:**

$$  \
\begin{aligned}
\dot{p} &= v \\
\dot{v} &= a - g \mathbf{e}_3
\end{aligned}
$$  

where $g$ is gravity and $\mathbf{e}_3 = [0, 0, 1]^T$.

**Discrete Integration (RK4):**

To ensure accuracy over the horizon step $dt$, we use 4th-order Runge-Kutta integration:

$$  \
\begin{aligned}
k_1 &= f(x_t, u_t) \\
k_2 &= f(x_t + \frac{dt}{2}k_1, u_t) \\
k_3 &= f(x_t + \frac{dt}{2}k_2, u_t) \\
k_4 &= f(x_t + dt k_3, u_t) \\
x_{t+1} &= x_t + \frac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}
$$  

### Cost Function

The objective is to minimize the expected cost over a finite horizon $H$. The trajectory cost $S(\tau)$ for a sample sequence $\tau = \{u_0, \dots, u_{H-1}\}$ is:

$$  \
S(\tau) = \sum_{t=0}^{H-1} \left( q(x_t, u_t) \right) + \phi(x_H)
$$  

#### 1. Acceleration Variant Costs
For the `mppi_acc` controller, the stage cost $q(x, u)$ includes:

$$  \
q(x, u) = \underbrace{Q_p \|p - p_{ref}\|^2}_{\text{Position}} + \underbrace{Q_v \|v - v_{ref}\|^2}_{\text{Velocity}} + \underbrace{R_a \|a - a_{ref}\|^2}_{\text{Control Effort}} + \underbrace{w_{obs} C_{map}(p)}_{\text{Obstacle}}
$$  

#### 2. Thrust-Quaternion Variant Costs
For the `mppi_tq` controller, the control inputs are thrust $T$ and orientation $q$. The cost function includes tracking and regularization terms:

$$  \
q(x, u) = \underbrace{Q_p \|p - p_{ref}\|^2 + Q_v \|v - v_{ref}\|^2}_{\text{Tracking}} + \underbrace{Q_T (T - T_{ref})^2 + Q_q \Psi(q, q_{ref})}_{\text{Control Tracking}} + \underbrace{R_T T^2 + R_q \Psi(q, \mathbf{I})}_{\text{Control Effort}} + \dots
$$  

*   **Orientation Error** $\Psi(q, q_{ref})$: Approximates the geodesic distance on the unit sphere:

$$  \
\Psi(q, q_{ref}) = 1 - |q \cdot q_{ref}|^2
$$  

*   **Angular Rate** $\omega$: Estimated from discrete quaternion differences, penalized with $Q_{\omega}$.

### MPPI Update Law

MPPI computes the optimal control sequence $u^*$ as the separate probability-weighted average of $K$ sampled trajectories.

1.  **Calculate Weights:**
    For each sample $k \in [1, K]$, compute the importance weight $w_k$ based on its total cost $S_k$:

$$  \
w_k = \exp\left( -\frac{1}{\lambda} (S_k - \min_{j} S_j) \right)
$$  

    where $\lambda$ is the temperature parameter.

2.  **Update Control Mean:**

$$  \
u^*_t = \frac{\sum_{k=1}^K w_k u_{k,t}}{\sum_{k=1}^K w_k}
$$  

    *Note: For quaternions, a weighted SLERP or renormalization is performed instead of simple arithmetic averaging.*

3.  **Explore:**
    New samples are generated around the updated mean:

$$  \
u_{k,t} \sim \mathcal{N}(u^*_t, \Sigma)
$$  

## Comparison with SO3 Controller

| Feature | SO3 Control | MPPI Control |
|---------|-------------|--------------|
| Type | Geometric tracking controller | Sampling-based MPC |
| Model-free | Yes (feedback only) | No (requires dynamics model) |
| Obstacle avoidance | No | Yes (via SDF map) |
| Preview capability | No | Yes (receding horizon) |
| Computation | Low (CPU) | High (GPU required) |
| Tuning complexity | Low (gains only) | High (cost weights, sampling params) |
| Robustness to model errors | High | Moderate |

## Hardware Requirements

- **GPU:** NVIDIA CUDA-capable GPU (compute capability 7.0+ recommended)
- **Memory:** ~500MB GPU memory for K=9000, H=50
- **Compute:** ~5-10ms per MPPI update on RTX 3060

## Tuning Guidelines

1. **Increase K** if tracking is erratic (more samples)
2. **Increase H** if preview is insufficient (longer horizon)
3. **Decrease lambda** if controller is too conservative
4. **Increase Q_pos** for tighter position tracking
5. **Increase R_rate** for smoother control (less aggressive changes)
6. **Adjust w_obs** for obstacle avoidance aggressiveness

## Implementation Files

| File | Description |
|------|-------------|
| `include/mppi_control/mppi_acc_node.hpp` | Acceleration variant header |
| `include/mppi_control/mppi_tq_node.hpp` | Thrust+Quaternion variant header |
| `src/mppi_acc_node.cpp` | Acceleration variant CPU code |
| `src/mppi_tq_node.cpp` | Thrust+Quaternion variant CPU code |
| `src/mppi_acc_kernels.cu` | Acceleration variant GPU kernels |
| `src/mppi_tq_kernels.cu` | Thrust+Quaternion variant GPU kernels |

## References

- Williams et al. "Model Predictive Path Integral Control: From Theory to Parallel Computation" (2017)
