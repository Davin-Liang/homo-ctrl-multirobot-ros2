# 6D Kinematic HPC Stability Analysis

This folder contains the MATLAB code for analyzing the stability of the 6D time-varying kinematic homogeneous formation controller.

## Dependencies

The original HPC toolbox (`../source/`):
- `lpc2hpc.m` — LPC-to-HPC upgrade (works for arbitrary dimension n)
- `hnorm.m` — Homogeneous norm via bisection
- `block_con.m`, `trans_con.m` — Block controllable canonical form
- `e_hpc.m` — Explicit HPC control law

## Usage

```matlab
cd matlab/6d_analysis
demo_6d_stability
```

## What it does

### 1. Frozen-Time Stability Analysis (Figure 1)

Sweeps over leader velocities (omega_l ∈ [-1.5, 1.5] rad/s, vx_l ∈ [0, 2] m/s) and verifies:
- All frozen-time closed-loop systems A_l + B*K are Hurwitz
- The HPC Lyapunov condition P*Gd + Gd'*P > 0 holds
- Admissible homogeneity degree range [nu_min, nu_max]

### 2. Leader-Follower Circular Trajectory Simulation (Figures 2–4)

Leader follows a circular trajectory (constant body-frame velocity). Follower tracks with:
- 6D HPC (homogeneous control)
- 6D LPC (linear control, ablation baseline)

Outputs: XY trajectories, position/heading errors, Lyapunov function V(e), control inputs.

### 3. 6D HPC vs 4D HPC Model Comparison (Figure 5)

Compares the 6D kinematic model against the original 4D point-mass model:
- The 6D model integrates yaw into the main control loop (no decoupled P+FF)
- Body-frame velocity semantics match the real robot cmd_vel interface
- Continuous boundary projection (no discrete formation point switching)

### 4. Robustness to Leader Velocity Variation Rate (Figure 6)

Tests tracking performance under sinusoidal leader turning at different frequencies (0.1–1.0 Hz). Demonstrates that gain-scheduled HPC maintains stability as the time-varying A matrix changes.

## Key Theoretical Result

For the 6D time-varying system with gain-scheduled HPC:
- **Frozen-time**: Each A_l(omega_l, vx_l, vy_l) + B*K is Hurwitz, and V(e) = ||e||_{Gd,P} is a valid Lyapunov function for the frozen system
- **Slow variation**: The tracking error is uniformly ultimately bounded, with bound proportional to the rate of change of leader velocity
- **Gain scheduling**: HPC parameters (G0, P, nu, Gd) are recomputed when leader velocity changes beyond threshold, ensuring stability across the operating envelope

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| mass | 8.0 | Translation tuning mass |
| I_val | 1.0 | Rotational inertia tuning |
| radius | 2.0 m | Formation safety circle |
| omega_d_pos | 1.5 | Position damping bandwidth |
| omega_d_theta | 1.5 | Yaw damping bandwidth |
| hpc_vel_threshold | 0.3 | HPC recomputation threshold |
