# 4D Forward-Prediction-Only Numerical Ablation Design

## Goal

Extend the 4D HPC numerical-delay experiment with a third, fair ablation: original 4D HPC supplied with only a first-order actuator forward prediction.  It must show the contribution of motor-lag prediction without Artstein compensation of the pure input delay.

## Scope

Modify `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py` and its focused Python regression coverage.  Update the accompanying 4D numerical-simulation document to describe the third group and the output labels.  No ROS node, launch file, controller C++ code, plant parameters, or pre-existing result artifact is changed.

## Experiment groups

Every delay-bearing scenario uses the same initial conditions, 4D HPC parameters, command clipping, delay queue, first-order plant, `Td`, `tau`, sample time, and (where applicable) noise seed.

1. `original`: feedback uses the measured leader and follower states, with no compensation.
2. `forward_prediction_only`: feedback uses the measured Leader state and a Follower state predicted over only the first-order motor lag `tau`; it does not evaluate the Artstein integral and does not advance the leader or follower by the pure delay `Td`.
3. `compensated`: feedback uses the existing Artstein input-delay transformation followed by first-order motor forward prediction; the Leader prediction retains its existing `Td + tau` horizon.

The physical plant remains `cmd_vel -> Td -> tau -> v_real` in all three groups.  Thus the new group deliberately leaves the true pure delay uncompensated.

## Design

Add a small helper that predicts the Follower's position and velocity over `tau` from its measured state and the previously published velocity command.  The closed-form prediction must be consistent with the existing first-order actuator model:

```math
v_{pred}=v_{cmd}+e^{-1}(v_{meas}-v_{cmd}),
```

```math
p_{pred}=p_{meas}+\tau v_{cmd}+\tau(1-e^{-1})(v_{meas}-v_{cmd}).
```

The delay and circle simulators select the feedback state by group.  They keep the existing `original` and `compensated` behavior unchanged.  `forward_prediction_only` calls the new helper and uses the measured Leader state directly.

Delay and circle plots become three-way comparisons: original in red, prediction-only in orange, full Artstein + prediction in blue.  The summary CSV adds one row per new experiment with stable names that identify `forward_prediction_only`; existing row names remain unchanged.

## Verification

Add focused Python tests that load the script as a module and verify:

- the prediction-only helper follows the closed-form first-order response for a known state and command;
- both delay and circle simulators accept `forward_prediction_only`, produce samples, and retain the physical delayed plant;
- the summary includes the prediction-only row names and the plot functions accept all three result series.

Run the focused Python test, then run the script at a short duration into a temporary directory and assert it produces the expected figures and CSV.  The normal default command remains available for regenerating publication-scale results.

## Success criteria

- A single command produces fair three-group plots and metrics for MATLAB-leader, clean-circle, and noisy-circle delay cases.
- The new group applies `tau` prediction only, while `Td` remains a plant delay and is never Artstein-compensated.
- Existing original and full-compensation numerical behavior and labels are preserved.
- Focused regression tests pass.
