# Final fix report

## Changes

- `simulate_delay_case("original")` now copies both controller state snapshots before the follower plant updates `x2` in place.
- The original and compensated delay regressions now run for `0.30 s`, exceeding `Td = 0.22 s`; the original expected row records the pre-plant controller state.
- Added the standard `python3-pytest` test dependency for the CTest command that invokes `python3 -m pytest`.

## Inspection outcome

`compensated` builds fresh prediction arrays, while `forward_prediction_only` builds a fresh follower prediction and does not mutate the leader after assigning its reference. Their recorded controller-state semantics therefore do not require an additional copy.

## Verification

- `python3 -m pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py` — 8 passed (one pre-existing Matplotlib Axes3D environment warning)
- `git diff --check` — passed
