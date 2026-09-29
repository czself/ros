# HOME heading precision pilot (2026-09-29)

The verified runtime before this experiment is `d9a61b4`; the current
development branch was backed up to GitHub as
`backup/task009-before-home-precision-20260929` at `5efd120`. The isolated
trial code is preserved at `33078e4`. The trial changed only the final HOME
control yaw by -0.029 rad. The route contract and parking acceptance target
remained `(1.714860, -1.599947, 1.606236)`.

| Run | Result | HOME position error | HOME heading error to contract |
| --- | --- | ---: | ---: |
| `20260929_task009_home_precision_probe_1` | Complete; independent and presentation audits pass | 0.01297 m | 0.00348 rad |
| `20260929_task009_home_precision_probe_2` | Failed at POINT_2: duplicate ROS `/route_executor` registration | Not reached | Not reached |
| `20260929_task009_home_precision_probe_3` | Failed: A-area stranger classified as resident at 0.398 | 0.02443 m | 0.00357 rad |

The previous four complete runs had HOME heading errors of about
0.027-0.030 rad. The trial therefore suggests a repeatable control-side
heading offset, but only one full trial passed all acceptance checks. The
second run is an environment interruption; the third completed navigation
and parking but failed the person count and class checks. Neither is counted
as a successful full task. No display scores, model weights, or person
thresholds were changed.

The full run directories and bags remain under
`/home/sz/ros1_ws/photo_stops/standee_route_runs/`. The trial is not promoted;
the runtime HOME command is restored to the verified contract yaw. A future
HOME-specific improvement should be tested with physical Gazebo start/end
pose and map localization side by side, then pass three new complete runs.
