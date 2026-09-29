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

The previous four complete runs had reported HOME heading errors of about
0.027-0.030 rad. However, those values are based on map localization, not
Gazebo's physical chassis pose. The trial made the reported heading more
accurate; it did not fix physical parking. The second run is an environment
interruption; the third completed navigation and parking but failed the
person count and class checks. Neither is counted as a successful full task.
No display scores, model weights, or person thresholds were changed.

## Physical start/end check

The `/gazebo/link_states` samples for `my_car::chassis` in each closed bag
give the following physical displacement from the first recorded chassis
pose to the last. This is an offline truth check; the controller does not use
Gazebo pose as a localization input.

| Run | Physical XY difference | Physical yaw difference |
| --- | ---: | ---: |
| `20260929_task009_approach_trace_1` | 0.0545 m | +0.1346 rad |
| `20260929_task009_approach_trace_2` | 0.0384 m | +0.1319 rad |
| `20260929_task009_approach_trace_3` | 0.0527 m | +0.1374 rad |
| `20260929_task009_person_confidence_probe_1` | 0.0648 m | +0.1458 rad |
| `20260929_task009_home_precision_probe_1` | 0.0463 m | +0.1364 rad |
| `20260929_task009_home_precision_probe_3` | 0.0223 m | +0.1175 rad |

Thus the existing `parking_pass` and earlier three-run acceptance do not
establish the newly requested physical return to the exact starting pose.
The AMCL/map pose and physical chassis pose diverge at HOME; changing only
the commanded yaw or tightening the AMCL threshold cannot establish physical
accuracy. The next investigation should compare wheel odometry, AMCL/TF and
Gazebo truth over the route, then improve the localization source or a
legitimate parking reference. Any revised result needs an independent
physical start/end check as well as the full task audits.

The full run directories and bags remain under
`/home/sz/ros1_ws/photo_stops/standee_route_runs/`. The trial is not promoted;
the runtime HOME command is restored to the previous contract yaw. The prior
runtime remains the accepted baseline under the old AMCL-based criterion,
but physical parking to the starting pose is unresolved.
