---
task_id: task006
type: runtime_readiness
status: completed
from: route_run
to: orchestrator
result: NOT_READY_FOR_ACCEPTANCE_RUN
---

# Task 006 runtime readiness — read-only check

## Conclusion

**Do not start an acceptance route from the current runtime.** The navigation nodes are present, but the Gazebo bridge/world interface and simulation clock are not currently available. The current local-costmap source parameter also differs from the revised worktree launch config, and traffic-gate enforcement is disabled. No route goal, reset, parameter change, or robot movement was issued in this check.

## Live state observed

- `rosnode list` contained AMCL, move_base, local/map servers, watchdog, wheel odometry, and sensors; it did not contain `/gazebo`.
- `/clock` produced no message during the probe. `/gazebo/get_model_state` and other Gazebo state services were absent; a model-state query timed out. The current physical model pose/twist therefore could not be verified.
- The most recent readable `/move_base/status` was terminal `ABORTED` (`Robot is oscillating. Even after executing recovery behaviors.`); no active goal was present. `/my_car/cmd_vel` produced no message during the short probe. The last saved Round19 stop snapshot records zero command, but that does not establish current Gazebo motion while `/gazebo` and `/clock` are unavailable.
- `/cmd_vel_watchdog/enforce_white_lines=true`; `/cmd_vel_watchdog/enforce_traffic=false`. The latter makes stop-line/zebra traversal diagnostic-only and cannot support traffic-compliance acceptance.
- `/map` and `/map_local_navigation` were both 224×224 OccupancyGrids in `map`, at 0.05 m/cell with origin `(-6,-6)`. Their arrays matched exactly: **50,176/50,176 cells**.
- The live local static-layer parameter still reads `/map_local_navigation`; `/move_base/local_costmap/track_unknown_space` is unset, while global unknown tracking is true. The local rolling costmap was 180×180 in `odom` at 0.05 m/cell.

## Launch and configuration evidence

- In this progress worktree, `navigation/local_costmap.yaml` sets `map_topic: /map` and `track_unknown_space: true`; `navigation/global_costmap.yaml` also sets `track_unknown_space: true`. The progress `planner.launch` loads those files without a later `map_topic` override, and progress `navigation.launch` starts one static `/map_server` rather than a separate raw local-map server.
- The currently running parameter `/move_base/local_costmap/static_layer/map_topic=/map_local_navigation` does not match that fresh-launch configuration. The two published map arrays currently match, but this does not prove what a future local static layer will subscribe to or retain.
- The main worktree has a separate local map server publishing `/map_local_navigation`; its current `planner.launch` explicitly passes `/map` to the local static layer. The live parameter still reports `/map_local_navigation`, so the active navigation nodes do not reflect either source configuration exactly.
- `scripts/start_navigation.sh` calls `runtime_control.py reset`, which sets the Gazebo model to the contract birth pose. I did not run it because this task forbids robot movement/reset. A fresh-launch readback of the revised local parameters remains unverified.
- `scripts/start_standee_photo_route.sh` keeps the white-line gate enabled, but does not set `ENFORCE_TRAFFIC=true`; `start_navigation.sh` defaults traffic enforcement to false. A caller must explicitly enable traffic gating before any traffic-compliance run.

## Existing route evidence

- The latest saved Round19 full-route record is `/home/sz/ros1_ws/navigation_diagnostics/20260926_round19_alpha02_p5_position_first_01/result_report.md` with bag `motion_0.bag` and photos under `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_round19_alpha02_p5_position_first_01/`. It completed the ten-point sequence and HOME, but P5 showed only three people instead of four and P7 clipped the signal top. Round19 ran with white-line enforcement disabled, so it is not white-line compliance evidence.
- The orchestrator's prior read-only route audit reports 11 Navfn legs with full-footprint clearance, monotonic projection onto `inner_route.yaml` including the HOME wrap, and maximum cross-track deviation `0.112 m`. This runtime-readiness check did not reissue plans because the simulation clock/Gazebo service was unavailable.
- OCR is outside this readiness check.

## Conditions before a later route run

1. Restore the Gazebo ROS API and advancing `/clock`; verify `/gazebo/get_model_state`, current model twist, and sensor/AMCL freshness.
2. After an explicitly authorized safe reset/fresh launch, verify the live local static-layer source is `/map`, both costmaps use `track_unknown_space=true`, and `/map`/local map contents remain identical.
3. Require `/cmd_vel_watchdog/enforce_white_lines=true` and, for traffic-compliance acceptance, `enforce_traffic=true` with fresh validated traffic perception. Otherwise label the run diagnostic-only.
4. Confirm no active goals, zero drive command, and readiness checks before dispatching the fixed route.

This audit performed only read-only ROS queries and file inspection. It sent no MoveBase goal, did not reset/restart the robot or stack, and did not change parameters or route points.
