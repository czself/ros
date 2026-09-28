---
task_id: task008
type: route_timing_analysis
status: completed
from: route_run
to: orchestrator
revision: 0
---

# First-run P1 timing breakdown

## Sources

- Bag: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260927_115820/motion.bag` (`rosbag info`: 213 s, 485,943 messages; topics include `/traffic_light/state`, `/traffic_light/gate_status`, `/traffic_light/braking`, `/my_car/cmd_vel`, `/move_base/feedback`, `/odom`, and `/move_base/result`).
- Summary: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260927_115820/run_summary.json`.
- Route log: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260927_115820/route_executor.log`.
- Round19 comparison: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_round19_alpha02_p5_position_first_01/run_summary.json`.

P1 target was `(1.7062, -0.5707)`; the goal was sent at sim `1682.178` and MoveBase returned `SUCCEEDED` at `1722.755` (40.577 sim seconds). The route executor records `duration_s=42.37` using its monotonic wall timer. It then failed at sim `1725.408` with `OrderedDict mutated during iteration`; no P1 photo was saved and the run stopped at P1.

## P1 interval breakdown

The command bins below come from `/my_car/cmd_vel` over the MoveBase goal interval. Message intervals were capped at 0.2 s; the bins cover 39.99 of the 40.58 sim-second interval, leaving about 0.59 s unclassified sampling gaps.

| State | Time | Notes |
|---|---:|---|
| Forward translation | 10.83 s | |
| Reverse translation | 6.17 s | Repeated direction changes. |
| Translation and turn combined | 10.53 s | |
| Pure in-place turn | 11.96 s | No net translation command. |
| Zero command | 0.50 s | No long zero-command wait. |
| Red/yellow gate braking | **0.00 s** | `/traffic_light/braking` remained false throughout the P1 goal. |

Signals changed to YELLOW at `1690.805`, RED at `1693.736`, GREEN at `1703.351`, YELLOW at `1717.822`, and RED at `1720.756`. The gate stayed `CLEAR` until a brief `NORTHBOUND:APPROACH` from `1718.850` to `1719.750`, then returned to `CLEAR`; it never asserted braking. Thus the extra time was not a red-light hold. The bag shows 16 forward/reverse sign flips, about 1.00 m net odometry displacement but 1.73 m odometry path length, and roughly 22.49 s with nonzero angular command (pure-turn plus combined translation/turn). This is consistent with oscillatory/turning control consuming the P1 time.

## Displacement speed and Round19 comparison

- Corridor/goal progress from HOME to P1: approximately `1.029 m`.
- First-run route-executor speed: `1.029 / 42.37 = 0.0243 m/s`.
- First-run speed over the sim-time action interval: `1.029 / 40.577 = 0.0254 m/s`.
- Round19 P1 action duration was `12.24 s`; the same corridor progress gives about `0.0841 m/s`. First run therefore took about `3.46×` as long and achieved about `29%` of Round19's P1 progress rate.

Round19's full ordered-corridor baseline is about `0.049 m/s` after the declared photo dwell. Both P1 measurements are below the proposed `0.15 m/s` three-run threshold. Round19 reached and captured P1; this first run reached the MoveBase target but failed before capture.

## Reproduction/limitations

The time bins were computed from the bag's timestamped `/my_car/cmd_vel` samples within `[1682.178, 1722.755]`; signal-state/gate transitions use the same sim-time window. Net displacement and path length use the first and last `/odom` samples in that interval. No ROS node, goal, or robot command was issued during this analysis.
