---
task_id: task008
type: route_speed_baseline
status: completed
from: route_run
to: orchestrator
revision: 0
---

# Round19 route speed, stalls, and turns

## Sources and method

- Round summary: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_round19_alpha02_p5_position_first_01/run_summary.json`.
- Route/action and turn log: `/home/sz/ros1_ws/navigation_diagnostics/20260926_round19_alpha02_p5_position_first_01/route_executor.log`.
- Bag: `/home/sz/ros1_ws/navigation_diagnostics/20260926_round19_alpha02_p5_position_first_01/motion_0.bag` (closed and previously checked with `rosbag info`). P5 in-place command stats are also saved in `targeted_analysis.json`.
- Progress distance is the ordered `inner_route.yaml` arclength between the projected saved HOME/photo poses, including HOME wrap; it is not raw odometry. The 11 leg distances sum to the route polyline length, `12.8855 m`.
- Round start was sim `2275.215`; parking verification was `2559.817`, total `284.602 s`. The route used fixed photo capture settings `.6 s settle`, 8 frames at `.2 s` intervals. Including the code's fixed `.25 s` pre-capture pose check gives `2.25 s × 10 = 22.5 s` photo dwell. No traffic wait was declared; Round19 had traffic enforcement disabled, so it is not traffic-compliance evidence.

## Run-level active progress

Using the full capture routine as declared dwell leaves `262.102 s` active, so `12.8855 / 262.102 = 0.0492 m/s`. If only the configured `.6 + 7×.2 = 2.0 s` settle/burst dwell is excluded and the fixed `.25 s` pose-check pause remains active, the result is `12.8855 / 264.602 = 0.0487 m/s`. Either interpretation is about one third of the proposed **0.15 m/s** minimum and fails it.

The per-action goal durations sum to `237.43 s`, including the P4 retry and both HOME parking goals. P5's two fixed-direction same-point turns add `23.30 s` (`13.75 + 9.55 s`), for `260.73 s` of logged active goal/turn time. The remaining `1.37 s` versus the mission-span estimate is handoff/validation overhead; it is not removed from the stricter run-level denominator.

## Per-leg baseline

| Leg | Corridor progress | Active time | Progress rate | Notes |
|---|---:|---:|---:|---|
| HOME→P1 | 1.029 m | 12.24 s | .084 m/s | One successful goal. |
| P1→P2 | 1.650 m | 11.55 s | .143 m/s | One successful goal. |
| P2→P3 | 1.154 m | 20.16 s | .057 m/s | One successful goal. |
| P3→P4 | 1.392 m | 63.04 s | .022 m/s | Attempt 1 aborted after 26.30 s; attempt 2 took 36.74 s. |
| P4→P5 | .199 m | 26.23 s | .008 m/s | 2.93 s XY goal plus 13.75 s approach turn and 9.55 s camera-yaw turn. |
| P5→P6 | 1.106 m | 14.16 s | .078 m/s | One successful goal. |
| P6→P7 | .039 m | 24.19 s | .002 m/s | Only .039 m along corridor; endpoint chord is .177 m laterally. |
| P7→P8 | 2.267 m | 15.28 s | .148 m/s | Closest to the .15 m/s target. |
| P8→P9 | .833 m | 11.43 s | .073 m/s | One successful goal. |
| P9→P10 | .550 m | 28.42 s | .019 m/s | One successful goal. |
| P10→HOME | 2.667 m | 34.03 s | .078 m/s | Two same-HOME parking actions, including precise-park correction. |

The slowest corridor-progress leg is **P6→P7**. Its route-projected progress is below `.05 m` over the entire `24.19 s` leg, although its endpoint chord is `.177 m`; the scalar corridor-progress definition therefore needs to state how legal cross-track correction counts. P4 and P9→P10 are also slow, with P4 spending `63.04 s` on one corridor leg because of an abort/retry.

## Stalls and turning time

- P5 approach-bearing turn: `-107.5°`, from sim `2391.734` to `2405.485`, **13.751 s** at the same point with zero measured position drift.
- P5 camera-yaw lock: `-74.3°`, from sim `2408.726` to `2418.276`, **9.550 s**, again at the same point with zero position drift.
- Bag samples show P5 `/my_car/cmd_vel_nav` angular sign was `-1` throughout both turns, with **zero sign flips** and zero linear command. There were 272 approach-turn samples and 188 camera-turn samples; each ended with one zero-angular sample.
- These purposeful turns still consume active time under the review rule. Each is individually longer than the proposed 8 s no-progress window while the robot remains at one corridor position. A strict `<.05 m progress for 8 s` test would therefore fail during either turn unless the metric explicitly recognizes verified same-point yaw motion; the current review says turn time is not excluded.
- P6→P7 is another stall-rule risk: total corridor progress is only `.039 m` over `24.19 s`. Its lateral endpoint displacement shows that arclength projection alone may undercount legitimate photo-pose repositioning.

## Is `.15 m/s` usable?

It is measurable as a strict **fail gate**, but this frozen Round19 baseline does not meet it: active progress is only `.049 m/s`. The run also includes the P4 retry and P5 in-place turns, which the review explicitly says to count. The threshold should not be loosened to make this run pass. Before the three-run streak, define corridor progress so it captures permitted lateral correction without counting loops/backtracking, then improve the controller while keeping the fixed points and the `.35 m/s` speed cap.

Possible control-only directions, subject to separate review and the same full-footprint sweep checks:

- Reduce the P5 direct-turn time by tuning a bounded angular-rate cap above the current `.15 rad/s`, retaining the fixed turn direction, zero translation, and pre-turn global/local/texture sweep checks.
- Diagnose and remove the P4 abort/retry so one leg does not consume `63 s`.
- Tune the P6→P7 forward-only XY+yaw controller around its short lateral transition; report both along-corridor and cross-track progress.
- Investigate P9→P10 DWA/recovery time (`28.42 s`) without changing route points or adding goals.

This baseline uses only saved Round19 logs/bag/route definitions. No ROS goal, parameter change, or robot motion was issued for this analysis.
