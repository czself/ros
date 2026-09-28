---
task_id: task008
type: bag_failure_diagnosis
status: completed
from: route_run
to: orchestrator
revision: 0
---

# 2026-09-27 first-run P1 no-progress diagnosis

## Outcome and goal timing

- Bag: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260927_121819/motion.bag` (32.2 s, 73,724 messages, readable with `rosbag info`).
- Summary: `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260927_121819/run_summary.json`.
- P1 target: `(1.7062, -0.5707, yaw 1.6168)`. The route executor sent the same goal twice. Attempt 1 ran from sim `2848.335` to `2863.435`; attempt 2 from `2864.425` to `2877.724`. Both ended as route-level `NO_PROGRESS`. MoveBase result status was `PREEMPTED` after the executor canceled each still-active goal; this was not a Navfn `ABORTED`/no-plan result.
- No photo was saved. After the second failure, the executor stopped with `OrderedDict mutated during iteration`; that later exception did not cause the two no-progress timeouts.

## Pose change and command evidence

| Attempt | Map feedback start→end | Feedback net XY | Wheel odom start→end | Command activity |
|---|---|---:|---|---|
| 1 | `(1.7151, -1.5978, 1.607)` → `(1.5787, -1.0042, 2.090)` | `.609 m` | `(2.7574, -.1151, -.408)` → `(3.3158, -.2582, .082)`; `.576 m` net | `/cmd_vel_nav`: 3.82 s forward, 3.07 s reverse, 7.41 s mixed, .30 s pure turn; 15 linear-direction sign flips. Actual `/my_car/cmd_vel` was similar, also 15 flips. |
| 2 | `(1.5765, -1.0002, 2.085)` → `(1.5640, -.9742, 1.971)` | `.029 m` | `(3.3176, -.2581, .080)` → `(3.3286, -.2586, -.030)`; `.011 m` net | `/cmd_vel_nav`: 7.20 s forward, 4.72 s reverse, .90 s mixed; 24 sign flips. Actual output was similar, with 25 flips. |

The physical Gazebo chassis during attempt 2 changed from world `(1.62943, -1.02817, yaw 2.10960)` to `(1.61939, -1.02059, yaw 1.99637)`: only **1.26 cm** net XY over 13.30 sim seconds, with about `−0.113 rad` yaw change. The commanded controller kept alternating forward and reverse, but the base did not translate toward P1. At the end, feedback remained roughly `.43 m` from P1.

## Watchdog, signal, sensors, and costmaps

- `/traffic_light/gate_status` stayed `CLEAR`; `/traffic_light/braking` stayed `false`. There was no watchdog stop or white-line veto. The simulated signal was RED at bag start, changed GREEN at `2853.541`, then YELLOW at `2867.951` and RED at `2870.860`; those phases did not produce a braking wait in this bag. The apparent stall is not red-light waiting.
- `/scan` published 321 scans in 32.2 s (360 rays). Near the attempts, valid ranges remained available; minimum returns were about `.31–.41 m`. There was no scan dropout.
- Costmaps were present. Global: `map`, 224×224 at `.05 m/cell`; local: `odom`, 180×180 at `.05 m/cell`. At the sampled attempt start/end poses, the full footprint was clear in the recorded local grid with zero extra margin. This only checks the current pose, not future DWA trajectories.
- AMCL began close to the HOME truth pose. During the stalled second attempt, AMCL updates were sparse/stale (the last nearby pose stamp was about 9 s older at attempt start, then updated near the attempt end). However, wheel odometry and Gazebo truth both confirm near-zero physical translation during attempt 2, so stale localization alone does not explain the no-progress result.

## Cause assessment

The clearest observed failure layer is **move_base local control/path-cost interaction**, not the watchdog: MoveBase accepted each goal, DWA's published commands repeatedly changed forward/reverse sign, the watchdog forwarded those commands without braking, and the physical chassis barely moved on attempt 2. Nearby scan returns and populated local-costmap cells make obstacle/costmap interaction a plausible trigger.

The bag does **not** contain `/move_base/NavfnROS/plan`, `/move_base/DWAPlannerROS/local_plan`, `/move_base/DWAPlannerROS/cost_cloud`, or trajectory candidates. Therefore it cannot distinguish a bad/stale global path from DWA oscillation caused by local obstacle/inflation costs. The recorded costmap snapshots show the current footprint is clear, not that all candidate paths are clear. Prioritize capturing the Navfn plan, local plan, cost cloud, recovery timestamps, and dynamic parameters around each attempt before changing planner or costmap settings.

No navigation goal, ROS service call, parameter change, or robot command was issued during this diagnosis.
