---
task_id: task008
type: minimal_navigation_rewrite_review
reviewer: independent_architecture_reviewer
scope: static-source review; no code edits or simulation
---

# Minimal ROS1 Navigation Rewrite Review

## Recommendation

Keep the recorded HOME pose and the ten saved photo poses as the only mission goals. Rebuild the motion path around one deterministic forward path follower, a single final safety gate, and one mission coordinator. Do not add recovery goals, intermediate `move_base` goals, or per-point special-case goal retries. If motion stops making route progress, cancel, publish zero, preserve the bag, and fail that run for offline diagnosis.

The current route launch chain is `start_standee_photo_route.sh` → `start_navigation.sh` → `navigation.launch`/`planner.launch` (Navfn + DWA) + `cmd_vel_watchdog.py` + YOLO, then `route_executor.py`. It already fixes the ordered photo list and keeps the watchdog as the base-facing command writer. Most of the risk is that `route_executor.py` has grown into a second motion stack alongside `move_base` recovery and DWA.

## What to retain

- **Exact mission sequence:** HOME reset, POINT_1…POINT_10 once in order, then HOME. Keep the photo coordinates and route contract as separate immutable inputs. The contract is a permitted corridor mask/path, not a list of extra navigation goals.
- **One command chain:** the path controller publishes only to `/my_car/cmd_vel_nav`; the watchdog remains the only node publishing `/my_car/cmd_vel` to the robot.
- **Hard line and obstacle safety:** global and local maps must contain the same white-line overlay; unknown/lethal cells stay blocked; check the measured full chassis polygon, including the buffer, for planned turns and swept commands. Keep the watchdog’s independent texture check and fail closed on stale pose, map, or scan.
- **Fail-closed traffic gate:** RED, YELLOW, unknown, stale, conflicting, or insufficient-confidence observations yield zero forward motion. Only source-stamped stable GREEN plus simulated GREEN and enough remaining time may authorize crossing. Both gates stay enabled in strict and diagnostic route modes.
- **Photo evidence contract:** preserve the raw source stamp, same-stamp YOLO annotation and detections, confidence/box checks, complete-image criteria, and fatal-on-failure behavior at each saved photo point. Use one acceptance manifest as the source of expected counts and frame margins.
- **Full-run bag and deterministic failure:** record command inputs/outputs, action states, plan/local plan, costmaps/footprints, scan, odometry/TF, gate states, detector timestamps/metrics, route status, and `/rosout` from before the first goal through the final stopped state. On failure, cancel the current action, publish zero, close the bag, and never advance to the next point or automatically repeat the failed goal.

For no-progress, project the measured chassis pose monotonically onto the active approved path. Outside explicit traffic waits and photo dwell, if path progress is **under 0.05 m for 8 s**, cancel and stop. Angular movement, wheel rotation, reverse/forward sign changes, or a recovery command do not count as path progress. Log progress `s`, cross-track error, velocity, command source, gate state, and action state at each sample so a single bag can distinguish DWA oscillation, blocked footprint, stale localization, and signal hold.

## Minimal three-module structure

| Module | Owns | Explicitly does not own |
|---|---|---|
| **MissionSequencer** | Loads the one fixed point list; sends exactly one navigation action for each saved point and HOME; starts capture at arrival; records state; stops the mission on any failure or 8 s route-progress stall. | Controller tuning, traffic classification, duplicate retry/skip behavior, ad-hoc route points. |
| **PathFollower + SafetyGate** | A single `move_base` local-planner plugin consumes the Navfn plan constrained to the approved free-space corridor, tracks it forward with bounded angular rate, stops for local obstacles, checks full-footprint motion, and applies the white-line/traffic gates. The watchdog remains the only final velocity publisher to the robot. | Per-photo counts, image writing, goal-sequence ownership. |
| **VisionCapture** | Runs `best.pt`; publishes source-stamped detections and lamp state/confidence; saves same-stamp raw/annotated/detection records and applies the frozen photo criteria; reports metrics. | Navigation commands, OCR/plate characters. |

Treat ROS stack launch and rosbag recording as deployment plumbing, not another mission state machine. For target orientation, use a single bounded, collision-checked, zero-linear-velocity heading phase after XY arrival. Apply it uniformly at photo points rather than combining DWA’s final-yaw maneuver with a separate one-off P5 controller. If its swept footprint is not clear, stop and fail; do not spin-recover.

## `route_executor.py`: simplify without dropping safety

### Duplicated/relocatable logic

- **Recovery and retries are nested today:** `move_base` enables conservative clear + SafeEscape and rotate clearing; the executor also retries a goal, clears costmaps, waits, and resends. Remove the route-level retry/clear loop and disable automatic rotate recovery. A no-progress/abort becomes one stop-and-fail event. This is the direct first simplification for reducing reversal and recovery-spin loops.
- **Two motion controllers:** DWA owns normal navigation, while the executor publishes its own direct angular servo for P5. Replace the P5-only branch and per-point DWA dynamic-reconfigure churn with one path-following controller and one post-XY heading phase. Never let two controllers command motion concurrently.
- **Route parsing has multiple active formats:** `route_executor.py` supports diagram `route_order`, diagram `points`, and photo-point files. The mission should load only the canonical fixed photo-point file plus the one corridor contract; delete unused legacy parser branches once all callers are migrated.
- **Photo state has duplicate/dead copies:** `latest_image`/`latest_image_stamp` coexist with the stamped frame caches, while the stamp caches already carry the usable image. Keep one locked cache keyed by exact `(secs,nsecs)` and evict all three matched records together. Remove the unused latest-image copies.
- **Acceptance values have multiple sources:** hard-coded per-point counts, `photo_acceptance_criteria`, and the hard-coded total of 18 are all competing acceptance definitions. Keep per-photo required counts in one reviewed manifest; keep street-wide de-duplication separate and do not derive “unique” count by summing repeated views.
- **Goal preflight code can be one helper:** retain both distinct checks—full-footprint goal/sweep safety and ordered-corridor path validity—but share one checker implementation and one report schema. Do not repeat polygon/grid sampling logic in the runner, path follower, and bag analyzer.

### Checks that must remain

Do not remove (or silently bypass) the strict runtime gate flags; readiness check for best.pt and fresh stamped vision data; exact point-order/one-capture-per-point assertion; same-stamp capture validation; per-point count/box/frame-margin checks; full-footprint static/local/texture checks; stop-line authorization; HOME pose and zero-speed verification; or the full-bag run ledger. `STRICT_ACCEPTANCE=false` may loosen reporting only if both safety gates remain true and every resulting artifact is permanently labeled diagnostic-only.

## What to reuse from `2025AiCOMP_ZHSQ`

This review environment did not expose a local directory named `2025AiCOMP_ZHSQ`; I checked the public upstream source and use it only as a structural reference. Its useful patterns are:

- Separate a small `Goal`/`NavConfig`/`NavState` model, an action-client wrapper, a mission manager, and a task handler instead of putting all mission behavior in one script.
- Keep mission order declarative and maintain per-goal timing/logging and a structured status topic.
- Let the navigation coordinator call `move_base` once per declared task pose and make camera/task work a separate module.

Sources: [repository overview](https://github.com/nanbloom001/2025AiCOMP_ZHSQ), [NavExecutor](https://github.com/nanbloom001/2025AiCOMP_ZHSQ/blob/main/nav_manager_mod/nav_executor.py), [goal/config model](https://github.com/nanbloom001/2025AiCOMP_ZHSQ/blob/main/nav_manager_mod/config.py), and [task handler](https://github.com/nanbloom001/2025AiCOMP_ZHSQ/blob/main/nav_manager_mod/task_handler.py).

### Do not copy

- **Not its DWA parameters as a fix.** The reference repository’s main launch still uses `dwa_local_planner/DWAPlannerROS`; TEB files are not its active controller. Its parameter file contains multiple commented/experimental blocks and a different robot, footprint, map, sensor stream, and motion limits. It is not evidence that a value such as controller frequency, inflation, reverse speed, or oscillation timeout will fix this world. See its [README](https://github.com/nanbloom001/2025AiCOMP_ZHSQ#%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%A7%88) and [move_base parameters](https://github.com/nanbloom001/2025AiCOMP_ZHSQ/blob/main/param/move_base_params.yaml).
- **Not its traffic fail-open behavior.** The reference task handler treats a 25 s traffic-light timeout as permission to proceed and exposes a manual skip. That contradicts the required RED/YELLOW/unknown hold. It also counts repeated GREEN status strings without a source timestamp, confidence contract, and the current simulator-time clearance checks.
- **Not its route or “none” waypoints.** Its active mission describes 33 goals, including many intermediate `none` transition goals, on another map and coordinate frame. This task permits only the recorded ten photo points and HOME; its route corridor is a constraint, not an invitation to append goals.
- **Not its failure semantics or teleop controls.** The manager advances to the next index after a failed navigation or task, and supports keyboard pause/skip. Acceptance must abort the entire run on a missed point or gate failure and cannot skip or continue.
- **Not its camera topic, hardware launch, odometry/TF, model environments, OCR, or vision protocol.** Those are deployment-specific. The task uses this world’s ROS topics and user-provided `best.pt`; OCR is user-owned.

## Minimal implementation/test order

1. Preserve the current approved maps, footprint, gate, HOME, and ten photo poses. Freeze one input manifest and remove redundant runner branches; do not change planner gains during this code simplification.
2. Replace DWA as the active local controller with one forward corridor-tracking `BaseLocalPlanner` plugin (or an equivalent single action-server controller). Keep Navfn and the live local obstacle costmap. Disable rotate recovery and nested retry/clear behavior. Use a bounded, collision-checked heading phase only after position arrival. Keep the watchdog as the final command gate.
3. Recompute the same HOME + ten photo-point paths using the ordered corridor and full footprint. Verify that every path is nonempty and paint-free before any motion.
4. Record one complete diagnostic bag before motion and through shutdown. Run one route only after static checks pass; use path-projection no-progress analysis to stop/fail, not to authorize a recovery turn. Do not call that run acceptance unless all photos, traffic encounters, white-line checks, and HOME checks pass.

This is a design review only. No code, launch state, goal, or simulation was changed or run.
