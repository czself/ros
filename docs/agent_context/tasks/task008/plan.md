---
task_id: task008
type: plan
status: revised_for_execution
from: planner
to: route_runner
revision: 1
requires_review: true
---

# Task 008 Plan: Three consecutive complete route runs

## Goal

After the white-line, live-perception, and traffic checks pass, freeze the route, maps, code, parameters, model, and confidence thresholds. Complete the exact sequence HOME -> POINT_1 ... POINT_10 -> HOME three consecutive times. OCR is excluded by user instruction.

Before freezing, run clearly labeled photo-framing checks. If a target is genuinely clipped, revise only that failing point and repeat static safety checks. After freezing, any code/map/route/parameter/model change or failed run resets the acceptance streak to 0/3.

## Pass gate for each run

### Route and photographs

- Start from the recorded birth pose; autonomously visit exactly POINT_1 through POINT_10 once and in order, capture one accepted photo at each, and return to HOME. No teleoperation, skipped point, inserted point, aborted/retried goal, or continuation from a partial run.
- The acceptance launcher runs strict mode with an 8-second no-progress timeout, ordered-corridor checks, and HOME verification within 3 cm/0.04 rad. The local planner reports goal success only after zero command and 0.35 s of pose stability (translation change <=3 mm, yaw change <=0.01 rad); if the chassis settles outside the target tolerance, it continues the same goal. Every goal must succeed on attempt 1; an extra attempt fails the run.
- Photo-pose position tolerances are 3 cm by default; POINT_3, POINT_7, and POINT_10 use 5 cm, and POINT_5 uses 2.5 cm with 0.025 rad heading tolerance. POINT_7 and POINT_10 retain their saved camera poses; the 5 cm tolerance prevents terminal-heading control from oscillating across a tighter XY threshold. Every accepted image still must pass its full-content and depth checks.
- Each photo is captured only inside the frozen per-point localization tolerances and has a raw image, same-source-stamp YOLO annotation, and detection JSON. The independent auditor checks matching stamps and image dimensions.
- Before the current freeze, POINT_1 was moved 5 cm backward along its saved camera view axis after run `20260927_221400` showed the `green_off` detection box touching image row 0. Its saved heading is unchanged; the current white-line/corridor static audit passes.
- P1/P7 show all three signal lamps; P2/P3/P4 show exactly 3 complete people; P5 shows 4; P6 shows 5; P8/P9/P10 show the full plate region. The actual target silhouette/plate must fit in frame; a fixed 5% box margin is not required when the target pixels are complete. No OCR is performed.
- While the vehicle is moving or a traffic gate is active, live YOLO averages >=4.95 processed frames/s and at least 95% of 1 s metric windows are >=5 Hz. The 95th percentile of window p95 inference latency is <=250 ms; no active-window maximum frame age exceeds 500 ms; and no source-stamp gap exceeds 500 ms. The small mean-rate tolerance covers one-frame window-boundary/queue jitter; the window fraction and source-gap limits still enforce sustained freshness. Startup and photo-idle windows are recorded but excluded from this service-rate check. Ordinary targets meet confidence >=0.25. A contextual P10 plate ROI may use a >=0.15 source-class detection when the source class, confidence, and complete ROI are retained. Report resident/stranger counts per point and outsider detections; do not infer a unique street-wide identity without re-identification.

### Traffic and lane rules

- Record all signal encounters. RED, YELLOW, unknown, stale, low-confidence, or simulation-conflicting state must hold the front edge >=0.04 m before the stop line; target margin 0.10 m. A valid crossing requires 3 consecutive GREEN detections, each <=500 ms old when received with increasing source stamps no more than 500 ms apart; the latest source stamp must still be <=500 ms old at authorization, simulation must also be GREEN, and >=2.0 s must remain. After this authorization starts forward motion, retain the entry latch for at most 1.2 s to reach the line; once the front crosses, continue clearing until the rear edge is >=0.04 m past the line, even if the lamp changes. An idle/unstarted authorization expires, and a stalled entry is revoked before another signal cycle. Zero false-GREEN authorizations.
- White-line gate and traffic gate stay enabled. Full rotated chassis overlap with ordinary lane paint and ordinary line crossings are both zero. Touching any prohibited pixel is a failure. Start-line and GREEN-gated stop/zebra exceptions are logged separately with pose and lamp evidence.
- The executed center path stays within the ordered approved inner corridor; no diagonal shortcuts outside it. The supervisor may project up to 10 cm behind the last completed route cursor to absorb the already-accepted endpoint localization tolerance; the cursor itself remains monotonic and the final white-line watchdog remains authoritative.

### Pace and no stall

- Route progress divided by time with a positive forward chassis command must be >=0.15 m/s. Report the route-progress speed over all active time, including turns and alignment, as a second metric. Total closed-bag mission duration, including turns, valid signal waits, photo dwell, and HOME parking, must be <=180 seconds.
- Fail a leg if progress is <0.05 m for 8 consecutive seconds outside declared waits/dwell. Max forward speed remains <=0.35 m/s and stop-line approach <=0.12 m/s.
- Record the configured route mode and prove an abort ends the run; the run summary must show one attempt per goal and no continuation after failure.

### Finish pose and evidence

- Return to HOME with position error <=0.03 m and yaw error <=0.04 rad; linear and angular speed stay <0.01 for 2 continuous seconds.
- Record AMCL and Gazebo truth at start/finish. Truth is evaluator-only and never a navigation input.
- Save a closed/readable bag, route/action log, raw/annotated photos, matched detection JSON, YOLO timing and traffic-gate traces, continuous chassis/paint-clearance audit with translation samples <=1 cm and yaw samples <=1 degree, per-leg speed/stall report, exact hashes of code/map/points/model/config, and final pose errors.

## Three-run state

- Initialize streak at 0/3.
- A run increments the streak only when every criterion above passes and `audit_standee_photo_run.py` reports `acceptance_pass: true` from the closed bag and saved evidence.
- Any single failure resets to 0/3. Preserve the failed run and reason, fix the cause, re-review any changed code/config, perform a fresh authorized HOME reset, and begin a new full route. Never resume mid-route or count a repaired point as the same run.
- No edits, tuning, or diagnostic route runs are allowed during the frozen 3/3 streak.
- End the workflow and session only after three consecutive independently accepted runs.

## Handoff

Maintain an acceptance ledger in this directory with `0/3`, `1/3`, `2/3`, `3/3`, artifact path, frozen hashes, and the independent bag-audit result for each attempt. Once the frozen streak starts, no runtime code, map, point, model, or parameter changes are allowed.
