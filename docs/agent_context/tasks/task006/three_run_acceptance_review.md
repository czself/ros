---
task_id: task008
type: independent_acceptance_gate_review
reviewer: independent_architecture_reviewer
decision: CHANGES_REQUESTED
scope: non-OCR end-to-end acceptance
---

# Three-Run Acceptance Gate Review

## Decision

**CHANGES_REQUESTED for the acceptance plan.** The order `task006 → task007 → task008` is the right safety order, but the current roadmap/todo do not specify three consecutive passes, speed limits, a real-time inference service level, reset semantics, or several measurable pass conditions. There is no task008 plan yet. Put the gate below into task008 before running an acceptance route.

This review does not request or perform a simulation run. OCR is excluded from this gate per the user's instruction; the agent pipeline must still detect and annotate the plate region and provide its crop for the user's OCR work.

## Recommended minimum pass criteria

Freeze the code, launch configuration, map, route/point file, `best.pt` SHA-256, class map, and detection thresholds before Run 1. Keep those identical for all three runs. Each run starts from the declared birth pose and autonomously follows the diagram corridor, visits `POINT_1` through `POINT_10` in order, captures each point, then returns to HOME. No teleoperation, skipped point, added goal, route retry, or continuation from a partial run is allowed.

### 1. Route, photos, perception, and reporting

- Each run has exactly ten ordered photo records and one HOME completion record. Every navigation action succeeds on its first attempt; an abort, timeout/reissue, missing point, or out-of-order point fails the run.
- Arrival at each photo point is within the already approved per-point position and heading tolerances. Do not change the saved point coordinates or loosen tolerances during the three-run streak.
- The designated saved image for each point passes the established content standard: P1 shows the complete signal; P2–P4 each show three complete people; P5 shows four; P6 shows five; P7 shows the complete signal; P8–P10 each show the complete plate. A person or plate is not complete if its visible silhouette/plate is clipped by the frame. Preserve the full image and annotated image for each accepted capture.
- Use `best.pt` live in the ROS camera path, not only as an offline batch after the run. Freeze a validated confidence threshold (the still-image audit used 0.25); every expected visible object must be detected at or above it. The class names must match the audited checkpoint: `resident`, `stranger`, `red_on`, `red_off`, `yellow_on`, `yellow_off`, `green_on`, `green_off`, and `license_plate`.
- Minimum real-time service level: at least **5 processed camera frames/second** sustained while the robot is moving or a signal gate is active; p95 capture-to-published-detection/annotation latency **≤250 ms**; no consumed frame older than **500 ms**. A stale or missing perception stream must fail closed for traffic movement. Record rates, frame age, and latency from timestamps in each run's bag/log.
- Every console detection has a matching annotated frame from the same source timestamp and contains class/content plus confidence. Report the street-wide **unique** person total against the scene's predeclared standee inventory so repeated views are not double-counted. At P8–P10, require the plate box and annotated crop; character OCR is outside this gate.

### 2. Traffic-light behavior

- The traffic evaluator must show all three lamp states in validation evidence. During route encounters, there must be **zero false-GREEN authorizations**. Unknown, stale, conflicting, or low-confidence states are STOP.
- A crossing is authorized only after at least **3 consecutive** fresh camera classifications of GREEN, agreement with the simulated signal state, and at least **2.0 s** remaining GREEN time. Keep the signal stream fresh within the 500 ms age limit above.
- On RED, YELLOW, or unknown/stale perception, the chassis front edge remains at least **0.04 m** before the stop line (target **0.10 m**). No chassis footprint crosses the line. Once legally committed on GREEN, clear the rear edge at least **0.04 m** beyond it before stopping. Record camera state, simulated state, remaining time, chassis pose, and gate status at each encounter.
- Near a controlled line, cap approach speed at **0.12 m/s**; never exceed the configured **0.35 m/s** navigation cap. ENFORCE_TRAFFIC=false runs are diagnostics and cannot count toward this gate.

### 3. Zero lane-marking intrusion

- Require **zero** overlap between the full chassis footprint and prohibited ordinary lane-paint pixels, and **zero** crossings of an ordinary lane boundary, in every run. Treat touching a prohibited pixel as a failure; do not excuse actual contact by map-grid tolerance. The declared start-line exception and GREEN-gated stop/zebra masks are evaluated separately under their stated rules.
- Evaluate recorded motion using the full rotated chassis polygon, with Gazebo pose used only for post-run evaluation, never navigation input. Interpolate between recorded poses so translation gaps are **≤1 cm** and yaw gaps are **≤1°**; report minimum paint clearance and every exception crossing. Check both the occupancy overlay and watchdog texture mask, with the swept footprint.
- The path must stay within the approved ordered corridor. A globally collision-free shortcut that leaves the competition route is a failure.

### 4. Not too slow; no stall

- Measure progress along the ordered route corridor, not raw odometry distance (which can be inflated by loops). Per run, route progress divided by active elapsed time must be **≥0.15 m/s**. Exclude only recorded red/yellow/unknown signal waits and the fixed photo-capture dwell configured before Run 1; do not exclude turn time, recovery time, or unexplained pauses.
- Outside those declared waits/dwells, fail the run if corridor progress is **<0.05 m for 8 consecutive seconds** on a navigation leg. Report every per-leg time, distance, progress speed, and stop interval.
- Enforce the **0.35 m/s** maximum speed above; performance thresholds must not be met by unsafe speed or by counting repeated/backtracking distance.

### 5. Return to birth pose and evidence

- At HOME, compare the chassis pose to the recorded birth pose. Position error must be **≤0.03 m**, yaw error **≤0.04 rad**, and linear/angular velocity each **<0.01** (m/s and rad/s) continuously for **2 s**. Record both localization pose and Gazebo ground truth; use ground truth only as an evaluator.
- Each run produces a closed, readable bag; route/action log; per-point raw and annotated images; detection/traffic logs; paint-clearance audit; start/HOME error; and a manifest of code, map, model, and config hashes. Incomplete evidence makes that run a failure.

## Consecutive-run and reset rule

Track the streak as `0/3`, `1/3`, `2/3`, `3/3`. A run counts only if every condition above passes. **Any single failure resets the streak to `0/3`**; it does not merely subtract one. Stop safely, retain and label the failed run, return/reset to the declared birth pose under the normal launch procedure, and start a new complete ten-point-plus-HOME run. Never resume mid-route or repair/retry one point and call it the same passing run. Any change to code, map, saved point, route contract, model, confidence threshold, or safety parameters also resets the streak to `0/3`. Three is achieved only after three successive full passes on the same frozen build with no intervening diagnostic route or configuration change.

## Is the task order appropriate?

Yes, with one documentation conflict to fix:

1. **task006 — navigation safety foundation:** finish the no-goal costmap, texture/grid, footprint, watchdog, and corridor audit. The approved task006 plan explicitly prohibits route goals. The roadmap currently says task006 must also run a bagged autonomous route; remove or move that item, because it contradicts the task006 plan and would start motion before perception/traffic integration.
2. **task007 — live perception and traffic gate:** integrate the frozen `best.pt` detector, validate real-time timing and annotations, and verify fail-closed traffic authorization against RED/YELLOW/GREEN. Keep OCR out of agent scope; expose plate boxes/crops to the user. Update stale “Detector and OCR” labels/references accordingly.
3. **task008 — end-to-end acceptance:** only after task006 and task007 reviews pass, execute the frozen route three consecutive times against this gate, with independent evidence per run.

This ordering makes task006 prove geometry and constraints, task007 prove the live perception/gate dependency, and task008 prove the integrated mission. The existing roadmap's “one line-compliant run” under task006 should be moved to task008 rather than treated as a prerequisite evidence run.

## Re-review condition

Create a task008 plan that adopts these thresholds (or records a measured, stricter replacement before the streak begins), explicitly implements the `0/3` reset rule, and resolves the task006 roadmap conflict. No OCR criterion is included in the three-run agent gate.
