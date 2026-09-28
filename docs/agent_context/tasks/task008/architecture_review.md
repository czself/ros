---
task_id: task008
type: architecture_review
reviewer: independent_architecture_reviewer
decision: CHANGES_REQUESTED
---

# Task 008 Architecture Review

## Decision

**CHANGES_REQUESTED.** The three-consecutive-run ledger, reset-to-0/3 rule, OCR exclusion, and quantitative perception/traffic/paint/photo conditions are clear. Before the streak begins, align the plan with the current launch and executor behavior; several current defaults violate its gate.

## Required revisions

1. **Force the traffic gate on in the acceptance launcher.** `start_navigation.sh` currently defaults `ENFORCE_TRAFFIC=false`, and `start_standee_photo_route.sh` passes only `ENFORCE_WHITE_LINES`. The task008 gate requires both gates enabled. Add an acceptance-specific launch contract that sets `ENFORCE_TRAFFIC=true` and refuses to start the route if the watchdog parameter, YOLO node, or required fresh observation stream is absent. Do not rely on an operator remembering an environment variable.

2. **Align retry and no-progress behavior with the stated pass rules.** The current photo launcher sets `_no_progress_timeout:=20.0`; `navigate_goal()` permits two attempts and clears costmaps/retries after a failed goal. Its progress heartbeat also counts yaw change as progress, whereas task008 measures forward progress along the route corridor. Task008 fails a leg at 8 seconds and disallows aborted/retried goals. Add an acceptance-mode corridor-progress monitor that cancels motion and fails the run when corridor progress is under 0.05 m for 8 seconds outside declared waits/dwells; allow only one attempt. The evidence checker must also treat any retry/action reissue as a failed run.

3. **Make HOME acceptance stricter than the executor's current success condition.** `precise_park()` currently declares `COMPLETE_PARKED` at up to 0.06 m and 0.06 rad after a 0.5 s settle, while task008 requires 0.03 m, 0.04 rad, and 2 continuous seconds below the velocity limits. Tighten the executor/acceptance monitor to these limits and evaluate actual Gazebo chassis pose as evaluator-only ground truth as well as the recorded localization estimate. A loose `COMPLETE_PARKED` status must not pass the stricter gate.

4. **Enforce or actively monitor the route corridor during photo-route execution.** In photo mode, `route_executor` loads only the ten saved photo poses and sends them to `move_base`; the planner may choose a free shortcut between them. The plan's center-path condition is not guaranteed by the current launcher or executor. Add the approved `inner_route` corridor to acceptance execution and either constrain the path to it or run a live corridor monitor that cancels and fails the run on departure. Retain the swept-footprint and post-run audit; endpoint validation alone is insufficient.

5. **Use task007's same-stamp evidence interface.** Current `capture_photo()` stores an unstamped latest raw frame. Task008 requires a matching raw image, YOLO annotation, and detection JSON. Make successful task007 review a prerequisite that confirms the route executor/capture coordinator consumes all three by the same source stamp and the required frame-margin/count checks are enforced. Incomplete or mismatched evidence remains a run failure.

6. **Bound and review any P5/P7 pose adjustment before freezing.** The pre-freeze diagnostic is useful, but “authorized small adjustment” has no numeric bound or independent decision rule here. State the maximum measured displacement allowed (or require an explicit separate reviewed task plan for any change), require P5/P7 image evidence for the proposed change, re-run the task006 corridor/footprint audit, and freeze the resulting point-file hash before Run 1. Any later change correctly resets the streak.

## Review notes

- OCR is properly excluded from the agent's 3/3 gate. The plate box, confidence, same-stamp annotation, and crop still need to pass; character recognition remains user-owned.
- The specified 5 Hz, 250 ms p95, and 500 ms frame-age limits are consistent with task007 if “latency” is defined identically as source stamp to published result.
- The 0.15 m/s corridor-progress metric, 8-second stall limit, ≤0.35 m/s cap, and ≤0.12 m/s stop-line approach are measurable. The current launcher/executor must be configured to enforce the 8-second abort behavior rather than only detect it later.
- The streak/failure rule is sound: any failed run or frozen-artifact change resets to 0/3, and no partial continuation counts.

## Re-review condition

Revise the plan with an acceptance launcher/executor profile that enforces the gate, one-attempt/8-second rule, strict HOME tolerance, and corridor constraint/monitor; resolve the bounded point-adjustment approval. No Gazebo run was performed or authorized by this review.
