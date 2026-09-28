---
task_id: task008
type: parallel_minimum_fix_review
reviewer: independent_architecture_reviewer
decision: GUARDED_DIAGNOSTIC_RERUN_ONLY
---

# Minimum Fix Review — Rerun Readiness

## Decision

**The four requested fixes are sufficient to attempt one guarded diagnostic rerun through `start_standee_photo_route.sh`; they are not enough to certify a 3/3 acceptance run.** This is static review only; no simulation or route goal was run.

## Fixed for a guarded rerun

- RViz Image displays now use the serialized `Image Topic` key for both RGB and depth feeds.
- `route_executor` caches raw frames, annotated frames, and detection JSON by the same `(secs,nsecs)` stamp under one lock, then selects only matched records. The capture callback and snapshot use the lock around these cache operations.
- The photo-route wrapper defaults `STRICT_ACCEPTANCE=true`, rejects either gate set false, and always passes `ENFORCE_WHITE_LINES=true` and `ENFORCE_TRAFFIC=true` to navigation. Navigation also defaults both gates on; readiness requires both gates and a fresh, same-stamp YOLO stream. `STRICT_ACCEPTANCE=false` remains a diagnostic profile while both safety gates stay on, and the run summary records that flag.
- The watchdog consumes the structured traffic JSON and checks confidence, source age, checkpoint hash, three consecutive states, simulated signal state, and GREEN time. The route executor refuses strict mode if either runtime gate parameter is false.

## Must fix before counting any run toward 3/3

1. **The 8-second stall rule is not yet route-progress enforcement.** The executor's heartbeat refreshes on either 2.5 cm odometry displacement or 0.06 rad yaw change. A robot can keep spinning and avoid the 8-second no-progress failure even while making no ordered-corridor progress. Before a streak, make the strict route monitor measure corridor-projection progress (`<0.05 m` in 8 s) and cancel/fail on the threshold. A single guarded diagnostic rerun can still be attempted, but any long spin or repeated oscillation is a failed diagnostic and must stop safely.
2. **`COMPLETE_PARKED` alone is weaker than the acceptance rule.** Strict parking checks AMCL at 0.03 m / 0.04 rad after a 0.5 s settle; it does not itself establish 2 continuous seconds of low velocity or the Gazebo-truth pose error. Keep those as mandatory bag-based acceptance checks, or add them to the strict finish monitor before claiming a pass.

## Can be deferred from a navigation diagnostic, not from final acceptance

- The route executor calculates the “unique” street-wide person total by summing the five per-point counts (and expects 18). This is not deduplication. It does not prevent a route diagnostic with the expected per-photo counts, but the final report must use the authored unique standee inventory or a reviewed deduplication method.
- Round19 already missed the P5 four-person and P7 full-signal framing criteria. The strict photo validator will stop at the first point whose count/margin fails, so a diagnostic may stop before later points until those views are corrected.
- `STRICT_ACCEPTANCE=false` still keeps both gates on, but the wrapper can exit successfully with `COMPLETE_PARKED`; preserve the `strict_acceptance:false` marker and label that artifact diagnostic-only. Do not add it to the acceptance ledger.
- No runtime evidence is available yet for RViz rendering, actual cache stamp coverage, YOLO throughput, or gate operation. Readiness checks reduce startup risk but do not replace validating the resulting bag and photos.

## Re-run boundary

Use the wrapper with its strict default and gates enabled for a controlled diagnostic. Do not claim task acceptance from `COMPLETE_PARKED` or a single run. The three-run streak remains blocked on task006/007 reviews and the strict corridor-progress and final-pose evidence above.
