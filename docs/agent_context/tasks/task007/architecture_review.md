---
task_id: task007
type: architecture_review
reviewer: independent_architecture_reviewer
decision: CHANGES_REQUESTED
---

# Task 007 Architecture Review

## Decision

**CHANGES_REQUESTED.** The plan correctly keeps OCR user-owned and defines useful live-rate, age, and confidence targets. Three runtime interfaces are underspecified or conflict with the current watchdog, readiness check, and photo capture path; resolve these before implementation approval.

## Required revisions

1. **Specify one timestamped, confidence-bearing traffic observation consumed by the watchdog.** The current watchdog subscribes to `/inspection/traffic_light` as `std_msgs/String`, increments its GREEN count on message arrival, and timestamps receipt locally. It does not consume confidence or the source camera stamp. Publishing a separate confidence topic without an atomic pairing contract would allow a stale or mismatched GREEN to authorize motion. Specify a single typed observation (or an equally unambiguous synchronized contract) containing source `Header.stamp`, state, confidence, and—if needed—lamp-level classes. Update the watchdog to reject observations below the frozen confidence floor, older than 500 ms, out of order, or inconsistent/ambiguous. Keep its three-consecutive-GREEN, simulated-state, and remaining-time checks on those same source-stamped observations.

2. **Make acceptance startup actually enable the traffic gate and start/check YOLO.** `start_navigation.sh` currently defaults `ENFORCE_TRAFFIC=false`; `start_standee_photo_route.sh` forwards only the white-line setting. `check_navigation_readiness.py` currently checks the white-line flag but not the traffic flag or detector outputs. Add an acceptance launch path that sets `ENFORCE_TRAFFIC=true`, starts the detector after the navigation stack is ready, then runs an extended readiness check after both are up and before the route executor starts. That check must fail closed unless the watchdog and YOLO node are live, source-stamped detections/annotations are fresh, and the traffic gate is enabled. Keep a clearly separated diagnostic mode; it must never be treated as acceptance.

3. **Connect the same-stamp photo contract to the route executor.** The current executor subscribes to raw `/camera/image_raw` into an unstamped `latest_image` and `capture_photo()` saves that latest raw image. It does not select an eligible sharp frame or match the annotation and detection JSON. Specify the route-capture interface and update the executor or a dedicated capture coordinator to atomically select raw image, annotated image, and detections by the same source stamp; a missing/mismatched member or failed content check must fail that photo and the run. Also specify how the required box counts and 5% frame margins are checked against the selected image.

## Review notes

- OCR is explicitly excluded and the plan still requires plate boxes, confidence, annotated crops, and handoff of the crop. This matches the current user scope.
- The confidence floor of 0.25 is consistent with the saved-frame audit. Freeze its meaning and the exact latency interval as source-image stamp to published result, then use that same definition in task008.
- The archived Round19 photos may be used to check model classes, but they cannot count as passing capture evidence: P5/P7 had known framing failures, and the old route did not run with the white-line gate enabled.

## Re-review condition

Revise the plan to define the watchdog observation schema/source-time freshness, the acceptance launcher/readiness requirements, and the stamped photo-selection contract. No Gazebo route or task008 streak is authorized by this review.
