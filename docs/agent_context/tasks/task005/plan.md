---
task_id: task005
type: plan
status: architecture_review
from: planner
to: coder
revision: 0
requires_review: true
---

# Calibrated inner route and independently observed closed-loop navigation

User authority: implement full workflow and navigation; ignore signals but never ordinary paint; latest annotated screenshot `/tmp/codex-clipboard-1oyA4K.png` supersedes previous invented routes. Start and finish are the identical Gazebo birth pose.

## Scope and architecture

1. Calibrate the actual ground texture against its authored/live transform using parking bays, L-shaped block and both crossings. Trace the screenshot's up, left, down, left, down, right, down route. Publish preview and fixed ordered route. Reject offline footprint or rotation collisions before motion.
2. Fix signal-bypass white-paint exception, ensure a single guarded final velocity publisher, fail closed for missing/stale sensors/pose. Sensor readings confirm obstacle-free approaches, while painted-road geometry supplies junction topology: flat paint cannot be identified from range alone.
3. Use ordered segment tracking, heading alignment and bounded speed through the guard. An independent raster footprint monitor and corridor/progress monitor must check actual Gazebo chassis samples throughout the run. Preserve laser and depth obstacle vetoes. No teleport except pre-run birth reset; no unguarded velocity publisher.
4. Save planned-versus-actual trajectory visualization, timestamped raw samples, turn sequence, contact count, completed segments, home error, sensor freshness and runtime authority. Completion requires all segments traversed in order, zero ordinary-paint contact, no forbidden excursion, stopped within 0.08 m of birth. Signals explicitly ignored, not a traffic-compliance claim.

## Verification

Regression tests must fail for ordinary-paint bypass under ignored signals, unsafe footprints, invalid route order and premature completion. Perform geometric offline preflight, ROS sensor/ownership checks, continuous simulation run and independent evidence review. On a violation stop, preserve failure evidence, revise before another run. Do not report point-goal success as route success.

## Deliverables

Calibrated contract, geometry helper, corrected gate, guarded controller/monitor, reproducible launcher, tests, preview and actual-track artifacts, coder summary and independent review. Preserve unrelated files. Do not change scene geometry or paint to force passage. Old task002 remains incomplete/superseded until this task's evidence is reviewed.
