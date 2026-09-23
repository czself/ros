---
task_id: task005
type: review
status: approved
from: architecture_reviewer
to: orchestrator
revision: 0
decision: APPROVED
next_action: implementation
---

# Architecture review

APPROVED for implementation of task005 plan, not runtime acceptance.

The calibrated shared paint geometry, ordered route corridors, sole guarded
velocity publisher and independent recorded trajectory address the observed
failure modes. Range sensors validate physical clearance; they cannot identify
flat painted junctions. Static calibrated line geometry supplies that topology.

## Findings in existing implementation

- Critical: `permitted_stop_line_pixel` returned true at every coordinate when
  signals were ignored, admitting all ordinary paint through the white gate.
- Critical: direct route driver published raw velocity and reported completion
  even after boundary abort or shutdown.
- Local costmap omitted the static layer and therefore painted walls.
- Legacy inner navigator chose five generic left openings, unrelated to the
  screenshot route. Existing route executor checked endpoint continuity only.
- Missing/stale pose was not fail-closed. Footprint point sampling omitted exact
  positive edges and could miss narrow paint. Motion prediction neglected curved
  translation and the chassis-to-drive-axle displacement of 0.0525 m.

## Acceptance gates

Verify calibrated world/map/texture transforms; conservative filled full-body
and swept-turn clearance; geometry-bounded traffic exemptions; one raw velocity
publisher with fresh pose and sensor vetoes; ordered corridor progress; and a
saved independent actual-trajectory audit. Completion requires every segment,
zero ordinary-paint contact or corridor departure, and a stopped return within
0.08 m of the identical starting position. An action success count is insufficient.

## Reviewer verification

Read task005 plan, project context, architecture analysis, relevant controller,
guard, map-builder, odometry and costmap source. Review was read-only; no motion
was commanded. Implementation and runtime evidence require subsequent review.
