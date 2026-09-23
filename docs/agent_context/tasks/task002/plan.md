---
task_id: task002
type: plan
status: ready_for_implementation
from: planner
to: coder
revision: 0
requires_review: true
---

# Task 002 Plan: Freeze the 4.2 m Autonomous Route Contract

## Status

Ready for implementation.

## Owner

Implementation Agent.

## Reviewer

Planner/Reviewer Agent.

## Context

Traffic-light visuals and final stop-line coordinates are complete. Existing
navigation scripts still use the obsolete +/-4 m map and random patrol goals,
so they cannot drive the required diagram path.

## Goal

Create a durable, measured route contract for one 4.2 m loop: robot birth
pose, ordered waypoints, heading at each waypoint, lane corridor limits, and
the two stop-line encounters. Prepare the map/navigation configuration that
task003 will consume.

## Relevant Files

- `worlds/competition_classic.world`
- `navigation/`
- `maps/`
- `scripts/start_navigation.sh`
- `docs/agent_context/tasks/task001/artifacts/stop_line_measurements.yaml`

## Required Behavior

- The route follows the supplied diagram arrows and stays inside painted lanes.
- No chassis corner, wheel, or body edge may contact or cross either white lane boundary.
- A stop line may be touched or crossed only while the recognized signal is GREEN.
- RED and YELLOW require the complete chassis to remain before the stop line.
- It starts at the robot's actual 4.2 m world pose and visits both traffic
  stop-line approaches in their travel direction.
- Route coordinates are measured and versioned, not inferred from legacy maps.
- The navigation configuration is bounded to the 4.2 m world.

## Required Changes

- Inspect the ground texture and final scene, then record the ordered route in
  a machine-readable route artifact.
- Record map origin, resolution, robot footprint, initial pose, headings, and
  lane/stop-line tolerances.
- Update or replace obsolete navigation launch/configuration files only where
  needed to use the measured scene.
- Add a non-moving validation that verifies every route segment stays inside
  its permitted lane corridor and reaches both stop-line approaches. The
  validator must check the complete vehicle footprint against white lines.

## Verification

```bash
xmllint --noout worlds/competition_classic.world
python3 -m py_compile scripts/*.py
./scripts/start_sim.sh --restart
```

Live verification must report the robot birth pose and the two route approach
poses from Gazebo; route execution itself is task003.

## Documentation Requirements

- Add route measurements and coordinate conversion notes under
  `docs/agent_context/tasks/task002/artifacts/`.
- Record all measurement commands and resulting coordinates in `summary.md`.

## Out Of Scope

- Person recognition, person statistics, plate OCR, and any associated image
  or terminal recognition output.
- Implementing the driving state machine (task003).
- Changing the final user-placed traffic-light positions.

## Acceptance Criteria

- A reviewed route artifact contains start, ordered waypoints, headings, lane
  corridors, and both stop-line approaches in 4.2 m world coordinates.
- Navigation configuration has no active +/-4 m legacy assumptions.
- Deterministic validation passes for all route segments.
- White-line contact/crossing is rejected by validation.
- Stop-line crossing is rejected for RED/YELLOW and permitted only for GREEN.
- The task summary records reproducible measurement evidence.
