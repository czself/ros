---
task_id: task001
type: plan
status: ready_for_implementation
from: planner
to: coder
revision: 2
requires_review: true
---

# Task 001 Plan: Traffic-Light Compliance

## Status

Ready for implementation. This revision supersedes the earlier route-input
plan after the user made traffic-light compliance the immediate priority.

## Owner

Implementation Agent.

## Reviewer

Planner/Reviewer Agent.

## Context

The 4.2 m world contains two synchronized lights and two painted stop lines.
The supplied six lamp photographs and 64 x 48 cm dimensions are authoritative.
The current random patrol does not enforce signal rules.

## Goal

Render each signal from the supplied front photograph on the route-facing side
and guarantee that navigation commands cannot move the vehicle body onto a
stop line during RED.

## Relevant Files

- `worlds/competition_classic.world`
- `models/traffic_light/model.sdf`
- `models/traffic_light/materials/`
- `scripts/traffic_light_controller.py`
- `scripts/cmd_vel_watchdog.py`
- `scripts/visual_inspector.py`
- `scripts/start_sim.sh`
- `scripts/start_navigation.sh`

## Required Behavior

- Both lights synchronously cycle RED 10 s, GREEN 15 s, YELLOW 3 s.
- Six supplied photos map one-to-one to active/inactive lamp materials.
- Each 14 cm lamp photo is visible only on the side facing the route.
- RED and YELLOW stop the entire chassis before either measured stop line.
- Motion is authorized only by stable camera GREEN plus a non-conflicting
  simulation-state veto and at least two seconds of remaining GREEN.
- A vehicle admitted on GREEN clears the line rather than stopping on it.
- Stale or missing perception fails safe to STOP.

## Required Changes

- Use 14 x 0.8 x 14 cm photo-faced boxes and exact source images. The thin box avoids the route-facing backface/occlusion failure seen with Ogre planes while the black housing masks the reverse side.
- Keep exactly two complete traffic-light models in the authored world and at runtime; do not hide inactive signal copies below the floor.
- Keep robust synchronized switching and health/countdown topics.
- Add non-bypassable stop-line gating at the final velocity watchdog.
- Publish gate decision and braking topics for terminal/video evidence.
- Add deterministic tests for geometry, cycle order/timing, and gate decisions.

## Verification

```bash
python3 -m py_compile scripts/traffic_light_controller.py scripts/cmd_vel_watchdog.py scripts/visual_inspector.py
bash -n scripts/start_sim.sh scripts/start_navigation.sh scripts/start_patrol.sh
xmllint --noout worlds/competition_classic.world models/traffic_light/model.sdf
./scripts/start_sim.sh --restart
```

Live verification must observe a full RED -> GREEN -> YELLOW -> RED cycle and
record both light model heights plus the stop-line gate decision.

## Documentation Requirements

- Record exact stop-line coordinates and how they were measured.
- Record commands and pass/fail output in `summary.md`.

## Out Of Scope

- Full fixed-route waypoint implementation.
- Rebuilding SLAM/navigation maps.
- Person counting and plate OCR.

## Acceptance Criteria

- Exact source-image hashes match deployed textures.
- The supplied lamp photo is visible from the route-facing side and the housing masks the reverse side.
- The authored world and steady-state runtime each contain exactly two complete traffic-light models.
- Cycle timing is within 0.25 s per phase.
- Unit tests prove RED/YELLOW/missing detection stop before both lines and
  stable GREEN permits crossing.
- Live Gazebo reports both signals synchronized through a full cycle.
