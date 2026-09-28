---
task_id: task006
type: plan
status: ready_for_implementation
from: planner
to: coder
revision: 1
requires_review: true
---

# Task 006 Plan: White-line-safe global and local navigation

## Status

Revision 1 responds to the independent review's CHANGES_REQUESTED decision. Re-review this plan before implementation. Do not dispatch a route goal in task006.

## Owner

Implementation Agent: route_run

Reviewer: route_diagnosis

## Context

The task requirement forbids crossing lane boundaries. The latest RViz screenshot showed a local path passing a narrow gap in a painted boundary. The goal came from RViz, not the ten-point route executor. At that time the local costmap used raw /map_local_navigation and white-line enforcement was disabled. The watchdog is now enabled. The live local map server was switched to the white-line overlay; /map and /map_local_navigation were then compared cell-for-cell and match (50,176/50,176). The runtime local static-layer parameter still names /map_local_navigation; a fresh-launch config points directly to /map.

Round19 completed the ten points and HOME but ran with the line watchdog disabled; it cannot be accepted as line-compliance evidence. A read-only Navfn audit of HOME, the ten points, and HOME again sampled the full footprint every 2 cm against the global map/costmap; all 11 legs were clear. A recent RViz goal ended with an oscillation abort and zero drive command. No goal is active.

## Goal

Make both global planning and local DWA scoring see the same white-line-safe map, keep the full-footprint watchdog enabled at runtime, and prevent any photo-route launch from disabling that gate. Record a read-only path audit for all fixed route legs.

## Relevant Files

- navigation/global_costmap.yaml
- navigation/local_costmap.yaml
- navigation/common_costmap.yaml
- scripts/cmd_vel_watchdog.py
- scripts/start_navigation.sh
- scripts/start_standee_photo_route.sh
- scripts/check_navigation_readiness.py
- scripts/route_executor.py

## Required Behavior

- Global and local costmaps consume identical verified 0.05 m/cell white-line occupancy data; a fresh launch should use /map for both.
- Both costmaps use the measured 19 cm x 17 cm footprint and reject unknown/lethal cells.
- The watchdog remains the sole navigation-mode command publisher and checks the complete swept footprint before forwarding motion.
- Startup and route execution fail closed if the white-line gate is disabled or the map/pose safety inputs are unavailable.
- Line policy is explicit: ordinary lane boundaries are always forbidden; the start line is only the measured departure exception; stop-line and zebra traversal require stable camera GREEN, matching simulation state, and sufficient remaining GREEN time. ENFORCE_TRAFFIC=false is diagnostic-only and cannot produce acceptance evidence.
- No goal is sent during this task. The robot stays stopped while configuration and paths are audited.
- Preserve the ten saved photo poses, their order, and HOME. Do not add points or use Gazebo truth as a navigation input.

## Required Changes

- Point local static_layer at /map, matching the global white-line map.
- Enable track_unknown_space on both global and local costmaps so neither planner treats space outside the mapped lanes as traversable.
- Keep white-line enforcement enabled in both navigation launchers; reject an explicit false setting for this route.
- Check the live watchdog parameter in navigation readiness and route-executor startup.
- Add a reproducible validator comparing the occupancy overlay to the texture-derived watchdog mask, including transforms, conditional exception masks, boundary discrepancies, and swept-footprint samples.
- Save a reproducible report of map frames/resolution, effective static-layer map sources, footprint, gate state, goal/velocity state, and each of the 11 Navfn plans.
- Audit the Navfn legs against the ordered corridor in navigation/inner_route.yaml. The fixed photo poses must project monotonically onto that route, and each inter-photo plan must stay within its corridor rather than shortcutting the diagram.

## Verification

```bash
bash -n scripts/start_navigation.sh scripts/start_standee_photo_route.sh
python3 -m py_compile scripts/check_navigation_readiness.py scripts/route_executor.py
```

Then, without dispatching a goal, start a fresh navigation stack and inspect /map, /map_local_navigation, both costmaps, their live static_layer/map_topic parameters, footprint topics, /cmd_vel_watchdog/enforce_white_lines, /cmd_vel_watchdog/enforce_traffic, /move_base/status, and /my_car/cmd_vel. Compare source occupancy cells and the texture-derived ordinary-paint mask, applying only the declared start/stop/zebra exceptions. Run read-only make_plan requests for HOME->POINT_1->...->POINT_10->HOME, project paths onto the ordered inner-route corridor, and sample the full rotated footprint at no more than 2 cm intervals against both occupancy maps and the watchdog texture mask. State how exact-cell boundary contact is handled and record every pass/fail.

## Documentation Requirements

Write the implementation result to this task's summary.md and the independent decision to review.md. Update project_context.md, roadmap.md, todo.md, and state.json only after reviewer decision.

## Out Of Scope

- Do not change route or photo coordinates, image acceptance criteria, or world geometry.
- Do not run the ten-point route or send RViz goals as part of this task.
- Do not integrate best.pt here; task007 covers detector boxes and annotations only. OCR is explicitly handled by the user and is out of scope for all agent tasks.
- Do not enable/disable traffic-signal logic here; task007 will connect perception to the existing stop-line gate.

## Acceptance Criteria

- Both live costmaps use the same white-line overlay and measured footprint; fresh-launch parameters are recorded.
- White-line gate is enabled in the running watchdog and cannot be disabled by the photo-route launcher.
- Texture-derived ordinary-paint cells and occupancy cells agree within the declared grid tolerance; conditional exception pixels are separately reported.
- Every fixed route leg has a nonempty plan, advances along the prescribed corridor, and the full footprint remains clear at 2 cm samples on both costmaps and the watchdog texture mask.
- Any run with ENFORCE_TRAFFIC=false is labeled diagnostic-only and cannot claim red-light or full-task acceptance.
- At verification time there are no active goals and /my_car/cmd_vel is zero.
- No route goal was dispatched during this task.

## Revision 1 — Response to architecture review

- Added an explicit ordinary-lane/start-line/stop-line/zebra exception matrix and labeled traffic-bypassed runs diagnostic-only.
- Added fresh-launch runtime readback of the local static-layer map source.
- Added texture-to-grid transform/boundary comparison and full-footprint sweep verification against both representations.
- Added a corridor audit against navigation/inner_route.yaml so point-to-point Navfn success cannot certify a shortcut.
