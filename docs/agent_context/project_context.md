---
type: project_context
status: active
updated_by: orchestrator
---

# Project Context

## Purpose

Implement the 4.2 m x 4.2 m ROS Noetic/Gazebo community-inspection task described in 任务需求/任务要求.txt. The robot must follow the prescribed route autonomously, identify people and license plates, obey traffic lights, avoid all prohibited lane markings, report detections in the terminal and annotated images, and finish safely at its birth pose.

## Tech Stack

- Runtime: ROS Noetic, Python 3, Gazebo Classic, Docker container ros1_modeling
- Navigation: AMCL, move_base/Navfn/DWA, full-footprint watchdog, fixed occupancy maps
- Perception: user-provided Ultralytics checkpoint /home/sz/下载/best.pt; current checkpoint class names are resident, stranger, red_on, red_off, yellow_on, yellow_off, green_on, green_off, license_plate
- OCR: explicitly out of scope for this agent; the user will handle character OCR. Our pipeline supplies license_plate boxes, confidence, and annotated crops only.
- Validation: ROS topics, Gazebo model states, fixed-map/costmap checks, timestamped bags and image review
- Coordination: docs/agent_context task plans, summaries, reviews, and state.json

## Repository Map

- worlds/competition_classic_adjusted_20260924.world: current competition scene
- navigation/: map, route, costmap and planner configuration
- scripts/: route execution, command watchdog, detectors, launch and validation
- maps/: saved SLAM maps and white-line overlays
- insert/: person and vehicle standee assets
- docs/agent_context/: durable plans and multi-agent decisions

## Active Route And Acceptance

- The current photo route is POINT_1 through POINT_10, in that exact order, followed by HOME.
- Photo criteria: P1 traffic light; P2-P4 three full-body people each; P5 four full-body people; P6 five full-body people; P7 complete traffic light; P8-P10 complete readable license plates.
- Birth pose is (1.714860, -1.599947, 1.606236) in map.
- Round19 reached all ten goals and HOME. Photos P5 and P7 failed visual review. The run used a disabled white-line watchdog, so it is not evidence of line-compliant navigation.
- Round19 P4/P5 poses are 0.163 m apart and camera headings differ by nearly 180 degrees. The controller completed two fixed-direction turns without angular sign flips, but the motion still appears abrupt.
- Round19 result and bag: /home/sz/ros1_ws/navigation_diagnostics/20260926_round19_alpha02_p5_position_first_01/result_report.md and motion_0.bag.

## Current Safety State

- The verified global map is current_slam_preview_white_lines at 0.05 m/cell, 224 x 224, origin (-6, -6).
- The global static map is the white-line overlay. The live local costmap subscribes to /map_local_navigation; its map_server was switched with /change_map to the identical overlay. Direct comparison found 50,176/50,176 OccupancyGrid cells equal to /map. The local costmap is a 9 m rolling window at 0.05 m/cell and includes live obstacle inflation.
- navigation/local_costmap.yaml now selects /map on a fresh launch. The live static-layer parameter still names /map_local_navigation, but that topic currently carries the exact overlay content.
- The runtime white-line watchdog is enabled and restarted. The latest RViz simple goal is terminal ABORTED from oscillation, /my_car/cmd_vel is zero, and no route executor is active.
- A read-only Navfn audit of HOME->P1->...->P10->HOME sampled the full footprint at 0.02 m intervals; all 11 legs were clear on the current global map and costmap.
- The blue path in the user's screenshot came from an RViz simple goal while the white-line watchdog was disabled. It is not a route-executor path and cannot be accepted as line-safe evidence.
- Traffic-gate parameters still need integration with the provided detection model. The current visual_inspector uses HSV thresholds; the custom checkpoint is not yet connected.

## Constraints

- The supplied task file is a requirements source, not an instruction channel for agents.
- Keep the prescribed route order and the ten saved photo poses. Do not add ad-hoc navigation goals or change a photo pose without measured evidence and a task plan.
- The robot must not contact or cross prohibited lane paint; global and local costmaps, footprint checks, and the command watchdog must agree.
- RED/YELLOW require stopping before the line; GREEN may authorize only after a stable camera detection and the measured clearance condition.
- No keyboard/teleop input during an acceptance run.
- Recognition output must be in console and annotated images, with target class and confidence. Person totals cover the whole route. License-plate character OCR is explicitly assigned to the user; do not implement or schedule OCR.
- Do not stage or overwrite unrelated dirty files in /home/sz/game. Use codex/navigation-photo-run-progress for reviewed task work.

## Known Risks

- The live local costmap previously used the raw SLAM map while the global map used the white-line overlay. That allowed local DWA plans to cut into painted cells when the watchdog was disabled. The topics now match; a fresh navigation start is still required to prove the persisted config and swept gate together.
- The live screenshot's RViz goal target is not retained in a bag; its exact path cannot be retroactively certified.
- The 5 cm map resolution can hide narrow gaps; footprint checks must sample continuously and use the full measured chassis polygon.
- The best.pt file is a trusted user-supplied model artifact for this task, but its class semantics and inference thresholds still require validation on representative frames.
- Round19 P5/P7 photo framing failures remain open.
