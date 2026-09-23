---
type: architecture_analysis
status: reviewed
updated_by: planner
review_required: true
---

# Architecture Analysis

## Current Shape

Gazebo hosts the 4.2 m scene, `my_car`, and exactly two traffic-light models. `traffic_light_controller.py` switches both signals by moving their internal ON/OFF photo panels between the route-facing surface and the opaque housing, and publishes state, remaining time, and readiness. `visual_inspector.py` publishes camera detections. `move_base` publishes `/my_car/cmd_vel_nav`, while `cmd_vel_watchdog.py` is the only navigation-mode publisher to Gazebo.

## Target Shape

2026-09-22 reviewed task005 supersedes the route implementation below for the
requested ignored-signal demonstration: calibrated texture geometry defines
the screenshot's inner corridors. `calibrated_navigation` tracks ordered
segments using simulated chassis localization and live laser/depth obstacle
vetoes; `cmd_vel_watchdog` exclusively owns raw Gazebo velocity and checks
filled footprint/swept paint. World/map identity is explicit in this simulation
mode, not an unproven AMCL transform. Planned and actual paths are published
in odom/world. Independent recorded-trajectory audit determines acceptance.
See tasks/task005/architecture_review.md. This does not claim depth-only
junction recognition, AMCL navigation completion, or red-light compliance.

A deterministic route executor will follow the supplied arrows. The final velocity safety boundary will combine chassis pose, traffic-light camera detection, signal-state veto, and two measured stop-line gates. It will stop the full body before RED/YELLOW, admit entry only on a stable GREEN with clearance time, and allow an admitted vehicle to clear the line. Person and plate recognition are explicitly out of scope.

## Key Decisions

- Use explicit route waypoints rather than random costmap sampling.
  Rationale: the task specifies a route and must be reproducible.
- Keep switching, perception, and motion gating as separate ROS contracts.
  Rationale: each can be tested independently, while the watchdog remains a non-bypassable final safety boundary.
- Keep exactly two complete signal models at runtime instead of burying inactive full-model variants.
  Rationale: hidden duplicates remained visible below the floor in the Gazebo editor and made the authored scene misleading.
- Require camera GREEN to authorize motion; use simulation state only as a veto and timing guard.
  Rationale: this demonstrates recognition while preventing a false-positive from violating the red-light rule.
- Measure line crossings against chassis edges rather than the model origin.
  Rationale: the requirement applies to the complete vehicle body.
- Keep one node responsible for task orchestration, while vision nodes only publish detections.
  Rationale: this makes detection evidence and driving decisions testable independently.

## Risks And Tradeoffs

- A hard stop can interfere with local-planner progress timers.
  Mitigation: keep the navigation goal active while the final command gate emits zero, then resume on GREEN.
- Stopping after GREEN expires could strand the body on the line.
  Mitigation: require at least two seconds remaining before admission and latch a committed crossing until the rear edge clears.
- Existing navigation configuration targets the legacy map.
  Mitigation: regenerate map, initial pose, costmap bounds, and footprint parameters for the final 4.2 m scene.

## Review Questions

- Does camera segmentation reliably reject dark inactive lamp photographs?
- Can each crossing clear within the two-second admission reserve?
