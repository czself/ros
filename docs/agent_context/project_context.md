---
type: project_context
status: active
updated_by: planner
---

# Project Context

## Purpose

ROS Noetic and Gazebo Classic simulation for a 4.2 m x 4.2 m smart-community route task. The robot must autonomously traverse the prescribed route and obey traffic lights reproducibly.

## Tech Stack

- Runtime: ROS Noetic, Python 3, Gazebo Classic, Docker container `ros1_modeling`
- Navigation: Gmapping, AMCL, `move_base`
- Perception: traffic-light camera detector only
- Validation: ROS topics, Gazebo model states, XML validation

## Repository Map

- `worlds/competition_classic.world`: current competition scene
- `navigation/`: mapping, localization, and planner launch files
- `scripts/`: robot control, navigation, inspection, and validation nodes
- `maps/`: generated and reference navigation maps
- `insert/`: reusable person and car standee assets

## Current Architecture Facts

- 2026-09-22 correction: task005 implements the user's final annotated inner
  route, with identical birth/finish, seven ordered segments. Authoritative
  route is `navigation/inner_route.yaml`; old guessed contracts/direct driver
  results are invalid and must not be used as acceptance evidence.
- Current requested mode ignores traffic signals but still forbids all
  ordinary white paint. Calibrated texture supplies painted junction topology;
  range sensors alone cannot recognize flat paint. Live laser/depth veto
  physical obstacles; measured chassis feedback supplies simulation localization.
- Runtime acceptance needs continuous full-footprint and ordered-corridor
  audit, full loop, home error <0.08 m and stationary end. Point goal counts
  alone are insufficient. task005 corrects the previously bypassable guard.

- The world is 4.2 m x 4.2 m and currently contains 10 placed person standees and 3 vehicle standees.
- The car and person standees remain as static scene objects; no person or plate recognition is in scope.
- The world and runtime contain exactly two synchronized traffic-light models. `scripts/traffic_light_controller.py` keeps both frames in place and swaps only their internal ON/OFF photo panels on a 10/15/3 second RED/GREEN/YELLOW cycle; no complete signal models are hidden underground.
- `scripts/cmd_vel_watchdog.py` is the sole navigation-mode publisher to the Gazebo drive topic and is therefore the final traffic-rule safety boundary.
- Existing navigation scripts still use legacy birth coordinates near `(4.0833, -3.9583)` and random patrol goals. They are unsuitable for the 4.2 m competition layout.

## Constraints

- Follow the supplied diagram, including material orientation, designated route, lane boundaries, and stop lines.
- The complete vehicle body must remain inside the white lane boundaries; touching or crossing a white line is a route violation.
- Stop-line contact/crossing is permitted only during GREEN. RED and YELLOW require the complete chassis to remain before the line.
- Traffic decisions must require a stable camera detection. Simulation state may veto an unsafe false positive but must never independently authorize GO.
- RED and YELLOW require stopping before the line. A vehicle already admitted on a safe GREEN must clear the line before RED rather than stop on it.
- No keyboard control is permitted during the demonstration run.

## Known Risks

- Existing maps, launch initial poses, and route scripts still use legacy coordinates near +/-4 m and cannot be used as evidence for the 4.2 m scene.
- Camera colour segmentation must distinguish the bright lamp from the two colour-bearing dark lamp photographs.
- Navigation parameters, saved maps, and the robot birth pose must be regenerated for the 4.2 m scene.
