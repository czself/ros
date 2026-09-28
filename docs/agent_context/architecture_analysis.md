---
type: architecture_analysis
status: reviewed
updated_by: architecture_reviewer
review_required: false
---

# Architecture Analysis

The architecture in this document was approved for task006 planning on 2026-09-27. Implementation and runtime acceptance remain pending; the review does not authorize a route run.

## Current Shape

Gazebo hosts the 4.2 m scene, my_car, two synchronized traffic-light models, person standees, and vehicle-plate standees. The 10-point route executor reads fixed poses from navigation/standee_photo_route.json and returns to HOME. AMCL provides map localization. move_base uses Navfn globally and DWA locally. cmd_vel_watchdog is the only navigation-mode publisher to the Gazebo drive topic.

The global costmap uses /map, which contains the permanent white-line overlay. The live local costmap parameter still subscribes to /map_local_navigation, but /local_map_server was switched via /change_map to publish the identical overlay; a direct check found all 50,176 cells equal to /map. On a fresh launch, the checked-in local_costmap.yaml subscribes directly to /map. The watchdog is now enabled. Before the correction, the local map was raw SLAM and the watchdog was disabled, which allowed a local trajectory to cut through a narrow painted-line gap.

The current visual_inspector uses color thresholds. The custom checkpoint /home/sz/下载/best.pt has seven classes covering residents/strangers, signal-lamp ON/OFF states, and license plates. It has not yet been integrated. No plate OCR stage currently converts a detected plate crop to characters.

## Target Shape

Use one identical white-line occupancy source for global and local costmaps. Keep the full measured footprint and swept-pose guard enabled for every autonomous goal. Keep traffic stop-line permission behind fresh detector output, stable GREEN, signal simulation veto, and sufficient clearance time.

Run a single ordered mission: prescribed diagram route and the ten fixed inspection captures, then HOME. Each capture is reviewed against the saved point criteria. Keep the P5 fixed-direction same-point heading controller but improve route transitions to avoid unnecessary stop-and-turn behavior without adding navigation points.

Use the user-supplied detector checkpoint to publish traffic states, people classes, plate boxes, confidence, and annotated frames. Expose plate boxes/crops for the user's OCR stage; OCR implementation is explicitly out of scope. Console output and annotated images must correspond to the same timestamped detections. The route summary records per-point detections and total person counts.

Maintain two evidence paths: a bag-based geometry/costmap audit for motion legality, and image-based review for recognition/content. Gazebo ground truth is used for evaluation, not as a replacement localization input.

## Key Decisions

- Global and local costmaps use the same verified white-line overlay; the local rolling costmap still includes live laser/depth obstacles.
- The full chassis footprint, not a point robot, is checked against static occupancy and swept commands.
- The watchdog remains the sole command publisher and fails closed if pose, scan, or the line map is unavailable.
- Traffic movement is authorized by perception and timing state, not by simulation light state alone.
- The 10 photo poses remain fixed unless measured visual evidence and a reviewed task explicitly approve a pose adjustment.
- Detections publish observations; the watchdog and route executor consume them through documented ROS topics.
- Preserve the exact launch/stop commands and route evidence per workflow task; never count a MoveBase action success as route or photo acceptance.

## Risks And Tradeoffs

- The current local map overlay may close a narrow passage. Before a full run, validate all eleven route legs and full-footprint sweeps against the same map.
- The 5 cm grid resolution can miss narrow painted gaps; compare map planning with the texture-based watchdog.
- The custom model may detect each inactive lamp as well as the active lamp; signal classification must reject ambiguous frames.
- A license-plate detector does not recognize characters by itself; user handles OCR. Our acceptance covers plate localization, confidence, and annotated crop output only.
- Person counts across multiple viewpoints require deduplication or the task's per-point count standard, without inflating the street-wide total.
- P5/P7 images currently fail their acceptance criteria despite successful pose control.

## Review Questions

- Does the local rolling costmap now load the same white-line overlay as the global map at runtime?
- Are all route segments feasible without any ordinary white-line contact?
- Do stop-line and zebra exceptions remain safe under the supplied task rules?
- Does the custom detector distinguish active lamps, inactive lamps, residents, strangers, and license plates reliably?
- What OCR stage produces readable plate strings with matching annotated output?
