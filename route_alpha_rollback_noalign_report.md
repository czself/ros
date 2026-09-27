# Round 6: AMCL alpha rollback, no hidden alignment

## Result

Reset to contract HOME and passed readiness. Gazebo was reused. The executor sent only POINT_1 through POINT_10; no ALIGN_FOR or same-position alignment goals were present. AMCL odom_alpha1-4 were 0.01; P4/P5 allowed reverse at min_vel_x=-0.08, P7 was forward-only; white-line gate remained enabled. No manual localization service calls were made.

POINT_1, POINT_2, and POINT_3 were photographed. The direct POINT_4 goal failed twice with ACTION_RESULT; the route stopped at P4. P4 photo, P5-P10, and HOME were not reached.

## Photos and AMCL refresh

| Point | AMCL stamp | JSON age at evidence write | Refresh source | Actual map pose | Position / heading error | Visual check |
|---|---:|---:|---|---|---|---|
| P1 | 12120.645 | 1.519 s | fresh_amcl_pose_after_scan | (1.7054,-0.5822,1.5786) | 1.15 cm / 0.0382 rad | Signal fully visible |
| P2 | 12137.653 | 1.523 s | fresh_amcl_pose_after_scan | (1.6794,1.0763,-3.0154) | 2.93 cm / 0.0390 rad | Three people full-body |
| P3 | 12189.064 | 1.531 s | fresh_amcl_pose_after_scan | (1.0937,1.6487,-1.5271) | 2.82 cm / 0.0376 rad | Three people full-body |

At P2 the first forced scan updated particlecloud without a new pose; the executor requested the next resample cycle, received a fresh AMCL pose, and captured the photo. JSON age is measured after the image burst when evidence is written.

## P4 failure evidence

- Direct POINT_4 started at sim 12190.608; attempt 1 aborted at 12201.394 and the single retry aborted at 12207.281. Both results were ACTION_RESULT / "Failed to find a valid control."
- Navfn kept publishing a global path of 50-51 poses to POINT_4 (0.5463, 0.8263).
- Near attempt 1 abort, MoveBase feedback was map (0.9744, 1.6300, yaw 2.4636), about 0.91 m from the target. The local DWA plan had shrunk to 2 poses with heading about -2.55 rad. The retry failed near the same location.
- Gazebo chassis truth at sim 12201.391 was world (0.94775, 1.66346, yaw 2.54245). Using the HOME-pair map/world calibration and base_footprint-to-chassis offset gives estimated map truth (0.8871, 1.6397, yaw 2.5486), about 0.882 m from P4.
- During the first 6 seconds the chassis yaw turned about -2.23 rad, from -1.51 to -3.74 rad (wrapped to +2.54). Angular command reached about -0.8 rad/s. Commands then fell to zero while the robot remained roughly 0.9 m from the goal.
- DWA repeatedly failed to produce a valid path. Costmaps were cleared; SafeEscape made three reverse arcs of about 0.33 m and one forward-turn arc of about 0.32 m. At the final probes it found no footprint-clean escape arc (v=0,w=0). The sampled global footprint had no lethal cell inside and its nearest global lethal cell was about 4.78 cm away; the second-attempt local footprint had no lethal cell inside and its nearest local lethal cell was about 2.97 cm away.

## Bag

The selective bag started before HOME reset and was stopped with SIGINT. There are no .active files; rosbag info validated motion_0.bag (447 s, 1,478,320 messages, about 394 MB). It includes particlecloud, AMCL parameter updates, scans, TF, odometry, Gazebo truth, DWA plans, costmaps, footprints, recovery status, and ROS logs. Continuous camera streams were excluded; the photos are stored separately.

Diagnostics: /home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha_rollback_noalign_01/
Photos: /home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_alpha_rollback_noalign_01/
