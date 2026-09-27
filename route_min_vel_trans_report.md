# Round 8: min_vel_trans=0.025 validation

## Result

Gazebo was reused. The base and AMCL were reset to contract HOME and readiness passed. AMCL odom_alpha1-4 were 0.01; the white-line watchdog was disabled for the photo route. The container received the latest route executor with no ALIGN_FOR or short-leg goals.

POINT_1, POINT_2, and POINT_3 navigation started in order. P1 and P2 were photographed. P3 timed out twice, so the executor stopped before P3 capture. P4-P10 and HOME were not attempted.

## AMCL refresh and dynamic parameters

- P1 and P2 JSON source was fresh_amcl_pose_after_scan; stamps were 12120.645 and 12137.653, evidence-write ages 1.519s and 1.523s.
- At P1 the first forced scan updated particlecloud without a new pose; the executor requested the next resample cycle and then captured.
- Bag parameter updates show min_vel_trans=0.025, min_vel_x=-0.08 and xy/yaw tolerances 0.03/0.04 at P1 (sim 14876.182), P2 (14883.849) and P3 (14897.763).
- P3 attempt 1 timed out at sim 14972.935; attempt 2 timed out at 15048.436. Run summary records 75.0s and 75.1s durations.

## P3 motion evidence

- At P3 goal start, Gazebo chassis world pose was (1.6415, 1.0648, yaw -3.0713). At attempt 1 timeout it was (1.5827, 0.5915, yaw 1.6024): the chassis moved south, away from P3; mapped base-footprint distance from P3 grew from about 0.773m to 1.129m.
- Attempt 2 started near (1.5827, 0.5916, yaw 1.6028) and ended at (1.5475, 0.9653, yaw 1.4268), still about 0.789m from P3.
- Attempt 1 had 2,296 cmd_vel samples: 674 reverse, 1,040 near-zero and 582 forward; angular command ranged from -0.8 to +0.673rad/s. Attempt 2 had 2,245 samples: 819 reverse, 446 near-zero and 980 forward; angular command ranged from -0.664 to +0.617rad/s.
- Recovery events included costmap clears and reverse SafeEscape arcs. After both timeouts, a 20-second observation window showed less than 0.01mm net Gazebo XY motion and cmd_vel stayed zero.

## Photos

P1 signal and P2/P3 people photos are in /home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_min_vel_trans_025_01/. P1 and P2 pass visual review; P3 has no photo because navigation never reached capture.

## Bag

The selected-topic bag ran from sim 14796.48 to 15198.27 (401s), with 1,324,486 messages, about 345MB. motion_0.bag passed rosbag info validation; no .active file remains. Topics include particlecloud, AMCL parameter updates, scan, TF, both maps, odometry, Gazebo truth, DWA plans, costmaps, footprints, recovery status, and ROS logs. Continuous camera streams were excluded.

Diagnostics: /home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_min_vel_trans_025_01/
