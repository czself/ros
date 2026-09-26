# Navigation photo route progress — 2026-09-26

The route still uses the ten recorded photo poses in order and returns to the
original HOME pose. No photo coordinates were changed and no hidden alignment
or transition goals are inserted.

## Verified improvements

- Removed the executor's hidden in-place alignment goals before POINT_5 and
  POINT_7. Each navigation action now targets the recorded point directly.
- Added a positive DWA `twirling_scale` for photo navigation. In P3 testing,
  near-pure-rotation commands fell from about 62% to about 35% of samples.
- Reworked SafeEscape to subscribe to the active Navfn plan, score collision-
  checked arcs by path progress and cross-track error, and prefer endpoints
  outside the inscribed obstacle halo.
- P3 was reached and photographed after setting only P3's XY navigation and
  capture tolerances to 0.05 m. The resulting photo shows all three people
  fully in frame; its map-TF error was about 3.3 cm and heading error about
  2 degrees.
- Disabling continuous DWA trajectory-cloud publication restored Gazebo
  performance; the P1 navigation time returned to roughly 9 seconds after a
  diagnostic run with trajectory-cloud publishing took about 31 seconds.

## Round 15 result

The full route reached POINT_1 through POINT_10 in order, returned to HOME, and
stopped with zero commanded velocity. The run summary is `COMPLETE_PARKED`.
P5 and P8 each timed out once at 75 seconds and succeeded on the executor's
single retry. The other goals succeeded on their first attempt.

Visual review of the ten captures:

- P1–P4, P6, and P8–P10 passed the recorded framing criteria.
- P5 failed: only three of the required four people are visible; the rightmost
  person is cut off.
- P7 failed: the top of the traffic-light fixture is clipped.

AMCL reported HOME within 2.6 cm and 0.036 rad of the saved pose. A later
Gazebo-truth sample was within 2.7 cm and 0.10 rad, but it was taken 47 seconds
after completion; the same-stamp HOME truth comparison is still pending.

The full rosbag (533 MB, 571 seconds, 1,896,936 messages) and captures are in:

- `/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha20_min05_pathaware_01/`
- `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_alpha20_min05_pathaware_01/`

## Cause of the repeated turning

The recovery plugin was choosing sharp arcs with angular speed up to 1.10
rad/s, then asking MoveBase to replan the same goal. Some accepted arcs moved
only 7 cm. At P8, it accepted a 7 cm arc with negative path-progress score
(-0.067) while cross-track error grew from 0.005 m to 0.054 m. The old rejection
threshold allowed that arc. P5 is especially sensitive because P4→P5 is only
about 0.17 m and the saved heading reverses by almost 180 degrees. There was no
DWA “failed to produce path” log at P5; its recovery sequence followed lack of
useful progress rather than a confirmed absence of local trajectories.

The first recovery revision limited candidate arcs to ±0.38 rad/s. After the
P5 failure in Round 16, the current candidate restores higher-curvature arcs
for tight spaces, samples only durations of 1.2–2.0 seconds, requires at least
0.12 m travel, penalizes angular speed, and rejects path score below 0.03 or
more than 0.05 m extra cross-track error. No route coordinates changed. The
package compiles inside `ros1_modeling`; route validation is pending.

Photo acceptance is still open until P5/P7 pass and HOME truth is confirmed.

## Round 16: recovery constraint check

The first low-curvature-only revision reached P1–P4 faster (about 11, 14, 22,
and 22 seconds) and did not execute the old repeated tight loops. It then failed
at P5: attempt 1 ended `NO_PROGRESS`, attempt 2 aborted, and no P5 photo was
taken. Both recovery probes found no collision-free long low-curvature arc
(endpoint cost 254). P6–P10 were not attempted. The bag is preserved at
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha20_min12_pathaware_01/`.

The next candidate keeps the 0.12 m minimum travel and the path-score/cross-
track gates, starts duration sampling at 1.2 seconds, and restores higher
curvature candidates only for cases where the local costmap rejects gentler
arcs. It applies an angular-rate penalty so a useful tight arc can pass while
7 cm micro-arcs cannot. This revision compiles, but has not yet been loaded or
tested in a route run.

## Round 16 P5 diagnosis and HOME return

The Round 16 bag confirms this is a short-goal control oscillation, not a DWA
planner with no path. At P5 the Navfn path stayed valid and DWA published local
plans, usually only two poses long. During both attempts, forward and reverse
commands each appeared in roughly 40–50% of samples, while pure rotation was
only 6–8%; net forward progress was near zero. SafeEscape's low-curvature-only
revision found no collision-free long arc in the live costmap, so the route
stopped at P5. The current local map had lethal wall cells and scan returns
0.18–0.22 m away around the short P4→P5 turn, despite a roughly 0.7 m visual
corridor. See
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha20_min12_pathaware_01/p5_no_progress_diagnosis.md`.

Round 17 tested POINT_5 with reverse disabled. It did not reach the point and
made DWA rotate in place more often: 84% of attempt 1 and 93% of attempt 2
commands had near-zero translation, with 32 and 27 angular-velocity reversals.
That setting has been reverted; POINT_5 again permits the configured reverse
samples. POINT_7 retains its existing forward-only behavior.

After the failed Round 16 route, navigation was restarted from the measured
current Gazebo pose (no teleport) and the rebuilt higher-curvature plugin was
loaded. One HOME action succeeded under AMCL's 0.05 m / 0.10 rad tolerances,
but truth at arrival was about 2.5 cm and 6.1 degrees from HOME. Tightening
the final tolerances made MoveBase report success while truth still differed
by about 4 cm and 5 degrees. With zero commanded velocity, the Gazebo chassis
continued to yaw slowly (about 0.00042 rad/s). A later same-HOME correction
ended in a 2.3 rad heading error and MoveBase oscillation abort. The vehicle
was stationary afterward; the 10-point route and exact HOME acceptance remain
open. The full return-attempt bag is
`/home/sz/ros1_ws/navigation_diagnostics/20260926_round16_final_home_relocalize_return_04/home_return_0.bag`.

The original round-8 bag was overwritten by a recorder started with a reused
directory. Its photos, P3 attempt samples, and report remain, but that bag is
unavailable. All later runs use unique directories.

## Round 17 result

P1, P2, and P4 passed the photo criteria. P3's three people were visible, but
their feet were cut off. P5 aborted twice and produced no photo; P6–P10 and the
route's own HOME return were not attempted. The executor did not add any goal
after the P5 failure. The bag is
`/home/sz/ros1_ws/navigation_diagnostics/20260926_round17_alpha02_escapev2_01/motion_0.bag`
(303 seconds, 1,003,501 messages, 265.5 MB).

P5 remained a valid Navfn goal, while local plans had zero or near-zero length.
The first SafeEscape arc (`v=-0.12, w=1.10`, 0.194 m) improved path progress in
AMCL's estimate but moved Gazebo truth farther from P5: true target distance
increased from about 10.2 cm to 11.7 cm. The map estimate was 6–8 cm from
truth. Two later SafeEscape probes found no collision-free arc (`endpoint_cost
254`). This explains why the visual width of the road does not guarantee a
safe turn: the local footprint map includes a nearby wall and the pose error
is comparable to the remaining goal distance. Full detail is in
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha02_escapev2_01/p5_no_progress_diagnosis.md`.

After Round 17, a separate HOME-only action returned `SUCCEEDED` with `cmd_vel`
zero. Its calibrated Gazebo truth was about 5.6 cm and 0.088 rad from the
contract HOME, so exact HOME acceptance remains open. Report and bag:
`/home/sz/ros1_ws/navigation_diagnostics/20260926_round17_home_only_return_01/`.

## Round 18 result and P5 turn plan

Round 18 used the measured 0.13572 m effective wheel-separation setting and
restored reverse sampling at P5. The encoder-vs-Gazebo error at P4 fell from
21.5 cm / 10.6 degrees to 5.2 cm / 4.0 degrees, but the route still aborted
at P5 twice. P1, P2, and P4 passed; P3 still cut the feet. No P6–P10 goals
were sent. A one-shot HOME-only goal succeeded afterward, with Gazebo truth
within 4.54 cm and 0.077 rad of contract HOME. See
`/home/sz/ros1_ws/navigation_diagnostics/20260926_round18_alpha02_wheel13572_reversep5_01/result_report.md`
and `/home/sz/ros1_ws/navigation_diagnostics/20260926_round18_home_only_return_01/home_only_return_report.md`.

The P4→P5 segment itself allows a checked in-place turn toward the segment
bearing: recorded costmaps show at least 5.9 cm global and 14.2 cm local
footprint clearance there. At the P5 failure pose, turning all the way to the
camera yaw leaves only 1.4 cm true lethal clearance. The current implementation
uses one MoveBase goal at the recorded P5 XY with yaw temporarily unconstrained,
then runs a low-speed, fixed-direction yaw servo at the same physical pose.
Before each servo it sweeps the full footprint against both current costmaps.
The P4 path-bearing turn is also checked. The route keeps the same ten
coordinates and sends no extra MoveBase pose goal; this is a candidate for the
next full-route validation and is not yet accepted.

## Wheel odometry calibration candidate

Round 17 joint-angle regression estimates an effective wheel separation near
0.140 m; the current plugin setting is 0.133 m. Replaying the same wheel
increments offline with 0.13572 m reduced the P4 truth-versus-odometry error
from about 21.5 cm / 10.6 degrees to 16.4 cm / 5.1 degrees, and reduced the
first P5 escape error from about 24.3 cm / 19.6 degrees to 17.8 cm / 11.7
degrees. Residual drift remains, so this is a conservative calibration test,
not a complete fix.

The test world and model now set the diff-drive effective separation to
0.13572 m while leaving every route pose unchanged. The model source value is
0.32314286 m before its 0.42 world scale; the adjusted Gazebo world uses the
scaled 0.13572 m directly. Round 18 verified a large reduction in wheel-odom
drift, but still aborted at P5; the correction is useful and insufficient by
itself.

## P5 same-point heading controller candidate

Round 18's captured costmaps show a clear full-footprint sweep at P4 while
turning toward the P4→P5 path bearing (about 5.9 cm global and 14.2 cm local
clearance). Turning all the way to the photo yaw at P5 is much tighter; the
recorded failures had only about 1.4 cm of lethal clearance. The executor now
has a candidate P5 sequence: a low-speed, collision-checked heading turn at
the current P4 position, one MoveBase goal to the recorded P5 XY with heading
unconstrained during translation, then a fixed-direction same-position turn
to P5's recorded camera yaw, also footprint-checked. It does not add another
pose or change the ten recorded coordinates. The source passes Python syntax
compilation. Round 19 has now validated this control sequence; the full-route
result and remaining photo misses are recorded below.

## Round 19 full-route result

Round 19 ran the original POINT_1 through POINT_10 sequence, then returned to
HOME. No photo coordinates were changed and no extra route point was added.
Every goal completed; POINT_4's first MoveBase attempt aborted and the
executor's same-goal retry succeeded. POINT_5 used one MoveBase XY goal and
then the two footprint-checked same-position heading turns. The approach turn
was 107.5 degrees and the camera-heading turn was 74.3 degrees, both in the
same direction. Each ended with zero measured base drift and under 0.02 rad
yaw error. This removed the earlier forward/reverse oscillation, but it still
looks like a large deliberate turn because POINT_4 and POINT_5 are only about
0.163 m apart and their recorded camera headings differ by almost 180 degrees.
This is a pose-geometry requirement, not evidence that the road width itself
prevents turning.

Visual review against the saved acceptance criteria:

- POINT_1: traffic light complete.
- POINT_2–POINT_4: three people, full bodies.
- POINT_5: only three of the required four people visible; fail.
- POINT_6: five people, full bodies.
- POINT_7: traffic-light top clipped by the image edge; fail.
- POINT_8–POINT_10: license plates complete and legible.

The route ended parked. The immediate Gazebo truth snapshot was 0.0352 m and
0.0255 rad from contract HOME; fresh map TF was 0.0264 m and 0.0374 rad away.
The base twist was zero and there were no active goals. The AMCL sample in
that snapshot was stale, so it is not used as the final pose measurement.
The complete bag passed `rosbag info` validation (430 s, 1,422,673 messages,
390.7 MB) at
`/home/sz/ros1_ws/navigation_diagnostics/20260926_round19_alpha02_p5_position_first_01/motion_0.bag`.
The route log, photos, and final truth snapshot are in the same run directory.

This validates stable completion of all ten navigation goals and HOME return,
but does not pass photo acceptance at POINT_5 or POINT_7. The P5/P7 image
failures need pose-at-capture truth alignment before deciding whether to
correct localization or adjust a recorded camera pose.

The capture-time pose comparison narrows the photo misses: at P5, map TF was
2.34 cm / 1.06 degrees from the saved pose, but calibrated Gazebo truth was
7.70 cm / 1.74 degrees from TF and 6.38 cm from the saved XY. At P7, map TF
was 1.78 cm / 1.98 degrees from the saved pose, while Gazebo truth differed
from TF by 8.42 cm and was 9.21 cm from saved XY. P7 AMCL was fresh (about
0.37 s old) and within roughly 0.5 cm of TF, so stale AMCL alone does not
explain the error. No AMCL pose was recorded within two seconds of P5 capture.
The wheel-separation correction improved encoder drift in Round 18, but
Round 19 still has these local TF-to-truth offsets; the current data does not
separate map registration error from scan-matching or odometry error. The
next diagnosis should compare the capture-time laser scan against the
occupancy map before touching the saved photo coordinates.
