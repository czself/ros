---
task_id: task006
type: architecture_review
reviewer: independent_architecture_reviewer
decision: CHANGES_REQUESTED
latest_decision: APPROVED
revision: 1
next_action: implementation
---

# Task 006 Architecture Review

## Decision

**CHANGES_REQUESTED.** The shared static-map direction and full-footprint watchdog are sound, but the plan does not yet define the special painted-line exceptions precisely enough or verify the runtime configuration and route constraint strongly enough to support its acceptance claim.

## Scope and evidence considered

Reviewed the task plan, project context, architecture analysis, the user requirements file as a product specification, and the relevant costmap, watchdog, readiness, and launcher sources. No ROS commands, route goals, model runs, or code changes were made. The supplied runtime evidence says `/map` and `/map_local_navigation` currently match cell-for-cell, the watchdog parameter is enabled, and all 11 point-to-point Navfn plans passed a full-footprint audit. That is useful snapshot evidence, but it does not resolve the items below.

## Required revisions

1. **Define the painted-line exception policy and the test/acceptance mode.** The ordinary white pixels are blocked by the watchdog, but the source has bounded exceptions: the start line is always passable; stop-line pixels are passable after GREEN authorization; zebra pixels are passable on stable GREEN. However, `start_navigation.sh` defaults `ENFORCE_TRAFFIC` to `false`, and `cmd_vel_watchdog.py` then treats stop-line and zebra pixels as permitted without a GREEN authorization. The photo-route launcher does not override that default. Thus `enforce_white_lines=true` alone does not mean every traffic-controlled marking is gated. Update the plan with an explicit matrix: ordinary lane boundaries are always forbidden; the start line is only the measured departure exception; stop lines and zebra crossings require the defined traffic authorization in any traffic-compliance run. State that a run launched with traffic enforcement disabled is diagnostic-only and cannot count as task acceptance. Keep detector integration out of task006 if desired, but preserve this boundary for task007 and final acceptance.

2. **Verify the effective local static-layer source after a fresh launch.** The current runtime local static layer still names `/map_local_navigation`; the checked-in fresh-launch config names `/map`. Comparing the two source OccupancyGrid messages at one instant proves their current contents match, not that a restarted local costmap loaded the intended source. Add an explicit runtime readback of the local `static_layer/map_topic` parameter after launch, and verify that the local costmap has incorporated that map with the expected frame, resolution, dimensions, and white-line cells. Retain the source-topic equality check as an additional provenance check.

3. **Audit registration and disagreement between the two safety representations.** Navfn/DWA use the occupancy overlay, while the watchdog samples the ground-texture image using its own world-to-pixel transform and bounded exception masks. The current verification describes auditing plans against costmaps, but does not compare the watchdog's texture-derived forbidden area and exceptions to the occupancy overlay. Add a reproducible comparison of their coordinate transforms and boundary cells, then check the full swept footprint along audited paths against both representations. Report disagreements explicitly; the watchdog may safely stop at them, but they can also explain planner progress failures and must not be mistaken for a valid drivable gap.

4. **Make the audited path correspond to the prescribed route.** Auditing the 11 shortest Navfn paths between HOME and the photo poses proves those particular paths are clear; it does not prove they follow the competition diagram's route. Include the route-contract corridor/ordered segments in the read-only audit, or state precisely how the photo-route plan is constrained to that corridor. Check the complete footprint against ordinary lane markings along the resulting path, not just at its endpoints. This audit can remain read-only and must not dispatch goals.

## Review notes

- The measured polygon, `robot_radius: 0`, unknown-space rejection in Navfn, local static layer, live obstacle layer, and fail-closed watchdog are appropriate ingredients for constraining motion.
- Global and local costmaps need the same white-line source; they should not be expected to have identical total costs because the local map also contains rolling-window sensor obstacles and its own inflation layer.
- A 2 cm path interval is useful for this 5 cm occupancy grid, but the report should state how every cell overlapped by the rotated footprint is tested and how grid-edge contact is treated. Sampling only center points or corners would not be sufficient evidence of a full-footprint check.
- The no-goal requirement is appropriate for this task. Passing task006's static audit will not itself prove end-to-end motion enforcement or traffic-light compliance; those remain separate runtime acceptance evidence.

## Re-review condition

Revise the task006 plan to include the four checks above, especially the default traffic-bypass caveat and exact start/stop/zebra semantics. Then implement and record the no-goal verification. No route run is authorized by this review.

## Revision 1 Re-review

**Decision: APPROVED (plan only).** Revision 1 addresses all four requested architecture changes:

- It separates ordinary lane boundaries from the bounded start-line exception and GREEN-gated stop-line/zebra traversal, and marks `ENFORCE_TRAFFIC=false` runs as diagnostic-only.
- It requires a fresh-launch readback of each live static-layer source and records the expected map metadata and costmap cells.
- It adds a reproducible comparison between the occupancy overlay and the watchdog's texture-derived mask, including transforms, exceptions, boundary discrepancies, and swept-footprint checks.
- It ties the fixed-point Navfn paths to the ordered `inner_route.yaml` corridor and requires monotonic route projection and no shortcut.

The plan also asks for full-footprint checks at intervals no greater than 2 cm and explicit treatment of exact-cell boundary contact. No unresolved architecture issue blocks implementation of the stated no-goal audit. This approves the revised plan only: implementation evidence is still pending, task006 must keep the robot stopped, and this decision does not authorize a route run or establish traffic-light/task acceptance.
