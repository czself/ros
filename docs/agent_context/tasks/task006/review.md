---
task_id: task006
type: review
status: changes_requested
from: reviewer
to: orchestrator
revision: 2
decision: CHANGES_REQUESTED
next_action: revise_same_task
---

# Task 006 Review — white-line hard constraints

## Decision

CHANGES_REQUESTED

This revision fixes the previous gaps in complete texture-footprint rasterization, angular sampling, and preflight stop checks. The static paint overlay still matches the current source map. The audit can still report a false pass if the map YAML decodes black pixels as free, does not check map-origin orientation, or a live costmap has a different footprint/source. The route-only command also reports a small reverse projection as a hard failure after the measured POINT_7 pose update.

## Findings

1. **[P1] Static overlay validation assumes black PGM pixels are lethal without checking map-server semantics or preserving all source obstacles.** `static_map_audit()` compares `overlay == 0` to the texture mask (`audit_white_line_constraints.py:102-135`) but does not decode `negate`, `free_thresh`, and `occupied_thresh`, nor check that every expected paint cell becomes occupancy value 100. With a changed `negate`/threshold configuration, the pixel comparison can pass while the map server publishes white paint as free. It also never rejects a source-occupied cell that the overlay accidentally clears. Decode the overlay with its YAML thresholds in the static check, assert all expected paint cells are lethal, compare all source/overlay occupancy metadata, and require `overlay` to retain every occupied source cell.

2. **[P1] PGM-to-live-grid equality is not an exact pose match yet.** `overlay_matches_live` and `same_map` compare frame, dimensions, resolution, origin x/y, and map cells (`audit_white_line_constraints.py:252-272`), but omit origin z and the full origin quaternion/yaw. `grid_meta()` also records only origin x/y. A grid with a rotated origin could pass these predicates even though its cell-to-world transform differs. Compare and report the complete origin pose, and include its orientation in both source-map and PGM/live-grid equality checks.

3. **[P1] Live costmap geometry and map source are not fully tied to the measured configuration.** The checker always supplies `DEFAULT_FOOTPRINT` rather than reading/comparing the live global and local costmap footprints (`audit_white_line_constraints.py:299-305`). The live audit reads only the local static-layer map topic and compares `/map` with `/map_local_navigation`; it does not verify the global layer's effective source or that every expected paint cell is lethal in each costmap. A route that avoids a missing obstacle region could still pass the sampled-path checks. Read back both live footprints and effective map sources, verify full map metadata, and compare the static paint mask against the relevant global cells and every in-window local route sample.

4. **[P1] Pending/canceling goals are not all treated as active.** The preflight now checks before `make_plan` and verifies again afterward, as required, but filters status values only to `{PENDING, ACTIVE}` (`audit_white_line_constraints.py:282-284, 450-452`). `PREEMPTING` and `RECALLING` are still nonterminal states. Include them in the refusal/final-status checks so an audit cannot run during goal cancellation.

5. **[P2] The exception counter is not a count of painted conditional contacts by type.** `texture_masks()` marks the full start/stop rectangles and zebra bounding boxes as `conditional`, including nonwhite pixels, and `texture_footprint_contacts()` reports one aggregate `CONDITIONAL_MARKING_CONTACT` (`audit_white_line_constraints.py:66-77, 357-365`). This mislabels the always-allowed start-line departure as traffic-gated and can count empty pixels as contacts; stop-line and zebra contacts are not separated. Build masks from actual white pixels, report start-line contact separately, and separately count actual stop-line and zebra paint contacts. Keep these contacts nonfatal to the ordinary-paint comparison, but do not treat `enforce_traffic=true` alone as proof that a crossing had stable GREEN authorization.

6. **[P2] Offline-only mode can still emit the generic `overall_pass` and exit 0 without live checks.** When `--live` is omitted, `result.get('live', {}).get('live_ready_for_full_task', True)` defaults to true (`audit_white_line_constraints.py:500-511`). A caller can therefore see `overall_pass: true` without any live map, gate, costmap, goal-state, or velocity evidence. Report this as `static_only_pass`/`live_not_checked`, or reserve `overall_pass` for a run that includes the required live audit.

7. **[P2] The static point-order test now rejects the measured POINT_7 adjustment for a 4.3 cm reverse projection.** The latest offline output has POINT_6 progress `6.5290 m` and POINT_7 `6.4864 m`, so strict monotonic projection makes `overall_static_pass=false`. The live per-path checker already tolerates up to 5 cm of backtrack (`audit_white_line_constraints.py:409-411`). Apply one documented tolerance consistently to endpoint and path checks, or explain why this measured photo-pose adjustment violates the contract; otherwise the static checker reports a failure even though it may be only a small camera-position adjustment along the local corridor.

## Resolved from revision 1

- The texture check now rasterizes the complete rotated chassis polygon and dilates by one pixel, matching the watchdog's pixel coverage method (`audit_white_line_constraints.py:336-365`).
- Path interpolation uses at most 2 cm translation steps and at most 2° yaw steps, including the initial start pose orientation (`audit_white_line_constraints.py:385-397`).
- Live preflight now checks for pending/active goals and zero drive command before the first `make_plan`; it checks again after planning. The cancellation-state gap above remains.
- `expected_occupancy_grid()` decodes the overlay using the PGM/YAML rules, and live mode requires exact cell-array equality plus frame/resolution/origin x/y checks; a failed `live_ready_for_full_task` makes `overall_pass` false in live mode.

## Reviewer Verification

- Command: `python3 scripts/audit_white_line_constraints.py`
  Result: exit 1. Texture mask remains 1,711 cells; 1,310 cells were newly added, 401 expected paint cells were already occupied, and missing/extra added cells are 0. Route projection is no longer monotonic after the saved POINT_7 coordinate adjustment: P6 `6.5290 m`, P7 `6.4864 m`; other poses remain within the reported 0.18 m point tolerance.
- No `--live` mode was invoked. No ROS service, goal, route, or robot command was used in this review.

## Required Fixes

- Validate PGM decoding semantics and that the overlay never clears source obstacles.
- Compare full map origin pose and verify the global/local live costmap sources and footprints against configuration.
- Treat PREEMPTING and RECALLING as active states.
- Split start-line versus actual white stop-line/zebra contacts and avoid counting nonwhite exception pixels.
- Do not report a full `overall_pass` from static-only mode.
- Resolve POINT_7's small monotonicity failure with an explicit, consistent corridor-progress tolerance or a reviewed route-contract decision.

## Next Action

Return task006 to implementation for these remaining checks. No route run, ROS call, or navigation parameter change was authorized by this review.
