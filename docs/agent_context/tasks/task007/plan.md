---
task_id: task007
type: plan
status: ready_for_architecture_review
from: planner
to: coder
revision: 0
requires_review: true
---

# Task 007 Plan: Live YOLO perception and traffic-light gate

## User scope

Use `/home/sz/下载/best.pt` for all camera recognition needed by the competition task. OCR is explicitly user-owned and excluded from agent work. The agent must localize and annotate license plates and preserve their crops for the user.

## Inputs to freeze

- Checkpoint: `/home/sz/下载/best.pt`, expected SHA-256 `fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad`.
- Camera: `/camera/image_raw`, 640x480 at 15 Hz.
- Expected classes: `resident`, `stranger`, `red_on`, `red_off`, `yellow_on`, `yellow_off`, `green_on`, `green_off`, `license_plate`.
- Initial confidence floor: 0.25, matching the saved-photo audit; freeze it before task008's three-run streak.

## Required implementation

- Replace the HSV-only traffic-light output and COCO placeholder with a live CPU-compatible YOLO stream using the supplied checkpoint. Bound processing to at least 5 frames/s and publish from the source image stamp; discard stale queued frames.
- Publish annotated `/inspection/image`, structured `/inspection/detections` JSON, and the watchdog-facing `/inspection/traffic_light` plus confidence. Every JSON detection must contain source stamp, class, confidence and pixel box; preserve raw and annotated images from the same source frame at each photo point.
- Publish `/inspection/traffic_light` as structured JSON with `state`, `confidence`, and the source camera stamp. The watchdog must parse and validate these fields; legacy plain text, malformed JSON, missing confidence, future/stale stamps, or confidence below 0.50 become UNKNOWN and clear the consecutive-GREEN count.
- Derive GREEN/RED/YELLOW only from a confident, unambiguous active lamp class. Missing, stale, conflicting, low-confidence, or model-error output must never become GREEN. Require three consecutive fresh YOLO GREEN detections, agreement with `/traffic_light/state`, and at least 2.0 s remaining before the watchdog authorizes a controlled crossing. Enforce <=500 ms both from source stamp and callback receipt.
- The YOLO node must record processing rate, frame age and inference-to-publish latency. Acceptance service level: sustained rate >=5 Hz, p95 processing latency <=250 ms, and no consumed frame older than 500 ms. Watchdog perception freshness must be <=500 ms.
- At each saved point, pair raw/annotated/detection evidence by the same camera stamp, select the sharpest eligible frame, and enforce the current per-point acceptance criteria. Require the expected people count, all three signal lamps, or at least one plate box as applicable. All required boxes must stay at least 5% inside the image boundary. A failure aborts the mission; it cannot silently save an incomplete photo as successful.
- Produce terminal detection summaries linked to image stamps and annotated frames. Accumulate the people count over the inspected street without double-counting repeated views; report both per-point counts and the unique total against the authored standee inventory. Do not infer plate characters.
- The photo-route launcher must force `/cmd_vel_watchdog/enforce_traffic=true` and reject a caller override of false. Readiness checks must reject a missing/wrong checkpoint, detector node, stale detections, false white-line/traffic gate, or absent annotation/detection stream before launching the route executor.

## Verification

- Inspect checkpoint metadata and verify exact class order and digest.
- Run the detector on the archived Round19 photos and confirm all classes needed by the route can be recognized; do not use OCR.
- Run the live node against the Gazebo camera and measure 5 Hz, p95 latency, maximum frame age, annotation/detection stamp pairing, and RED/YELLOW/GREEN classification at signal views.
- Verify fail-closed behavior for absent model, stale image, conflicting lamp classes, stale GREEN, simulation disagreement, and insufficient green time. No full route acceptance run is counted in task007.
- Confirm point photo evidence has raw image, same-stamp annotated image, matching detection JSON, and the prescribed object count plus 5% frame margin.

## Out of scope

- OCR or plate-character recognition.
- Changing the 10-point order, adding points, scene assets, map geometry, or navigation speed parameters.
- Starting task008's three-run streak. Any diagnostic route/photo validation before the freeze is labeled non-acceptance and does not count toward 3/3.

## Handoff

After an independent plan approval, the coder implements this task, records the live/offline evidence in `summary.md`, and hands it to an independent reviewer. Only after review approval does task008 freeze the build and begin three consecutive complete runs.
