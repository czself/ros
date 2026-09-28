---
task_id: task007
type: review
status: completed
from: perception_model_reviewer
to: orchestrator
revision: 0
decision: CHANGES_REQUESTED
next_action: add_plate_ocr_before_accepting_full_perception_pipeline
---

# Independent `best.pt` perception audit — 2026-09-27

## Decision

**CHANGES_REQUESTED for the complete perception requirement; the checkpoint is suitable for still-image object/state boxes on this Round19 sample.** It detects the visible traffic-light states, people and plate regions. It does not recognize plate characters, so OCR remains a separate missing component. The runtime integration and traffic-gate safety behavior were not tested here.

This is an isolated checkpoint review against the active project acceptance criteria. No task007 plan/implementation summary or code diff was present to review. No ROS node, robot, or navigation goal was run, and no project files other than this audit report were changed.

## Artifact and inference setup

- Source: `/home/sz/下载/best.pt`, SHA-256 `fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad`.
- The container did not mount the source pathname, so the exact-hash file was copied read-only for inference to `/tmp/weights_audit_20260927_best.pt` inside `ros1_modeling`.
- Ultralytics `8.2.103`; PyTorch `2.2.2+cpu`; task `detect`; `torch.cuda.is_available() == false`. GPU timings are unavailable in this container.
- `MPLBACKEND=Agg` was set because this headless container lacks `tkinter`; no package was installed.
- Images: saved Round19 POINT_1–POINT_10 RGB photos, each 640×480. Inference used `conf=0.25`, `imgsz=640`, `device=cpu`, `save=False`; one warm-up per image, then three timed passes. Wall time below is the median of those three passes; Ultralytics' per-image inference speed is also shown.

## Actual checkpoint class map

`model.names` returned:

```text
0 resident
1 stranger
2 red_on
3 red_off
4 yellow_on
5 yellow_off
6 green_on
7 green_off
8 license_plate
```

## Per-image detections and CPU timing

Confidence values are per detected box. “Boxes” is the total number of boxes at the stated `.25` threshold.

| Image | Boxes | Detected classes and confidence | CPU wall median | Ultralytics preprocess / inference / postprocess |
|---|---:|---|---:|---:|
| P1 | 3 | `red_on` .905; `yellow_off` .944; `green_off` .872 | 89.89 ms | .70 / 86.37 / .39 ms |
| P2 | 6 | `stranger` .854; `resident` .828, .730; `green_on` .838; `red_off` .730; `yellow_off` .645 | 90.61 ms | .85 / 86.03 / .35 ms |
| P3 | 3 | `resident` .747, .728, .693 | 88.77 ms | .84 / 93.03 / .49 ms |
| P4 | 3 | `resident` .895, .888, .880 | 88.62 ms | .66 / 83.26 / .33 ms |
| P5 | 6 | `resident` .902, .850; `stranger` .851; `red_off` .931; `yellow_off` .844; `green_on` .819 | 91.82 ms | .67 / 86.07 / .37 ms |
| P6 | 5 | `resident` .888, .871, .855, .844, .730 | 94.89 ms | .70 / 87.03 / .69 ms |
| P7 | 3 | `red_off` .934; `yellow_off` .902; `green_on` .907 | 88.73 ms | .77 / 81.83 / .37 ms |
| P8 | 1 | `license_plate` .848 | 87.49 ms | .64 / 83.74 / .34 ms |
| P9 | 1 | `license_plate` .898 | 88.95 ms | .84 / 84.65 / .34 ms |
| P10 | 1 | `license_plate` .897 | 87.16 ms | .76 / 81.21 / .37 ms |

Overall median CPU wall time was about **88.86 ms/image** (roughly 11 images/s for this batch inference path). No GPU time can be reported because the installed PyTorch build is CPU-only.

## Capability assessment

- **Traffic lights:** All three lamp heads were boxed in both P1 and P7, with the active lamp classified `red_on` in P1 and `green_on` in P7; the other visible lamps were classified as off. This supports single-frame lamp/state detection on these samples. It does not establish temporal stability or safe signal-gate authorization.
- **People:** The model returned the expected counts for P2–P4 (three boxes each) and P6 (five boxes). P5 returned three person boxes—two residents and one stranger—which matches the three people visible in that saved frame, not the four-person photo criterion. A detector cannot recover a person absent from the image. The checkpoint also does not provide identity tracking/deduplication across views.
- **License plates:** P8–P10 each produced one high-confidence `license_plate` box. These are plate-region detections only.
- **OCR:** **Still missing.** The model has no character/text class or text-recognition output; it returns a plate bounding box, not a string. A separate OCR stage is required to satisfy readable-plate recognition.
- **Operational scope:** At about 90 ms CPU wall time per still, this is suitable for per-photo inference in the inspected batch. GPU inference was not testable. Continuous-stream latency, ROS message wiring, annotated-output correlation, confidence calibration, multi-frame traffic-state stability and gate fail-safe behavior remain unverified.

## Reviewer verification

- Command: copy the source checkpoint into the container and load it with `YOLO('/tmp/weights_audit_20260927_best.pt')`; run `predict(..., conf=0.25, imgsz=640, device='cpu', save=False)` on each saved Round19 image.
- Result: source/container SHA-256 matched; `model.names` and `model.task` were read from the loaded model; all ten images returned the counts/confidences above.
- GPU check: `torch.cuda.is_available()` returned `false` (`torch 2.2.2+cpu`).

## Scope update

On 2026-09-27, the user explicitly assigned all OCR work to themselves and asked that subsequent agent tasks skip OCR. The OCR gap above remains a capability fact, but it is not an agent acceptance blocker. Agent scope is detector boxes, confidence, annotated output, and plate crops only.
