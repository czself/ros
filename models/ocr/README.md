# OCR trial assets

`traffic_light_ros.zip` supplies YOLO detections and a plate-crop topic; it does
not contain an OCR engine or recognition weights. The downstream trial tool
uses Tesseract 4+ with Chinese simplified and English language data, and the
6 px crop margin used by `tl_vision`.

The `.traineddata` files come from the official
[`tesseract-ocr/tessdata_fast`](https://github.com/tesseract-ocr/tessdata_fast)
repository and are licensed under Apache-2.0. SHA-256:

- `chi_sim.traineddata`: `a5fcb6f0db1e1d6d8522f39db4e848f05984669172e584e8d76b6b3141e1f730`
- `eng.traineddata`: `7d4322bd2a7749724879683fc3912cb542f19906c83bcc1a52132556427170b2`

`scripts/plate_ocr_trials.py` runs offline trials over accepted P8–P10 raw
images and detection JSON. It reports multiple crop preprocessing and page
segmentation candidates. Results are for review and do not control navigation,
green-light authorization, or route acceptance.

The container must have the Tesseract CLI installed. To try one closed run:

```bash
python3 scripts/plate_ocr_trials.py \
  --run-dir /home/sz/ros1_ws/photo_stops/standee_route_runs/20260928_135500 \
  --output-dir /home/sz/ros1_ws/photo_stops/plate_ocr_trials_20260928_135500
```
