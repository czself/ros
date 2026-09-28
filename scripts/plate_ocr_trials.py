#!/usr/bin/env python3
"""Offline OCR trials over accepted P8-P10 photos.

This is a downstream consumer for the `license_plate` detections produced by
the route's YOLO node.  The supplied traffic_light_ros archive exposes the
same useful boundary (a plate box/crop) but contains no OCR engine.  This tool
uses Tesseract on saved raw frames and never feeds recognized text back into
navigation or the traffic-light gate.
"""

import argparse
import csv
import json
import subprocess
import sys
from io import StringIO
from pathlib import Path

import cv2
import numpy as np


PLATE_POINTS = ('POINT_8', 'POINT_9', 'POINT_10')


def _crop_with_margin(image, box, margin):
    height, width = image.shape[:2]
    x1, y1, x2, y2 = (int(round(float(v))) for v in box)
    left, top = max(0, x1 - margin), max(0, y1 - margin)
    right, bottom = min(width, x2 + margin), min(height, y2 + margin)
    if right <= left or bottom <= top:
        return np.empty((0, 0, 3), dtype=np.uint8)
    return image[top:bottom, left:right].copy()


def _variants(crop, scale):
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    enlarged = cv2.resize(gray, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_CUBIC)
    return {
        'gray': enlarged,
        'otsu': cv2.threshold(enlarged, 0, 255,
                              cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1],
    }


def _run_tesseract(image_path, tessdata_dir, languages, psm):
    common = [
        'tesseract', str(image_path), 'stdout',
        '--tessdata-dir', str(tessdata_dir),
        '-l', languages, '--oem', '1', '--psm', str(psm),
        '-c', 'user_defined_dpi=300',
    ]
    # Ask Tesseract to emit TSV through its config variable. The `tsv` config
    # file is not shipped in our isolated tessdata directory, so passing the
    # config filename would silently lose the word-confidence rows.
    tsv_run = subprocess.run(common + ['-c', 'tessedit_create_tsv=1'],
                             capture_output=True, text=True,
                             timeout=30, check=False)
    if tsv_run.returncode != 0:
        detail = tsv_run.stderr.strip() or 'Tesseract TSV returned %d' % tsv_run.returncode
        raise RuntimeError(detail)
    confidences = []
    words = []
    for row in csv.DictReader(StringIO(tsv_run.stdout), delimiter='\t'):
        try:
            if row.get('level') == '5' and row.get('text', '').strip():
                words.append(row['text'].strip())
                confidence = float(row.get('conf', '-1'))
                if confidence >= 0.0:
                    confidences.append(confidence)
        except (TypeError, ValueError):
            continue
    raw = ' '.join(words)
    compact = ''.join(char for char in raw if char.isalnum())
    return {
        'text_raw': raw,
        'text_compact': compact,
        'mean_word_confidence': (sum(confidences) / len(confidences)
                                 if confidences else None),
        'psm': int(psm),
        'stderr': tsv_run.stderr.strip(),
    }


def run_trials(run_dir, output_dir, tessdata_dir, languages, margin, scale, psms):
    run_dir = Path(run_dir).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    tessdata_dir = Path(tessdata_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    plate_records = []
    crop_dir = output_dir / 'crops'
    crop_dir.mkdir(parents=True, exist_ok=True)

    for point in PLATE_POINTS:
        point_dir = run_dir / point
        detections_files = sorted(point_dir.glob('*.detections.json'))
        if not detections_files:
            plate_records.append({'waypoint': point, 'status': 'NO_DETECTION_JSON'})
            continue
        detection_path = detections_files[-1]
        stem = detection_path.name[:-len('.detections.json')]
        raw_path = point_dir / (stem + '.raw.png')
        if not raw_path.is_file():
            plate_records.append({'waypoint': point, 'status': 'NO_RAW_IMAGE'})
            continue
        image = cv2.imread(str(raw_path), cv2.IMREAD_COLOR)
        if image is None:
            plate_records.append({'waypoint': point, 'status': 'RAW_IMAGE_READ_FAILED'})
            continue
        detection_record = json.loads(detection_path.read_text(encoding='utf-8'))
        detections = [item for item in detection_record.get('detections', [])
                      if item.get('class') == 'license_plate']
        if not detections:
            plate_records.append({'waypoint': point, 'status': 'NO_LICENSE_PLATE_BOX'})
            continue

        for index, detection in enumerate(detections):
            crop = _crop_with_margin(image, detection['box'], margin)
            if crop.size == 0:
                plate_records.append({'waypoint': point, 'index': index,
                                      'status': 'EMPTY_CROP'})
                continue
            crop_stem = '%s_%s_%02d' % (run_dir.name, point, index)
            source_crop = crop_dir / (crop_stem + '_margin%d.png' % margin)
            cv2.imwrite(str(source_crop), crop)
            candidates = []
            for variant, pixels in _variants(crop, scale).items():
                variant_path = crop_dir / (crop_stem + '_%s_x%d.png' % (variant, scale))
                cv2.imwrite(str(variant_path), pixels)
                for psm in psms:
                    result = _run_tesseract(variant_path, tessdata_dir, languages, psm)
                    candidates.append({'variant': variant, **result})
            best = max(candidates,
                       key=lambda item: item['mean_word_confidence']
                       if item['mean_word_confidence'] is not None else -1.0)
            plate_records.append({
                'waypoint': point,
                'source_stamp': detection_record.get('source_stamp'),
                'confidence': float(detection.get('confidence', 0.0)),
                'box': detection.get('box'),
                'crop_path': str(source_crop),
                'status': 'OCR_TRIAL_COMPLETE',
                'best_candidate': best,
                'all_candidates': candidates,
            })

    result = {
        'run_dir': str(run_dir),
        'tessdata_dir': str(tessdata_dir),
        'languages': languages,
        'crop_margin_px': int(margin),
        'scale_factor': int(scale),
        'psm_trials': [int(value) for value in psms],
        'navigation_or_traffic_control_uses_ocr': False,
        'points': plate_records,
    }
    report = output_dir / 'ocr_results.json'
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n',
                      encoding='utf-8')
    return report, plate_records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True,
                        help='one closed accepted route-run directory')
    parser.add_argument('--output-dir', required=True,
                        help='directory for crops and OCR trial JSON')
    parser.add_argument('--tessdata-dir', default='',
                        help='defaults to models/ocr/tessdata beside this script')
    parser.add_argument('--languages', default='chi_sim+eng')
    parser.add_argument('--margin', type=int, default=6,
                        help='crop extension; 6 px matches tl_vision plate_crop')
    parser.add_argument('--scale', type=int, default=4)
    parser.add_argument('--psm', type=int, nargs='+', default=[7, 13])
    args = parser.parse_args()

    tessdata = Path(args.tessdata_dir) if args.tessdata_dir else (
        Path(__file__).resolve().parents[1] / 'models' / 'ocr' / 'tessdata')
    try:
        report, points = run_trials(args.run_dir, args.output_dir, tessdata,
                                    args.languages, args.margin, args.scale,
                                    args.psm)
    except (FileNotFoundError, RuntimeError, subprocess.TimeoutExpired) as error:
        print('OCR trial failed: %s' % error, file=sys.stderr)
        return 1
    complete = [point for point in points
                if point.get('status') == 'OCR_TRIAL_COMPLETE']
    recognized = [point for point in complete
                  if point.get('best_candidate', {}).get('text_compact')]
    print('OCR trial saved: %s; completed=%d recognized_nonempty=%d/%d' % (
        report, len(complete), len(recognized), len(complete)))
    for point in complete:
        candidate = point['best_candidate']
        print('%s: %r (mean word confidence=%s)' % (
            point['waypoint'], candidate['text_raw'],
            candidate['mean_word_confidence']))
    return 0 if complete else 2


if __name__ == '__main__':
    raise SystemExit(main())
