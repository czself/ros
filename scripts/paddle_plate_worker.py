#!/usr/bin/env python3
"""Independent host OCR worker; reads camera jobs, never scene labels or truth."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK','True')
os.environ.setdefault('PADDLE_PDX_MODEL_SOURCE','BOS')
os.environ.setdefault('OMP_NUM_THREADS','2')
sys.path.insert(0,str(ROOT/'ros_packages/tl_vision/src'))
import cv2
import numpy as np
from tl_vision.ocr import PlateOcr


def host_path(path):
    prefix = '/root/ros1_ws/'
    return Path('/home/sz/ros1_ws'/Path(path).relative_to(prefix)) if path.startswith(prefix) else Path(path)


def atomic_json(path, data):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temporary.replace(path)


def result_record(plate, elapsed):
    return {'raw':plate.raw,'text':plate.text,'display':plate.display,
            'confidence':plate.confidence,'valid':plate.is_valid,'issue':plate.issue,
            'elapsed_ms':round(elapsed*1000,3)}


def recognition_band(image):
    height,width = image.shape[:2]
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    _,mask = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    _,_,stats,_ = cv2.connectedComponentsWithStats(mask)
    glyphs = [s for s in stats[1:] if s[3]>=height*.25 and
              s[4]>=height*width*.002 and s[2]<width*.25]
    if 5 <= len(glyphs) <= 12:
        padding = max(2,int(round(height*.03)))
        top = max(0,min(s[1] for s in glyphs)-padding)
        bottom = min(height,max(s[1]+s[3] for s in glyphs)+padding)
        if .30*height <= bottom-top <= .85*height:
            return image[top:bottom], [0,int(top),width,int(bottom)]
    return image,[0,0,width,height]


def predict(engine, image, input_path=None):
    started = time.perf_counter()
    band, bounds = recognition_band(image)
    if input_path:
        cv2.imwrite(str(input_path),band)
    result = result_record(engine.read(band),time.perf_counter()-started)
    result['recognition_band'] = bounds
    result['input_shape'] = list(band.shape)
    return result


def process_job(engine, path, compare_low=False):
    started = time.perf_counter()
    job = json.loads(path.read_text())
    frames, issues = [], []
    for item in job['frames']:
        crop_path = host_path(item['crop_image'])
        image = cv2.imread(str(crop_path))
        if image is None:
            raise RuntimeError('MISSING_OCR_CROP:'+str(crop_path))
        input_path = crop_path.with_name(crop_path.stem+'.input.png')
        result = predict(engine,image,input_path)
        frames.append(dict(result,crop_image=item['crop_image'],
                           recognition_image=str(input_path),
                           recognition_image_sha256=hashlib.sha256(input_path.read_bytes()).hexdigest(),
                           source_stamp=item['source_stamp'],box=item['box'],
                           image_sha256=hashlib.sha256(crop_path.read_bytes()).hexdigest()))
    votes = Counter(f['text'] for f in frames if f['valid'])
    text, count = votes.most_common(1)[0] if votes else ('',0)
    valid = count >= 2
    if not valid:
        issues.append('NO_TWO_FRAME_VALID_CONSENSUS')
    confidence = (min(f['confidence'] for f in frames if f['valid'] and f['text']==text)
                  if valid else 0.0)
    report = {'waypoint':job['waypoint'],'text':text if valid else '',
              'valid':valid,'confidence':confidence,'matching_frames':count,
              'frames':frames,'issues':issues,'job_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
              'engine':'PaddleOCR recognition-only','model_name':'PP-OCRv5_mobile_rec',
              'source':'camera pixels only; no material names or scene truth',
              'total_elapsed_ms':round((time.perf_counter()-started)*1000,3)}
    if compare_low:
        point_dir = path.parent.parent/job['waypoint']
        detections_path = next(point_dir.glob('*.detections.json'))
        record = json.loads(detections_path.read_text())
        plate = max([p for p in record['detections'] if p['class']=='license_plate'],
                    key=lambda p:p['confidence'])
        x1,y1,x2,y2 = map(int,plate['box'])
        raw = cv2.imread(str(next(point_dir.glob('*.raw.png'))))
        report['low_resolution_comparison'] = dict(predict(engine,raw[y1:y2,x1:x2]),
                                                   box=plate['box'])
    atomic_json(path.with_name(job['waypoint']+'.result.json'),report)
    print(json.dumps({'waypoint':job['waypoint'],'text':report['text'],
                      'valid':valid,'confidence':confidence,'elapsed_ms':report['total_elapsed_ms']},
                     ensure_ascii=False),flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--watch',action='store_true')
    parser.add_argument('--compare-low',action='store_true')
    parser.add_argument('--ready-file',type=Path)
    args = parser.parse_args()
    model_base = ROOT/'models/ocr/paddle'
    manifest = json.loads((model_base/'manifest.json').read_text())
    for name, digest in manifest['files'].items():
        if hashlib.sha256((model_base/name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('OCR_MODEL_SHA_MISMATCH:'+name)
    engine = PlateOcr(recognition_only=True,model_name='PP-OCRv5_mobile_rec',
                      model_dir=str(model_base/'PP-OCRv5_mobile_rec_infer'),
                      strict=True,min_conf=.85,cpu_threads=2,enable_mkldnn=False)
    engine.read(np.zeros((48,160,3),dtype=np.uint8))
    ready = {'status':'READY','pid':os.getpid(),'model_manifest_sha256':hashlib.sha256(
        (model_base/'manifest.json').read_bytes()).hexdigest(),'api':engine.api_version}
    if args.ready_file:
        atomic_json(args.ready_file,ready)
    print(json.dumps(ready),flush=True)
    seen = set()
    while True:
        directory = args.run_dir/'ocr'
        if directory.is_dir():
            for path in sorted(directory.glob('*.job.json')):
                if path.name in seen:
                    continue
                process_job(engine,path,args.compare_low)
                seen.add(path.name)
        if not args.watch or len(seen) >= 3:
            break
        time.sleep(.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
