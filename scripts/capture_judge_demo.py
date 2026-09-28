#!/usr/bin/env python3
"""Capture the arranged live judge desktop at key task stages."""
import argparse
import json
from pathlib import Path
import time
from PIL import ImageGrab

parser=argparse.ArgumentParser();parser.add_argument('run_dir',type=Path);args=parser.parse_args()
directory=args.run_dir/'judge';screens=directory/'screenshots';screens.mkdir(exist_ok=True)
saved=set();deadline=time.monotonic()+600
while time.monotonic()<deadline:
    state_path=directory/'dashboard_state.json'
    if state_path.is_file():
        try:state=json.loads(state_path.read_text())
        except ValueError:time.sleep(.2);continue
        criteria=[('01_traffic',state['goal'] in ['POINT_1','POINT_2'] and state['frame_pairs']>5),
                  ('02_people',state['people_counts']['stranger']>0),('03_ocr',len(state['ocr'])>0),
                  ('04_complete',state['route_status']=='COMPLETE_PARKED')]
        for name,ready in criteria:
            if ready and name not in saved:
                time.sleep(.5);ImageGrab.grab(xdisplay=':1').save(screens/(name+'.png'));saved.add(name)
        if '04_complete' in saved:break
    time.sleep(.2)
