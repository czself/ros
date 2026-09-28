#!/usr/bin/env python3
"""Snapshot texture provenance solely for independent OCR evaluation."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    base = Path('/root/car_standees')
    inventory = {}
    for path in (base/'plate_inventory').iterdir():
        if path.is_file() and path.stat().st_size:
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            inventory.setdefault(digest,set()).add(path.stem)
    material = (base/'materials/scripts/car_standees.material').read_text()
    plates = {}
    for name,block in re.findall(r'material\s+CarStandee/Plate(\d+)(.*?)(?=\nmaterial|\Z)',material,re.S):
        filename = re.search(r'\btexture\s+(\S+)',block).group(1)
        digest = hashlib.sha256((base/'materials/textures'/filename).read_bytes()).hexdigest()
        labels = inventory.get(digest,set())
        if len(labels)!=1:
            raise RuntimeError('Cannot uniquely establish scene texture truth:'+filename)
        plates['car_standee_plate_'+name] = {'expected':next(iter(labels)),
                                            'texture':filename,'texture_sha256':digest}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'scope':'Evaluation only; never read by OCR worker',
        'material_sha256':hashlib.sha256(material.encode()).hexdigest(),'plates':plates},
        ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
