#!/usr/bin/env python3
"""Create a local review page from independently accepted OCR runs."""
import argparse
import html
import json
import os
from pathlib import Path


def render(runs, output):
    output.parent.mkdir(parents=True,exist_ok=True)
    sections = []
    accepted = []
    for run in runs:
        audit_path = run/'independent_audit.json'
        if not audit_path.is_file():
            continue
        audit = json.loads(audit_path.read_text())
        if not audit['acceptance_pass']:
            continue
        accepted.append(audit)
        cards = []
        for plate in audit['plate_audit']['plates']:
            point = plate['waypoint']
            job = json.loads((run/'ocr'/(point+'.job.json')).read_text())
            image = run/'ocr'/(point+'.annotated.png')
            crop = run/'ocr'/Path(job['frames'][0]['crop_image']).name
            full_link = html.escape(os.path.relpath(image,output.parent),quote=True)
            crop_link = html.escape(os.path.relpath(crop,output.parent),quote=True)
            box = job['frames'][0]['box']
            cards.append('<article><h3>%s · %s</h3><p>核对一致，置信度 %.3f；高清框 %d×%d</p>'
                         '<img class="crop" src="%s"><details><summary>查看标注原图</summary>'
                         '<a href="%s"><img class="full" src="%s"></a></details></article>' % (
                             html.escape(point),html.escape(plate['text']),plate['confidence'],
                             box[2]-box[0],box[3]-box[1],crop_link,full_link,full_link))
        duration = audit['bag_audit']['mission_duration_s']
        sections.append('<section><h2>%s</h2><p>完整任务 %.1f 秒；普通白线接触 0、未授权通行 0；'
                        '三牌整牌核对通过。</p><div class="cards">%s</div></section>' % (
                            html.escape(run.name),duration,''.join(cards)))
    unique = {p['text'] for a in accepted for p in a['plate_audit']['plates']}
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>高清车牌 OCR 验证</title>
<style>body{font:16px/1.6 sans-serif;background:#f5f7fa;color:#172638;max-width:1300px;margin:30px auto;padding:0 20px}
.cards{display:flex;gap:16px;flex-wrap:wrap}article{background:white;padding:18px;border-radius:10px;flex:1;min-width:270px}
.crop{max-width:100%%;min-height:100px;object-fit:contain}.full{width:100%%}section{margin:35px 0}summary{cursor:pointer;color:#165c95}</style>
<h1>高清车牌 OCR 验证</h1><p>独立审计通过 %d 轮，共 %d 次整牌识别，覆盖 %d 个不同号码。</p>
<p>拍照相机 1920×1440，POINT_8 前移 20 厘米；每次任务换三张车牌并排除上一批。
原始识别、规范化号码及多帧证据保存在各轮 OCR JSON 中。当前结果针对本仿真场景。</p>%s</html>''' % (
        len(accepted),len(accepted)*3,len(unique),''.join(sections))
    output.write_text(page,encoding='utf-8')
    return output


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    print(render(args.runs,args.output))
