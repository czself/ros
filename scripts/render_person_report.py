#!/usr/bin/env python3
"""Create a local review page from a saved person report without changing evidence."""
import argparse
import html
import json
from pathlib import Path

import cv2
from person_reporting import annotate_people


def render(directory):
    report = json.loads((directory/'person_report.json').read_text())
    for point in sorted({o['waypoint'] for p in report['people'] for o in p['observations']}):
        files = [p for p in (directory/point).glob('*.json')
                 if not p.name.endswith(('.detections.json', '.failure.json'))]
        evidence = json.loads(files[0].read_text())
        raw = cv2.imread(evidence['raw_image'])
        people = [dict(o, person_id=p['person_id']) for p in report['people']
                  for o in p['observations'] if o['waypoint']==point]
        cv2.imwrite(str(directory/'persons'/('review_'+point+'.png')), annotate_people(raw, people))
    escape = html.escape
    def relative(path):
        # Runtime paths are absolute inside the container; keep page links local.
        parts = Path(path).parts
        index = parts.index(directory.name)
        local = '/'.join(parts[index+1:])
        if local.startswith('persons/POINT_'):
            local = 'persons/review_'+Path(local).name
        return escape(local, quote=True)
    rows = []
    for street, counts in report['street_counts'].items():
        rows.append('<tr><td>%s</td><td>%d</td><td>%d</td><td>%d</td></tr>' % (
            escape(street), counts['total'], counts['resident'], counts['stranger']))
    foreign_cards = []
    for person in report['foreign_people']:
        observation = next(o for o in person['observations'] if o.get('foreign_crop_path'))
        foreign_cards.append('<article><h3>%s · %s 街区</h3><p>外来人员，置信度 %.3f</p>'
                             '<img class="crop" src="%s"><p><a href="%s">查看整张证据图</a></p></article>' % (
                                 escape(person['person_id']), escape(person['street']),
                                 person['confidence'], relative(observation['foreign_crop_path']),
                                 relative(observation['image_path'])))
    people_rows = []
    for person in report['people']:
        observation = person['observations'][0]
        people_rows.append('<tr><td>%s</td><td>%s</td><td>%s</td><td>%.3f</td>'
                           '<td><a href="%s">%s</a></td></tr>' % (
                               escape(person['person_id']), escape(person['street']),
                               '外来人员' if person['class']=='stranger' else '社区人员',
                               person['confidence'], relative(observation['image_path']),
                               escape(observation['waypoint'])))
    counts = report['counts']
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>人物识别报告</title>
<style>body{font:16px/1.6 sans-serif;max-width:1050px;margin:32px auto;padding:0 20px;background:#f6f7f9;color:#182331}
table{border-collapse:collapse;width:100%%;background:white;margin:16px 0}th,td{padding:10px;text-align:left;border-bottom:1px solid #ddd}
.cards{display:flex;gap:20px;flex-wrap:wrap}article{padding:20px;background:white;border:2px solid #e99a20;border-radius:10px}
.crop{height:260px;max-width:100%%;object-fit:contain}a{color:#165d9c}audio{max-width:100%%}</style>
<h1>人物识别报告</h1><p>运行：%s</p><p><strong>共 %d 人 · 社区人员 %d 人 · 外来人员 %d 人</strong></p>
<p>统计状态：%s。人员编号用于本次静态场景去重，不代表真实身份认证。</p>
<h2>街区统计</h2><table><tr><th>街区</th><th>总人数</th><th>社区人员</th><th>外来人员</th></tr>%s</table>
<h2>外来人员证据</h2><div class="cards">%s</div>
<h2>中文播报</h2><p>%s</p><audio controls src="person_report.wav"></audio>
<h2>全部人员</h2><table><tr><th>编号</th><th>街区</th><th>类别</th><th>置信度</th><th>源图</th></tr>%s</table>
<p><a href="person_report.json">完整报告</a> · <a href="person_audit.json">人物独立审计</a> ·
<a href="independent_audit.json">路线独立审计</a></p></html>''' % (
        escape(directory.name), counts['total'], counts['resident'], counts['stranger'],
        '观测统计完整' if report['complete'] else '待复核', ''.join(rows), ''.join(foreign_cards),
        escape(report['speech_text']), ''.join(people_rows))
    output = directory/'person_report.html'
    output.write_text(page, encoding='utf-8')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    print(render(args.run_dir))
