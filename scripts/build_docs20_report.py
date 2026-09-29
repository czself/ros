#!/usr/bin/env python3
"""Build an inclusive report from all twenty closed, independently audited runs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import statistics

SOURCE_COMMIT = '5efd120a3f9ce9a3dde9d9cea584ac89d8cda5fc'
RUN_NAMES = (['20260929_docs20_%02d' % n for n in range(1, 10)] +
             ['20260929_docs20b_%02d' % n for n in range(11, 22)])


def read(path):
    return json.loads(path.read_text())


def stats(values):
    return {'n': len(values), 'mean': statistics.mean(values),
            'median': statistics.median(values), 'min': min(values),
            'max': max(values), 'sample_stddev': statistics.stdev(values)
            if len(values) > 1 else 0}


def build(root, output):
    rows, detail = [], {}
    for ordinal, name in enumerate(RUN_NAMES, 1):
        run = root / name
        summary = read(run/'run_summary.json')
        audit = read(run/'independent_audit.json')
        phase = read(run/'navigation_phase_metrics.json')
        supplement = read(run/'docs20_supplementary.json')
        judge = (read(run/'judge/presentation_audit.json')
                 if (run/'judge/presentation_audit.json').exists() else None)
        bag, park = audit['bag_audit'], summary['parking']
        gt = supplement['physical_home']
        p7 = phase['by_goal']['POINT_7']
        people = summary['person_report']['people']
        outsiders = {p['street']: p['confidence'] for p in people
                     if p['class'] == 'stranger'}
        home_pose_ok = (park['position_error_m'] <= .03 and
                        park['heading_error_rad'] <= .04 and
                        park['linear_speed_mps'] < .01 and
                        park['angular_speed_rps'] < .01 and
                        park['stationary_seconds'] >= 2)
        person_checks = audit['person_audit'].get('checks', {})
        plates = audit['plate_audit'].get('plates', [])
        goal7 = next(g for g in summary['goals'] if g['name'] == 'POINT_7')
        row = {
            'ordinal': ordinal, 'run': name, 'source_commit': SOURCE_COMMIT,
            'route_status': summary['route_status'],
            'independent_acceptance': audit['acceptance_pass'],
            'navigation_map_checks_pass': all([audit['goal_pass'], audit['photo_pass'],
                                               audit['bag_safety_pass'],
                                               audit['speed_pass'], home_pose_ok]),
            'mission_sim_s': bag['mission_duration_s'],
            'signal_wait_estimate_s': phase['signal_wait_estimate_s'],
            'mission_excluding_signal_estimate_s':
                bag['mission_duration_s']-phase['signal_wait_estimate_s'],
            'point7_goal_s': goal7['duration_s'],
            'point7_final_segments': p7['final_segments'],
            'point7_postfinal_path_reentries': p7['reentries_after_final'],
            'point7_map_final_sign_crossings':
                supplement['point7']['map_final_error_sign_crossings_over_001rad'],
            'ordinary_white_contacts': bag['ordinary_paint_contact_samples'],
            'out_of_bounds_samples': bag['out_of_bounds_pose_samples'],
            'unauthorized_contacts': len(bag['unauthorized_conditional_contacts']),
            'green_authorizations': len(bag['green_authorization_transitions']),
            'green_authorizations_valid': bag['all_green_authorizations_valid'],
            'person_total': summary['person_report']['counts']['total'],
            'resident_total': summary['person_report']['counts']['resident'],
            'stranger_total': summary['person_report']['counts']['stranger'],
            'person_reported_inventory_match': (
                summary['person_report']['counts'] == {'total':18,'resident':16,'stranger':2}
                and all(summary['person_report']['street_counts'][street] ==
                        {'total':9,'resident':8,'stranger':1} for street in ('A','B'))),
            'person_full_audit_pass': audit['person_report_pass'],
            'person_count_matches_truth': person_checks.get('counts_match_truth', False),
            'person_all_classes_correct': person_checks.get('all_classes_correct', False),
            'person_min_confidence': min(p['confidence'] for p in people),
            'foreign_A_confidence': outsiders['A'], 'foreign_B_confidence': outsiders['B'],
            'ocr_exact_count': sum(p['exact_match'] for p in plates),
            'ocr_consensus_count': sum(p['valid_consensus'] for p in plates),
            'ocr_expected_count': 3, 'plate_audit_pass': audit['plate_ocr_pass'],
            'ocr_min_reported_confidence': min(p['confidence'] for p in summary['ocr_results']),
            'home_amcl_position_cm': 100*park['position_error_m'],
            'home_amcl_heading_rad': park['heading_error_rad'],
            'home_amcl_tolerance_pass': home_pose_ok,
            'home_physical_chassis_return_cm': gt['chassis_return_error_cm'],
            'home_physical_axle_return_cm': gt['axle_return_error_cm'],
            'home_physical_yaw_return_rad': gt['yaw_return_error_rad'],
            'home_physical_3cm_004rad_pass': gt['chassis_within_3cm_004rad'],
            'matched_commands': bag['command_decision_pairing']['matched_commands'],
            'command_pairing_pass': bag['command_decision_pairing']['pass'],
            'judge_presentation_pass': judge['pass'] if judge else None,
            'bag_size_bytes': (run/'motion.bag').stat().st_size,
            'failed_audit_flags': ';'.join(k for k in ('goal_pass','photo_pass','parking_pass',
                'speed_pass','bag_safety_pass','person_report_pass','plate_ocr_pass')
                if audit[k] is False)}
        rows.append(row)
        detail[name] = {'goals': summary['goals'], 'ocr': [
            {k: p.get(k) for k in ('waypoint','text','expected','confidence',
                                  'exact_match','valid_consensus')} for p in plates],
            'point7': supplement['point7'], 'physical_home': gt,
            'person_audit_checks': person_checks}
    output.mkdir(parents=True, exist_ok=True)
    passed = [r for r in rows if r['independent_acceptance']]
    aggregates = {
        'missions': len(rows), 'original_independent_passes': len(passed),
        'original_independent_success_rate': len(passed)/len(rows),
        'navigation_map_checks_passes': sum(r['navigation_map_checks_pass'] for r in rows),
        'person_count_matches_truth_runs': sum(r['person_count_matches_truth'] for r in rows),
        'person_reported_inventory_match_runs': sum(r['person_reported_inventory_match'] for r in rows),
        'person_full_audit_pass_runs': sum(r['person_full_audit_pass'] for r in rows),
        'person_all_classes_correct_runs': sum(r['person_all_classes_correct'] for r in rows),
        'ocr_exact_characters': sum(r['ocr_exact_count'] for r in rows),
        'ocr_consensus_successes': sum(r['ocr_consensus_count'] for r in rows),
        'ocr_total': 3*len(rows),
        'physical_home_passes': sum(r['home_physical_3cm_004rad_pass'] for r in rows),
        'all_mission_time_sim_s': stats([r['mission_sim_s'] for r in rows]),
        'accepted_mission_time_sim_s': stats([r['mission_sim_s'] for r in passed]) if passed else None,
        'point7_goal_s': stats([r['point7_goal_s'] for r in rows]),
        'physical_chassis_return_cm': stats([r['home_physical_chassis_return_cm'] for r in rows]),
        'physical_yaw_return_rad': stats([r['home_physical_yaw_return_rad'] for r in rows]),
        'amcl_home_position_cm': stats([r['home_amcl_position_cm'] for r in rows]),
        'amcl_home_yaw_rad': stats([r['home_amcl_heading_rad'] for r in rows]),
        'person_min_confidence': min(r['person_min_confidence'] for r in rows),
        'all_point7_postfinal_reentries': sum(r['point7_postfinal_path_reentries'] for r in rows),
        'all_point7_map_final_sign_crossings': sum(r['point7_map_final_sign_crossings'] for r in rows),
        'matched_commands': sum(r['matched_commands'] for r in rows),
        'judge_tested_runs': sum(r['judge_presentation_pass'] is not None for r in rows),
        'judge_passed_runs': sum(r['judge_presentation_pass'] is True for r in rows)}
    choices = {}
    if passed:
        choices['fastest_accepted'] = min(passed, key=lambda r:r['mission_sim_s'])['run']
        display = [r for r in passed if r['judge_presentation_pass'] is True] or passed
        median = aggregates['accepted_mission_time_sim_s']['median']
        choices['representative_with_display'] = min(display, key=lambda r:abs(r['mission_sim_s']-median))['run']
        choices['strong_visual_scores'] = max(passed, key=lambda r:min(
            r['foreign_A_confidence'],r['foreign_B_confidence'],r['ocr_min_reported_confidence']))['run']
    payload = {'source_commit': SOURCE_COMMIT, 'runtime_commit':'d9a61b4',
               'aggregates': aggregates, 'selected_examples': choices,
               'selection_note':'Examples are labeled; all twenty runs and all failures retained.',
               'rows':rows, 'details':detail,
               'raw_data_root':str(root),
               'startup_attempts_outside_twenty_missions':[
                   {'run':'20260929_docs20_10','reason':'BadWindow before task dispatch'},
                   {'run':'20260929_docs20_11','reason':'Operator batch restart, exit 141, no task result'},
                   {'run':'20260929_docs20_12','reason':'Batch stopped during startup, no task dispatch'}]}
    (output/'results.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2)+'\n')
    with (output/'all_twenty_runs.csv').open('w', newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (output/'all_point_timings.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['run','point','duration_s','action_state','position_error_m','heading_error_rad','attempts'])
        for name,item in detail.items():
            for g in item['goals']:
                writer.writerow([name,*[g.get(k) for k in ('name','duration_s','action_state',
                    'position_error_m','heading_error_rad','attempts')]])
    evidence=output/'evidence'
    for name in RUN_NAMES:
        dest=evidence/name;dest.mkdir(parents=True,exist_ok=True)
        run=root/name
        for name2 in ('independent_audit.json','navigation_phase_metrics.json',
                      'docs20_supplementary.json','run_summary.json','person_audit.json',
                      'plate_audit.json','people_terminal.txt','person_report.txt',
                      'person_report.audio.json','route_executor.log','ocr/ocr_terminal.txt',
                      'judge/presentation_audit.json'):
            src=run/name2
            if src.exists():
                target=dest/name2;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target)
    for name in set(choices.values()) | {r['run'] for r in rows if not r['plate_audit_pass']}:
        run=root/name;dest=evidence/name
        for folder in [run/('POINT_%d'%n) for n in range(1,11)]+[run/'ocr',run/'persons',run/'judge/screenshots']:
            if folder.exists():
                for src in folder.rglob('*'):
                    if src.is_file() and src.suffix.lower() in ('.png','.json'):
                        target=dest/src.relative_to(run);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target)
    legacy=root/'legacy_bag_cleanup_20260929.json'
    if legacy.exists():shutil.copy2(legacy,output/legacy.name)
    manifest=Path('/tmp/docs20_manifest.tsv')
    if manifest.exists():shutil.copy2(manifest,output/'original_launcher_manifest.tsv')
    for name in RUN_NAMES + ['20260929_docs20_10','20260929_docs20_11','20260929_docs20_12']:
        log=Path('/tmp')/(name+'.launcher.log')
        if log.exists():
            target=output/'launcher_logs'/log.name
            target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(log,target)
    return payload


def plots(output, payload):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=payload['rows'];x=[r['ordinal'] for r in rows]
    fig,ax=plt.subplots(figsize=(11,4.2))
    colors=['#16877c' if r['independent_acceptance'] else '#bf4747' for r in rows]
    ax.bar(x,[r['mission_sim_s'] for r in rows],color=colors)
    ax.set(xlabel='Mission (all 20 trials)',ylabel='Simulation seconds',xticks=x,
           title='Complete task time; red = independent audit failed')
    ax.axhline(statistics.median(r['mission_sim_s'] for r in rows),linestyle='--',color='#445466',label='All-run median')
    ax.legend();fig.tight_layout();fig.savefig(output/'mission_times.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4.2))
    axes[0].plot(x,[r['home_amcl_position_cm'] for r in rows],'o-',label='AMCL contract error')
    axes[0].plot(x,[r['home_physical_chassis_return_cm'] for r in rows],'s-',label='Physical chassis return')
    axes[0].axhline(3,color='#bf4747',linestyle='--');axes[0].set(xlabel='Mission',ylabel='Position error (cm)',title='HOME position: two measurement definitions')
    axes[1].plot(x,[r['home_amcl_heading_rad'] for r in rows],'o-',label='AMCL contract error')
    axes[1].plot(x,[r['home_physical_yaw_return_rad'] for r in rows],'s-',label='Physical chassis return')
    axes[1].axhline(.04,color='#bf4747',linestyle='--');axes[1].set(xlabel='Mission',ylabel='Heading error (rad)',title='HOME heading')
    for ax in axes:ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(output/'home_errors.png',dpi=180);plt.close(fig)


def markdown(output, payload):
    a=payload['aggregates'];rows=payload['rows'];t=a['all_mission_time_sim_s']
    p=a['accepted_mission_time_sim_s'];phys=a['physical_chassis_return_cm']
    lines=['# 智算三行队：20次完整任务数据（2026-09-29）','',
        '本报告保留全部任务结果和启动中断记录，展示样例不替代总体统计。运行源码来自 '+
        '`main` 提交 `5efd120`，运行冻结代码 `d9a61b4`；导航43个冻结文件一致。统计工具只读取已关闭的bag。','',
        '## 样本口径','',
        '共20次实际发车的十点任务：前9次为 `docs20_01…09`，后11次为 `docs20b_11…21`。'+
        '每次重启仿真、随机抽取3块车牌，原速度/白线/绿灯/识别阈值保留。'+
        '第10次窗口BadWindow、随后11/12启动中断另列，未冒充任务样本。补跑21凑齐20次。'+
        '运行过程还有准备阶段 `docs_data_1` 同名ROS节点被替换的中断，原目录保留，不在本批20次中。','',
        '前9次开启比赛展示，后11次关闭judge窗口：'+
        '原图/标注、检测、人物/车牌源帧、终端日志与bag继续保存；后11次没有judge_event配对记录，'+
        '不能宣称这11次通过了比赛展示图文审计。GUI开销存在差异，耗时不能解释为完全相同的显示条件。','',
        '时长为motion bag中的仿真秒，包含任务内OCR等待；不包含场景/模型启动和完成后的约26秒语音。'+
        '信号等待和各控制阶段时长为事件积分估计，不可相加。成功率用全部20次作分母，未剔除OCR失败。','',
        '## 全部样本统计','',
        '| 指标 | 结果 |','|---|---|',
        f"| 原独立完整验收 | {a['original_independent_passes']}/20（{100*a['original_independent_success_rate']:.1f}%） |",
        f"| 导航/白线/绿灯/速度/AMCL回位分项 | {a['navigation_map_checks_passes']}/20 |",
        f"| 报告人数18/16/2、A/B各9与场景库存一致 | {a['person_reported_inventory_match_runs']}/20 |",
        f"| 人物完整独立验收（含播报） | {a['person_full_audit_pass_runs']}/20 |",
        f"| 已完成独立审计中人数核对 / 类别全部正确 | {a['person_count_matches_truth_runs']} / {a['person_all_classes_correct_runs']} 轮 |",
        f"| 车牌整牌字符正确 | {a['ocr_exact_characters']}/{a['ocr_total']} |",
        f"| OCR多帧一致且达到原阈值 | {a['ocr_consensus_successes']}/{a['ocr_total']} |",
        f"| 全部20次任务均值 / 中位数 | {t['mean']:.3f} / {t['median']:.3f} 秒 |",
        f"| 全部20次最短 / 最长 / 样本标准差 | {t['min']:.3f} / {t['max']:.3f} / {t['sample_stddev']:.3f} 秒 |",
        f"| 通过轮次均值（n={p['n'] if p else 0}） | {p['mean']:.3f} 秒 |" if p else '| 通过轮次均值 | 无通过轮次 |',
        f"| 第7点对齐后回到路径模式总次数 | {a['all_point7_postfinal_reentries']} |",
        f"| 第7点最终对齐map估计航向误差变号（>0.01rad）总次数 | {a['all_point7_map_final_sign_crossings']} |",
        f"| 比赛展示图文对应审计 | {a['judge_passed_runs']}/{a['judge_tested_runs']}；其余未采集 |",
        f"| 同周期命令配对 | {a['matched_commands']} 条 |",'',
        '人物置信度取模型原始输出，全体最低为 '+f"{a['person_min_confidence']:.5f}"+
        '。人数/类别正确不等于模型概率已校准或训练问题已经解决。'+
        'OCR失败轮次未进入完成播报流程，原人物审计因缺音频证据不能完成；'+
        '报告人数一致与完整人物验收通过分开统计，缺失证据不标成通过。','',
        '![全部任务时长](mission_times.png)','',
        '## HOME：定位验收与实际车身回位','',
        '原完整审计使用AMCL/map合同位姿。补充检查以同一bag第一帧和最后一帧Gazebo车身真值相减，'+
        '直接检查相对出发位置和方向；不使用未标定的map/world绝对坐标差。车身与轮轴结果均保留。','',
        f"实际车身满足3厘米和0.04弧度的轮次：{a['physical_home_passes']}/20。"+
        f"车身位置差均值{phys['mean']:.3f}厘米，范围{phys['min']:.3f}–{phys['max']:.3f}厘米；"+
        f"方向差均值{a['physical_yaw_return_rad']['mean']:.5f}弧度。",
        '因此不能把原独立审计通过写成实际车身精确回到原位、原方向。该差异单独报告，当前用户已要求冻结导航，'+
        '本次未调参或修改控制器。是否满足物理回位要求，应看这一列，不能只看AMCL列。','',
        '![HOME两种测量](home_errors.png)','',
        '## 每次任务明细','',
        '| 序号 | RUN_ID | 原完整审计 | 时长s | 第7点s/回转次数 | 人数 | OCR整牌 | AMCL回位cm | 实际车身回位cm/rad |',
        '|---:|---|---|---:|---|---:|---:|---:|---|']
    for r in rows:
        lines.append(f"| {r['ordinal']} | [{r['run']}](evidence/{r['run']}/independent_audit.json) | "+
            ('通过' if r['independent_acceptance'] else '失败')+
            f" | {r['mission_sim_s']:.3f} | {r['point7_goal_s']:.2f}/{r['point7_postfinal_path_reentries']} | "+
            f"{r['person_total']} | {r['ocr_exact_count']}/3 | {r['home_amcl_position_cm']:.3f} | "+
            f"{r['home_physical_chassis_return_cm']:.3f}/{r['home_physical_yaw_return_rad']:.5f} |")
    lines += ['', '第7点回转列仅指FINAL_HEADING之后重新进入行驶/补位模式。'+
        '它不能排除此前路径对齐先越过拍照朝向、随后摆正的动作。新增map航向误差变号检查只覆盖最终对齐/稳定阶段，'+
        '不能用来宣称第7点全程没有用户观察到的大转向。','', '## 失败与展示样例','']
    for r in rows:
        if not r['independent_acceptance']:
            lines.append(f"- `{r['run']}`：原审计失败项 `{r['failed_audit_flags']}`。"+
                         '参阅原审计和对应源图，未回写结果。')
    for label,name in payload['selected_examples'].items():
        lines.append(f'- `{label}`：`{name}`，只作为明确标注的展示样例。')
    lines += ['', 'PPT可展示示例照片、终端与OCR裁剪，并同时报告20次总体通过率、时长范围及物理回位差异。'+
        '不使用最佳样例的耗时或置信度冒充均值，不写20/20完美任务。','',
        '## 数据与复现','',
        '- [全部20轮CSV](all_twenty_runs.csv)，[全部点位时长CSV](all_point_timings.csv)，[结构化统计与选样规则](results.json)。',
        '- `evidence/` 保存全部20轮审计、汇总、关键日志；示例和失败轮次另存照片/OCR图及元数据。',
        '- 完整motion bag、深度数组、全量高清帧在本机 '+f"`{payload['raw_data_root']}/<RUN_ID>`，未上传GitHub。",
        '- 92个2026-09-27/28旧bag已按用户要求删除，释放约406.22GiB；原照片/结果仍在。'+
          '删除清单见 `legacy_bag_cleanup_20260929.json`，原bag不能从GitHub结果文件恢复。'+
          '本批20个bag和之前最终验证三轮bag仍保留。',
        '- 离线复现：在ROS环境先运行原审计（带 `--require-command-decisions`）、阶段统计和 '+
          '`analyze_docs20_run.py <RUN_DIR>`；再运行 `build_docs20_report.py --output <OUTPUT_DIR>`。', '']
    (output/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=Path,default=Path('/home/sz/ros1_ws/photo_stops/standee_route_runs'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();payload=build(args.runs,args.output);plots(args.output,payload);markdown(args.output,payload)
    print(json.dumps(payload['aggregates'],ensure_ascii=False,indent=2))
