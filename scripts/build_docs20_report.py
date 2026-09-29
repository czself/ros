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
            'point7_physical_final_sign_crossings':
                supplement['point7']['physical_final_error_sign_crossings_over_001rad'],
            'ordinary_white_contacts': bag['ordinary_paint_contact_samples'],
            'out_of_bounds_samples': bag['out_of_bounds_pose_samples'],
            'unauthorized_contacts': len(bag['unauthorized_conditional_contacts']),
            'green_authorizations': len(bag['green_authorization_transitions']),
            'green_authorizations_valid': bag['all_green_authorizations_valid'],
            'person_total': summary['person_report']['counts']['total'],
            'resident_total': summary['person_report']['counts']['resident'],
            'stranger_total': summary['person_report']['counts']['stranger'],
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
        'all_point7_final_sign_crossings': sum(r['point7_physical_final_sign_crossings'] for r in rows),
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


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=Path,default=Path('/home/sz/ros1_ws/photo_stops/standee_route_runs'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();payload=build(args.runs,args.output);plots(args.output,payload)
    print(json.dumps(payload['aggregates'],ensure_ascii=False,indent=2))
