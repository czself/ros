#!/usr/bin/env python3
"""Build a复赛 source ZIP and an honest index of the remaining deliverables."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
YOLO_SHA = 'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'
SCRIPTS = {
    'randomize_car_plates.py', 'traffic_light_controller.py',
    'runtime_control.py', 'wheel_encoder_odom.py', 'cmd_vel_watchdog.py',
    'yolo_inspector.py', 'goal_sanitizer.py', 'navigation_goal_safety.py',
    'check_foundation.py', 'check_navigation_readiness.py', 'route_executor.py',
    'person_reporting.py', 'hd_plate_capture.py', 'plate_result_bridge.py',
    'paddle_plate_worker.py', 'record_plate_scene_truth.py', 'judge_monitor.py',
    'capture_judge_demo.py', 'announce_people.py',
    'render_person_report.py', 'setup_paddle_ocr.sh',
    'car_teleop.py', 'model_state_odom.py', 'survey_mapper.py',
    'autonomous_mapper.py', 'coverage_report.py', 'verify_slam_map.py',
    'capture_tf_evidence.py', 'build_white_line_navigation_map.py',
    'build_navigation_map.py', 'generate_ground_truth_map.py',
    'autonomous_mapping.rviz', 'competition.rviz', 'competition_judge.rviz',
    'audit_standee_photo_run.py', 'audit_person_report.py',
    'audit_plate_ocr.py', 'audit_judge_presentation.py',
    'command_decision_audit.py', 'navigation_phase_metrics.py',
    'diagnose_navigation_run.py', 'analyze_docs20_run.py',
}
PACKAGES = ('forward_path_follower', 'safe_escape_recovery', 'tl_vision')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')


def copytree(source, target):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns(
        '.git', '.github', '__pycache__', '*.pyc', '*.pyo',
        '.pytest_cache', 'build', 'devel'), dirs_exist_ok=True)


def adapt_shell(source, target):
    original = source.read_text(encoding='utf-8')
    text = original.replace('CONTAINER=ros1_modeling',
                            'CONTAINER="${INSPECTION_CONTAINER:-ros1_modeling}"')
    text = text.replace('$ROOT_DIR/ros_packages/', '$ROOT_DIR/../')
    text = text.replace('DISPLAY=:1', 'DISPLAY="${DISPLAY:-:1}"')
    text = text.replace('-v /home/sz/ros1_ws:/root/ros1_ws',
                        '-v "${INSPECTION_DATA_DIR:-/home/sz/ros1_ws}:/root/ros1_ws"')
    text = text.replace('-v /home/sz/.gazebo:/root/.gazebo',
                        '-v "${INSPECTION_GAZEBO_DIR:-/home/sz/.gazebo}:/root/.gazebo"')
    text = text.replace('/home/sz/ros1_ws/photo_stops/',
                        '${INSPECTION_DATA_DIR:-/home/sz/ros1_ws}/photo_stops/')
    text = text.replace('osrf/ros:noetic-desktop-full tail',
                        '${INSPECTION_IMAGE:-osrf/ros:noetic-desktop-full} tail')
    text = text.replace('YOLO_CHECKPOINT:-/home/sz/下载/best.pt',
                        'YOLO_CHECKPOINT:-$ROOT_DIR/weights/best.pt')
    if source.name == 'start_navigation.sh':
        marker = '    expected=data.get("source_map_sha256")'
        if marker not in text:
            raise ValueError('Navigation map provenance guard moved')
        text = text.replace(marker,
            '    if source and not os.path.isabs(source):\n'
            '        source=os.path.normpath(os.path.join(os.path.dirname(sys.argv[1]),source))\n'
            + marker)
    write(target, text)
    target.chmod(source.stat().st_mode & 0o777)
    return {'file': source.name, 'upstream_sha256': sha(source),
            'package_sha256': sha(target)} if text != original else None


def build_workspace(workspace, weights, team, commit):
    project = workspace / 'src/community_inspection'
    project.mkdir(parents=True)
    for directory in ('assets', 'insert', 'navigation', 'models'):
        copytree(ROOT / directory, project / directory)
    for package in PACKAGES:
        copytree(ROOT / 'ros_packages' / package, workspace / 'src' / package)
    (project / 'worlds').mkdir()
    shutil.copy2(ROOT / 'worlds/competition_classic_adjusted_20260924.world',
                 project / 'worlds/competition_classic_adjusted_20260924.world')
    map_dir = project / 'maps'
    map_dir.mkdir()
    for pattern in ('current_slam_preview_white_lines.*',
                    'competition_rescan_full_20260922.*'):
        for path in (ROOT / 'maps').glob(pattern):
            if path.is_file():
                shutil.copy2(path, map_dir / path.name)
    for name in ('current_slam_preview.pgm', 'current_slam_preview_local.yaml'):
        shutil.copy2(ROOT / name, project / name)
    map_manifest = map_dir / 'current_slam_preview_white_lines.manifest.json'
    recorded = json.loads(map_manifest.read_text())
    if sha(project / 'current_slam_preview.pgm') != recorded['source_map_sha256']:
        raise ValueError('Packaged SLAM map source hash differs from manifest')
    recorded['source_map'] = '../current_slam_preview.pgm'
    recorded['source_yaml'] = '../current_slam_preview_local.yaml'
    write(map_manifest, json.dumps(recorded, ensure_ascii=False, indent=2) + '\n')
    write(map_dir / 'README.md',
          'current_slam_preview_white_lines.yaml 是默认导航规则图，来源是上级测量地图。\n'
          'competition_rescan_full_20260922.* 是历史建图证据，不替代建图全过程录像。\n')
    script_dir = project / 'scripts'
    script_dir.mkdir()
    changes = []
    for name in sorted(SCRIPTS):
        source = ROOT / 'scripts' / name
        if not source.is_file():
            raise FileNotFoundError(source)
        destination = script_dir / name
        if name.endswith('.sh'):
            change = adapt_shell(source, destination)
            if change:
                changes.append(change)
        else:
            shutil.copy2(source, destination)
    shutil.copy2(ROOT / 'submission/native_demo.py', script_dir / 'native_demo.py')
    worker = script_dir / 'paddle_plate_worker.py'
    original = worker.read_text()
    updated = original.replace("ROOT/'ros_packages/tl_vision/src'",
                               "ROOT.parent/'tl_vision/src'")
    updated = updated.replace("Path('/home/sz/ros1_ws'/Path(path).relative_to(prefix))",
        "Path(os.environ.get('INSPECTION_DATA_DIR', '/home/sz/ros1_ws'))/Path(path).relative_to(prefix)")
    if updated == original or "ROOT/'ros_packages/tl_vision/src'" in updated:
        raise ValueError('OCR worker host/package path adaptation failed')
    write(worker, updated)
    changes.append({'file': worker.name, 'upstream_sha256': sha(ROOT / 'scripts' / worker.name),
                    'package_sha256': sha(worker)})
    truth_script = script_dir / 'record_plate_scene_truth.py'
    text = truth_script.read_text().replace('import argparse', 'import argparse\nimport os')
    text = text.replace("base = Path('/root/car_standees')",
        "base = Path(os.environ.get('CAR_STANDEE_DIR', '/root/car_standees'))")
    write(truth_script, text)
    for name in ('audit_person_report.py', 'audit_plate_ocr.py'):
        path = script_dir / name
        text = path.read_text().replace('import argparse', 'import argparse\nimport os') if name == 'audit_person_report.py' else path.read_text().replace('import hashlib', 'import hashlib\nimport os')
        text = text.replace("Path('/root/competition_classic_adjusted_20260924.world')",
            "Path(os.environ.get('COMPETITION_WORLD_PATH', '/root/competition_classic_adjusted_20260924.world'))")
        text = text.replace("ET.parse('/root/competition_classic_adjusted_20260924.world')",
            "ET.parse(os.environ.get('COMPETITION_WORLD_PATH', '/root/competition_classic_adjusted_20260924.world'))")
        write(path, text)
        changes.append({'file': name, 'upstream_sha256': sha(ROOT / 'scripts' / name),
                        'package_sha256': sha(path)})
    changes.append({'file': truth_script.name,
                    'upstream_sha256': sha(ROOT / 'scripts' / truth_script.name),
                    'package_sha256': sha(truth_script)})
    write(project / 'models/ocr/README.md',
          '当前默认 OCR：本地 PP-OCRv5_mobile_rec，独立主机进程，多帧一致性。\n'
          'tessdata 是历史试验数据，不参与当前识别。\n')
    for name in ('CMakeLists.txt', 'package.xml'):
        shutil.copy2(ROOT / 'submission' / name, project / name)
    (project / 'weights').mkdir()
    shutil.copy2(weights, project / 'weights/best.pt')
    if sha(project / 'weights/best.pt') != YOLO_SHA:
        raise ValueError('Packaged YOLO weights changed')
    write(project / 'README.md',
          '当前源码和模型：从压缩包根目录 README.md 执行环境准备与十点任务。\n')
    for name in ('README.md', 'native_demo.sh', 'native_audit.sh'):
        source = ROOT / 'submission' / ('工程README.md' if name == 'README.md' else name)
        text = source.read_text(encoding='utf-8').replace('{{TEAM}}', team)
        text = text.replace('{{SOURCE_COMMIT}}', commit)
        target = workspace / name
        write(target, text)
        if name.endswith('.sh'):
            target.chmod(0o755)
    return changes


def copy_evidence(output, team, metrics):
    evidence = output / '05_运行证据'
    evidence.mkdir(parents=True)
    source = ROOT / 'docs/agent_context/tasks/task009/docs20_20260929'
    for name in ('REPORT.md', 'results.json', 'all_twenty_runs.csv',
                 'all_point_timings.csv', 'mission_times.png', 'home_errors.png'):
        shutil.copy2(source / name, evidence / name)
    for name in ('representative_with_display', 'strong_visual_scores',
                 'point7_reentry_counterexample'):
        run = metrics['selected_examples'][name]
        original = source / 'evidence' / run
        if name == 'strong_visual_scores':
            for target in (original / 'persons/POINT_2.png',
                           original / 'ocr/POINT_10.annotated.png'):
                dest = evidence / '展示图片' / target.name
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, dest)
        else:
            shutil.copy2(original / 'independent_audit.json',
                         evidence / (run + '.independent_audit.json'))
    return evidence


def copy_reproduction(evidence, run, packaged_workspace):
    """Archive one completed run of the same unpacked runtime, without its bag."""
    audit = json.loads((run / 'independent_audit.json').read_text())
    summary = json.loads((run / 'run_summary.json').read_text())
    original_workspace = run.parent.parent
    original_manifest = json.loads((original_workspace / 'PACKAGE_MANIFEST.json').read_text())
    ignored = {'README.md', 'native_audit.sh'}
    differences = [name for name, expected in original_manifest['files'].items()
                   if name not in ignored and sha(packaged_workspace / name) != expected]
    if differences:
        raise ValueError('Tested runtime differs from final ZIP: ' + ', '.join(differences[:5]))
    if not (audit['acceptance_pass'] and summary['route_status'] == 'COMPLETE_PARKED'
            and audit['plate_ocr_pass'] and audit['person_report_pass']
            and len(summary['goals']) == 11 and (run / 'motion.bag').is_file()):
        raise ValueError('Reproduction evidence is not a completed, audited full task')
    audio = json.loads((run / 'person_report.audio.json').read_text())
    if not audio['synthesis_pass'] or not audio['playback_pass']:
        raise ValueError('Reproduction audio evidence failed')
    folder = evidence / '源码包解压复现_20260930'
    folder.mkdir()
    names = ('run_summary.json', 'independent_audit.json',
             'navigation_phase_metrics.json', 'docs20_supplementary.json',
             'person_audit.json', 'plate_audit.json', 'person_report.audio.json',
             'people_terminal.txt', 'ocr/ocr_terminal.txt', 'ocr_scene_materials.txt')
    for name in names:
        source = run / name
        target = folder / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for source in (run / 'persons/POINT_2.png',
                   run / 'ocr/POINT_8.annotated.png',
                   next(p for p in (run / 'POINT_7').glob('*.png')
                        if not p.name.endswith('.raw.png'))):
        target = folder / '图片' / source.name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    physical = json.loads((run / 'docs20_supplementary.json').read_text())['physical_home']
    readme = (f'# 解压源码独立 ROS Noetic 复现：{run.name}\n\n'
              '使用正式工程ZIP v6解压得到的四个catkin包，在现有Noetic环境新建隔离工作空间编译并启动；'
              '主机OCR复用了版本相符的Python3.12环境。Docker镜像和构建文件不在源码ZIP里。'
              '第一次试跑因旧仿真同时占用计算资源，POINT_1无合格同步帧而停止；'
              '清理旧仿真后，这一轮从同一ZIP新编号完成11个目标。\n\n'
              f"原独立审计：`{audit['acceptance_pass']}`；任务时长 {audit['bag_audit']['mission_duration_s']:.3f} 仿真秒；"
              f"普通白线接触 {audit['bag_audit']['ordinary_paint_contact_samples']}；"
              f"未授权接触 {len(audit['bag_audit']['unauthorized_conditional_contacts'])}；"
              f"人物 {summary['person_report']['counts']}；车牌3/3正确；"
              f"命令配对 {audit['bag_audit']['command_decision_pairing']['matched_commands']}/"
              f"{audit['bag_audit']['command_decision_pairing']['command_records']}。\n\n"
              f"物理车身起终点差：{physical['chassis_return_error_cm']:.3f}厘米、"
              f"{physical['yaw_return_error_rad']:.5f}弧度；这一项未达到额外设定的3厘米/0.04弧度目标。"
              '早期独立审计HOME使用AMCL，两个指标不可混同。新电脑的首次依赖安装未在本机证实，'
              '本次结论限定为已有Noetic与Paddle环境中从ZIP独立解压编译和执行。\n')
    write(folder / 'README.md', readme)
    return {'run_id': run.name, 'source_unpacked_zip_matches_final_runtime': True,
            'catkin_packages_built': 4, 'independent_audit_pass': True,
            'mission_sim_s': audit['bag_audit']['mission_duration_s'],
            'physical_home_error_cm': physical['chassis_return_error_cm'],
            'physical_home_yaw_rad': physical['yaw_return_error_rad'],
            'bag_retained_locally': True,
            'fresh_machine_dependency_install': 'not_tested_here',
            'audit_sha256': sha(run / 'independent_audit.json')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--team', default='智算三行队')
    parser.add_argument('--output', type=Path, default=ROOT / '复赛提交材料')
    parser.add_argument('--weights', type=Path, default=Path('/home/sz/下载/best.pt'))
    parser.add_argument('--repro-run', type=Path,
                        help='Completed audited run of an unpacked candidate ZIP')
    args = parser.parse_args()
    if not args.team or any(ch in args.team for ch in '/\\\0'):
        parser.error('队名包含非法路径字符')
    if not args.weights.is_file() or sha(args.weights) != YOLO_SHA:
        raise SystemExit('本地检测权重缺失或 SHA256 不匹配')
    paddle_model = ROOT / 'models/ocr/paddle/PP-OCRv5_mobile_rec_infer/inference.pdiparams'
    if not paddle_model.is_file():
        raise SystemExit('本地 PaddleOCR 模型缺失')
    commit = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
                                     text=True).strip()
    result = args.output / args.team
    stage = result.with_name(result.name + '.building')
    if result.exists() or stage.exists():
        raise SystemExit('输出目录已存在，拒绝覆盖；请指定新的 --output')
    stage.mkdir(parents=True)
    title = args.team + '-智慧社区复赛工程代码'
    workspace = stage / '01_工程代码' / title
    changes = build_workspace(workspace, args.weights, args.team, commit)
    files = {str(p.relative_to(workspace)): sha(p)
             for p in sorted(workspace.rglob('*')) if p.is_file()}
    manifest = {'team': args.team, 'source_git_commit': commit,
                'runtime_freeze_commit': 'd9a61b4', 'weights_sha256': YOLO_SHA,
                'ocr_model_sha256': sha(paddle_model),
                'portable_script_changes': changes,
                'notes': 'Host paths, container name, OCR worker import/path and relative map '
                         'provenance only; controller, safety limits and model bytes unchanged.',
                'files': files}
    write(workspace / 'PACKAGE_MANIFEST.json',
          json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    zip_path = stage / '01_工程代码' / (title + '.zip')
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in sorted(workspace.rglob('*')):
            if path.is_file():
                archive.write(path, path.relative_to(workspace.parent))
    size = zip_path.stat().st_size
    if size >= 150_000_000:
        raise SystemExit('工程 ZIP 超过150MB')
    with zipfile.ZipFile(zip_path) as archive:
        if archive.testzip() is not None:
            raise SystemExit('ZIP CRC 校验失败')
    metrics = json.loads((ROOT/'docs/agent_context/tasks/task009/docs20_20260929/results.json').read_text())
    evidence = copy_evidence(stage, args.team, metrics)
    reproduction = (copy_reproduction(evidence, args.repro_run, workspace)
                    if args.repro_run is not None else None)
    for directory in ('02_技术方案', '03_答辩展示', '04_综合展示视频', '06_规则与参考'):
        (stage / directory).mkdir()
    for source_name, target in (
        ('submission/技术方案草稿.md', '02_技术方案/技术方案整理稿.md'),
        ('submission/答辩展示提纲.md', '03_答辩展示/答辩提纲.md'),
        ('submission/视频录制脚本.md', '04_综合展示视频/录制脚本.md')):
        text = (ROOT / source_name).read_text(encoding='utf-8').replace('{{TEAM}}', args.team)
        write(stage / target, text)
    for name in ('第八届全球校园人工智能算法精英大赛算法应用赛道-智慧社区.pdf', '任务要求.txt'):
        shutil.copy2(ROOT / '任务需求' / name, stage / '06_规则与参考' / name)
    readme = (ROOT / 'submission/README.md').read_text(encoding='utf-8')
    write(stage / 'README.md', readme.replace('{{TEAM}}', args.team).replace('{{SOURCE_COMMIT}}', commit))
    payload = {'team': args.team, 'source_git_commit': commit,
               'engineering_zip': str(Path('01_工程代码') / zip_path.name),
               'engineering_zip_bytes': size, 'engineering_zip_sha256': sha(zip_path),
               'engineering_zip_under_150MB': True,
               'engineering_status': ('passed_unpacked_noetic_reproduction'
                                      if reproduction else 'packaged_unverified_runtime'),
               'technical_pdf': 'not_created', 'presentation_pdf': 'not_created',
               'comprehensive_mp4': 'not_created',
               'local_paddle_model_sha256': sha(paddle_model),
               'validation': {'zip_crc': True, 'docker_files_in_zip': False,
                              'unpacked_reproduction': reproduction or 'not_run'}}
    write(stage / 'inventory.json', json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    result.parent.mkdir(parents=True, exist_ok=True)
    stage.rename(result)
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
