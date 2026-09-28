#!/usr/bin/env python3
"""Create an additive competition handoff; keep all original workspaces intact."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile


ROOT = Path(__file__).resolve().parents[1]
BEST_SHA = 'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'
ACTIVE_SCRIPTS = {
    'start_sim.sh', 'randomize_car_plates.py', 'traffic_light_controller.py',
    'start_navigation.sh', 'start_standee_photo_route.sh', 'route_executor.py',
    'runtime_control.py', 'cmd_vel_watchdog.py', 'wheel_encoder_odom.py',
    'goal_sanitizer.py', 'navigation_goal_safety.py', 'check_foundation.py',
    'check_navigation_readiness.py', 'yolo_inspector.py', 'person_reporting.py',
    'announce_people.py', 'render_person_report.py', 'audit_person_report.py',
    'audit_standee_photo_run.py', 'audit_white_line_constraints.py', 'audit_calibrated_run.py',
    'start_slam_mapping.sh', 'start_autonomous_mapping.sh', 'model_state_odom.py',
    'survey_mapper.py', 'view_autonomous_mapping.sh', 'teleop.sh', 'car_teleop.py',
    'save_slam_map.sh', 'verify_slam_map.py', 'capture_tf_evidence.py',
    'coverage_report.py', 'build_white_line_navigation_map.py',
    'build_navigation_map.py', 'generate_ground_truth_map.py',
    'rescan_and_navigate.sh', 'competition.rviz', 'autonomous_mapping.rviz',
    'route_geometry.py', 'plate_ocr_trials.py',
}


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            digest.update(data)
    return digest.hexdigest()


def copy_tree(source, target):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns(
        '__pycache__', '*.pyc', '*.pyo', '.git', 'build', 'devel'), dirs_exist_ok=True)


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def fill(template, team):
    return (ROOT/'submission'/template).read_text(encoding='utf-8').replace('{{TEAM}}', team)


def portable_scripts(project):
    changes = []
    for path in (project/'scripts').glob('*.sh'):
        original = path.read_text()
        text = original
        lines = text.splitlines(keepends=True)
        lines.insert(1, 'source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/submission_env.sh"\n')
        text = ''.join(lines).replace('CONTAINER=ros1_modeling', 'CONTAINER="$INSPECTION_CONTAINER"')
        text = text.replace('ros1_modeling:', '${INSPECTION_CONTAINER}:')
        text = text.replace(' ros1_modeling ', ' "$INSPECTION_CONTAINER" ')
        text = text.replace('/home/sz/ros1_ws', '${INSPECTION_DATA_DIR}')
        text = text.replace('-v ${INSPECTION_DATA_DIR}:/root/ros1_ws', '-v "$INSPECTION_DATA_DIR:/root/ros1_ws"')
        text = text.replace('-v /home/sz/.gazebo:/root/.gazebo', '-v "$INSPECTION_GAZEBO_DIR:/root/.gazebo"')
        text = text.replace('DISPLAY=:1', 'DISPLAY="$DISPLAY"')
        text = text.replace('${YOLO_CHECKPOINT:-/home/sz/下载/best.pt}', '${YOLO_CHECKPOINT:-$ROOT_DIR/weights/best.pt}')
        text = text.replace('$ROOT_DIR/ros_packages/', '$ROOT_DIR/../')
        if path.name == 'start_navigation.sh':
            text = text.replace('expected=data.get("source_map_sha256")',
                'if source and not os.path.isabs(source):\n'
                '        source=os.path.normpath(os.path.join(os.path.dirname(sys.argv[1]),source))\n'
                '    expected=data.get("source_map_sha256")')
        if path.name == 'start_sim.sh':
            text = text.replace('osrf/ros:noetic-desktop-full tail', 'smart-community-submission:noetic tail')
        if text != original:
            write(path, text)
            changes.append(str(path.relative_to(project)))
        path.chmod(0o755)
    shutil.copy2(ROOT/'submission/submission_env.sh', project/'scripts/submission_env.sh')
    return changes


def save_pending(main, target):
    """Copy pending source files for traceability without checking out or resetting."""
    target.mkdir(parents=True, exist_ok=True)
    patch = subprocess.check_output(['git','-C',str(main),'diff','--binary'])
    (target/'tracked_changes.patch').write_bytes(patch)
    untracked = subprocess.check_output(
        ['git','-C',str(main),'ls-files','--others','--exclude-standard','-z']).decode().split('\0')
    copied, large = [], []
    for relative in untracked:
        if not relative or relative.startswith('复赛提交材料/'):
            continue
        path = main/relative
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        if path.stat().st_size > 5*1024*1024:
            large.append({'file':str(path),'size_bytes':path.stat().st_size,'sha256':sha(path)})
            continue
        output = target/'untracked'/relative
        output.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path, output)
        copied.append(relative)
    write(target/'inventory.json',json.dumps({'main_head':subprocess.check_output(
        ['git','-C',str(main),'rev-parse','HEAD'],text=True).strip(),
        'untracked_source_copied':copied,'large_files_preserved_at_original_paths':large},
        ensure_ascii=False,indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--team', default='智算三行队')
    parser.add_argument('--output', type=Path, default=Path('/home/sz/game/复赛提交材料'))
    parser.add_argument('--weights', type=Path, default=Path('/home/sz/下载/best.pt'))
    parser.add_argument('--vision-archive', type=Path, default=Path('/home/sz/下载/traffic_light_ros (2).zip'))
    args = parser.parse_args()
    if not args.team or any(c in args.team for c in '/\\\0'):
        raise SystemExit('队名不能包含路径分隔符')
    if sha(args.weights) != BEST_SHA:
        raise SystemExit('best.pt SHA 与已验收检测模型不一致')
    output = args.output/args.team
    if output.exists():
        raise SystemExit('输出目录已存在，保留原内容。请指定新的 --output。')
    output.mkdir(parents=True)
    names = ['01_工程代码','02_技术方案','03_答辩展示','04_综合展示视频','05_运行证据','06_历史与参考']
    folders = {name:output/name for name in names}
    for folder in folders.values():
        folder.mkdir()
    write(output/'README.md', fill('README.md',args.team))
    write(output/'提交检查清单.md',fill('提交检查清单.md',args.team))
    if (ROOT/'submission/OCR接入建议.md').is_file():
        write(output/'OCR接入建议.md',fill('OCR接入建议.md',args.team))
    write(folders[names[1]]/(args.team+'-智慧社区复赛技术方案.md'), fill('技术方案草稿.md',args.team))
    write(folders[names[2]]/(args.team+'-智慧社区复赛答辩展示.md'), fill('答辩展示提纲.md',args.team))
    write(folders[names[3]]/'视频录制脚本.md',fill('视频录制脚本.md',args.team))

    workspace = folders[names[0]]/(args.team+'-智慧社区复赛工程代码')
    project = workspace/'src/community_inspection'
    project.mkdir(parents=True)
    for directory in ['models','insert','navigation','assets','tests']:
        copy_tree(ROOT/directory,project/directory)
    (project/'scripts').mkdir()
    for name in sorted(ACTIVE_SCRIPTS):
        shutil.copy2(ROOT/'scripts'/name, project/'scripts'/name)
    (project/'worlds').mkdir()
    shutil.copy2(ROOT/'worlds/competition_classic_adjusted_20260924.world',project/'worlds')
    (project/'maps').mkdir()
    for path in (ROOT/'maps').iterdir():
        if path.is_file() and (path.name.startswith('current_slam_preview_white_lines.') or
                              path.name.startswith('competition_rescan_full_20260922.')):
            shutil.copy2(path,project/'maps'/path.name)
    for name in ['current_slam_preview.pgm','current_slam_preview_local.yaml']:
        shutil.copy2(ROOT/name, project/name)
    manifest_path = project/'maps/current_slam_preview_white_lines.manifest.json'
    original_manifest = json.loads(manifest_path.read_text())
    migrated = dict(original_manifest,source_map='../current_slam_preview.pgm',
                    source_yaml='../current_slam_preview_local.yaml',
                    submission_path_note='Source paths relocated; grid and source SHA unchanged.')
    write(manifest_path,json.dumps(migrated,ensure_ascii=False,indent=2)+'\n')
    write(project/'maps/原始导航图来源.json',json.dumps(original_manifest,ensure_ascii=False,indent=2)+'\n')
    write(project/'maps/README.md', '默认巡检图：current_slam_preview_white_lines.yaml。\n'
          '原始测量图在包根 current_slam_preview.pgm；保存 SHA。\n'
          'competition_rescan_full_20260922.* 为历史 SLAM 测量与来源材料；不替代新建图过程录像。\n')
    (project/'weights').mkdir()
    shutil.copy2(args.weights,project/'weights/best.pt')
    for name in ['package.xml','CMakeLists.txt','Dockerfile']:
        shutil.copy2(ROOT/'submission'/name,project/name)
    for package in ['forward_path_follower','safe_escape_recovery']:
        copy_tree(ROOT/'ros_packages'/package,workspace/'src'/package)
    with zipfile.ZipFile(args.vision_archive) as archive:
        for member in archive.infolist():
            path = Path(member.filename)
            if path.is_absolute() or '..' in path.parts:
                raise SystemExit('视觉压缩包中存在非法路径')
            if not path.parts or path.parts[0] != 'traffic_light_ros' or member.is_dir():
                continue
            destination = workspace/'src/tl_vision'/Path(*path.parts[1:])
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(archive.read(member))
    for path in (workspace/'src/tl_vision/scripts').iterdir():
        path.chmod(0o755)
    changes = portable_scripts(project)
    write(workspace/'README.md',fill('工程README.md',args.team))
    write(project/'README.md',fill('工程README.md',args.team))
    shutil.copy2(ROOT/'submission/setup_environment.sh',workspace/'setup_environment.sh')
    (workspace/'setup_environment.sh').chmod(0o755)
    for template, name in [('技术方案草稿.md','technical_solution.md'),
                           ('答辩展示提纲.md','presentation_outline.md')]:
        write(project/'docs'/name,fill(template,args.team))
    write(project/'scripts/README.md',
          '主要入口：start_sim.sh、start_slam_mapping.sh、view_autonomous_mapping.sh、teleop.sh、'
          'save_slam_map.sh、start_standee_photo_route.sh。\n'
          '其余为运行节点、地图与结果工具；OCR 试验脚本默认不启动。\n')

    reference = folders[names[5]]
    copy_tree(ROOT/'docs',reference/'历史开发文档')
    shutil.copy2(ROOT/'README.md',reference/'原工程README.md')
    copy_tree(ROOT/'任务需求',reference/'比赛规则原件')
    shutil.copy2(args.vision_archive,reference/args.vision_archive.name)
    for path in (ROOT/'scripts').iterdir():
        if path.is_file() and path.name not in ACTIVE_SCRIPTS:
            target = reference/'旧实验脚本'/path.name
            target.parent.mkdir(exist_ok=True)
            shutil.copy2(path,target)
    save_pending(Path('/home/sz/game'), reference/'原工作区未提交修改')
    video = Path('/home/sz/game/maps/cam_044058.mp4')
    if video.exists():
        target = folders[names[3]]/'视频素材'/video.name
        target.parent.mkdir()
        shutil.copy2(video,target)

    runs = Path('/home/sz/ros1_ws/photo_stops/standee_route_runs')
    recent = runs/'20260928_person_report_2'
    if recent.is_dir():
        for path in recent.rglob('*'):
            if path.is_file() and path.suffix in {'.json','.txt','.png','.html','.wav','.npy'}:
                destination = folders[names[4]]/'人物改进版'/recent.name/path.relative_to(recent)
                destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(path,destination)
    for run in ['20260928_135500','20260928_135919','20260928_140509']:
        for name in ['independent_audit.json','run_summary.json']:
            path = runs/run/name
            if path.is_file():
                target = folders[names[4]]/'旧导航冻结版_连续3次'/run/name
                target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(path,target)
    shutil.copy2(ROOT/'docs/agent_context/tasks/task008/frozen_manifest_20260927.json',
                 folders[names[4]]/'旧导航冻结版_冻结清单.json')
    write(folders[names[4]]/'README.md',
          '旧导航连续三次与人物改进版一次通过分别归档，不合并验收次数。\n'
          '多 GB motion.bag 未复制，原目录：'+str(runs)+'。\n'
          'JSON 保留原来源路径；报告 HTML 的图片/音频链接为相对路径，可本地打开。\n')

    package_manifest = {
        'team':args.team,'source_git_commit':subprocess.check_output(
            ['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        'backup_branch':'backup/submission-organize-20260928',
        'weights_sha256':BEST_SHA,'vision_archive_sha256':sha(args.vision_archive),
        'transformed_shell_scripts':changes,
        'transform_scope':'host paths, container name, packaged weights, plugin source layout, relative map provenance only; no navigation thresholds changed',
        'files':{str(path.relative_to(workspace)):sha(path) for path in sorted(workspace.rglob('*'))
                 if path.is_file()},
    }
    write(workspace/'PACKAGE_MANIFEST.json',json.dumps(package_manifest,ensure_ascii=False,indent=2)+'\n')
    zip_path = folders[names[0]]/(args.team+'-智慧社区复赛工程代码.zip')
    with zipfile.ZipFile(zip_path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for path in sorted(workspace.rglob('*')):
            if path.is_file():
                archive.write(path,path.relative_to(workspace.parent))
    if zip_path.stat().st_size >= 150_000_000:
        raise SystemExit('工程包超过150MB，整理结果保留，请减小后再提交')
    with zipfile.ZipFile(zip_path) as archive:
        if archive.testzip():
            raise SystemExit('工程ZIP完整性检查失败')
    inventory = {'team':args.team,'output':str(output),'source_git_commit':package_manifest['source_git_commit'],
                 'engineering_zip':str(zip_path),'engineering_zip_bytes':zip_path.stat().st_size,
                 'engineering_zip_sha256':sha(zip_path),'engineering_zip_under_150MB':True,
                 'formal_deliverables':{'engineering_zip':'prepared_for_reproduction',
                    'technical_pdf':'missing_markdown_draft_provided',
                    'presentation_pdf':'missing_outline_provided',
                    'comprehensive_mp4':'missing_reference_clip_and_script_provided'},
                 'ocr_status':'paused_not_integrated',
                 'new_environment_end_to_end_reproduction':'not_run'}
    write(output/'inventory.json',json.dumps(inventory,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(inventory,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
