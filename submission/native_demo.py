#!/usr/bin/env python3
"""Run the packaged ROS Noetic workspace directly, without Docker commands."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT.parents[1]
TOPICS = (
    '/clock /tf /tf_static /odom /my_car/wheel_odom /my_car/cmd_vel_nav /my_car/cmd_vel '
    '/scan /camera/depth/points /camera/depth/image_raw /camera/depth/camera_info /camera/camera_info '
    '/camera/image_raw /inspection/image /amcl_pose /move_base/goal /move_base/status /move_base/feedback '
    '/move_base/NavfnROS/plan /move_base/ForwardPathFollower/local_plan '
    '/move_base/ForwardPathFollower/progress /move_base/ForwardPathFollower/status '
    '/move_base/result /move_base/global_costmap/costmap /move_base/local_costmap/costmap '
    '/move_base/global_costmap/footprint /move_base/local_costmap/footprint '
    '/route/status /inspection/detections /inspection/traffic_light /route/progress '
    '/inspection/person_report /inspection/person_image /inspection/plate_capture '
    '/inspection/plate_text /inspection/plate_image /inspection/plate_report '
    '/inspection/judge_yolo_image /inspection/judge_person_image /inspection/judge_ocr_image '
    '/inspection/judge_event /inspection/judge_display_image '
    '/inspection/traffic_light_confidence /inspection/yolo/metrics '
    '/traffic_light/state /traffic_light/time_remaining /traffic_light/gate_status '
    '/traffic_light/braking /traffic_light/command_decision /gazebo/link_states'
).split()


class Processes:
    def __init__(self, directory, env):
        self.directory, self.env, self.children = directory, env, []

    def start(self, name, command):
        stream = (self.directory / (name + '.log')).open('w')
        child = subprocess.Popen(command, cwd=str(PROJECT), env=self.env,
                                 stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        stream.close()
        self.children.append((name, child))
        return child

    def stop(self, name):
        for label, child in self.children:
            if label == name and child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
                return

    def stop_all(self):
        for name, child in reversed(self.children):
            if child.poll() is None:
                self.stop(name)


def command(args, env, timeout=5):
    return subprocess.run(args, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=timeout, text=True)


def wait_until(check, description, timeout=90):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return
        time.sleep(.5)
    raise RuntimeError('Timeout waiting for ' + description)


def prepare_assets(run, env):
    assets = run / 'assets'
    assets.mkdir()
    for name, source in (
        ('person_standees', PROJECT / 'insert/person_standees'),
        ('car_standees', PROJECT / 'insert/car_standees'),
        ('traffic_light', PROJECT / 'models/traffic_light')):
        shutil.copytree(source, assets / name)
    previous = WORKSPACE / 'runtime_data/last_plate_selection.json'
    generated = assets / 'selected_plates'
    args = [sys.executable, str(PROJECT / 'scripts/randomize_car_plates.py'),
            str(PROJECT / 'insert/car_standees/plate_inventory'), str(generated)]
    if previous.exists():
        args.extend(['--previous', str(previous)])
    subprocess.run(args, env=env, check=True)
    shutil.copytree(generated / 'materials', assets / 'car_standees/materials',
                    dirs_exist_ok=True)
    shutil.copy2(generated / 'plate_selection.json', previous)
    world = (PROJECT / 'worlds/competition_classic_adjusted_20260924.world').read_text()
    template = (assets / 'traffic_light/model.sdf').read_text()
    for name in ('person_standees', 'car_standees', 'traffic_light'):
        world = world.replace('file:///root/' + name,
                              'file://' + str(assets / name))
    template = template.replace('file:///root/traffic_light',
                                'file://' + str(assets / 'traffic_light'))
    (assets / 'traffic_light/model.sdf').write_text(template)
    world_path = assets / 'competition_native.world'
    world_path.write_text(world)
    config = assets / 'navigation'
    config.mkdir()
    for filename in ('robot.launch', 'planner.launch', 'navigation.launch'):
        source = PROJECT / 'navigation' / filename
        text = source.read_text().replace('/root/navigation/',
                                          str(PROJECT / 'navigation') + '/')
        (config / filename).write_text(text)
    return assets, world_path, config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_id')
    parser.add_argument('--port', type=int, default=11311)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--external-ocr-worker', action='store_true')
    parser.add_argument('--external-audio', action='store_true')
    args = parser.parse_args()
    if not args.run_id.replace('_', '').replace('-', '').isalnum():
        parser.error('RUN_ID may contain only letters, digits, underscores and hyphens')
    if not (WORKSPACE / 'devel/setup.bash').exists():
        parser.error('Run catkin_make in the package root first')
    data = WORKSPACE / 'runtime_data'
    data.mkdir(exist_ok=True)
    run = data / args.run_id
    if run.exists():
        parser.error('RUN_ID already exists; existing evidence is protected')
    run.mkdir()
    for name in ('ocr', 'judge', 'audit_inputs'):
        (run / name).mkdir()
    external_uid = os.environ.get('OCR_WORKER_UID')
    if external_uid:
        uid, gid = int(external_uid), int(os.environ.get('OCR_WORKER_GID', external_uid))
        for path in (run, run / 'ocr'):
            os.chown(str(path), uid, gid)
    env = dict(os.environ)
    env['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(args.port)
    env['ROS_HOSTNAME'] = '127.0.0.1'
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:' + str(args.port + 106)
    env['MPLBACKEND'] = 'Agg'
    env['GAZEBO_MODEL_PATH'] = str(PROJECT / 'models') + ':' + env.get('GAZEBO_MODEL_PATH', '')
    env['CAR_STANDEE_DIR'] = str(run / 'assets/car_standees')
    env['COMPETITION_WORLD_PATH'] = str(run / 'assets/competition_native.world')
    process = Processes(run, env)
    status = 'STARTING'
    try:
        if command(['rosparam', 'list'], env).returncode == 0:
            raise RuntimeError('ROS master already uses this port; choose another --port')
        process.start('roscore', ['roscore', '-p', str(args.port)])
        wait_until(lambda: command(['rosparam', 'list'], env).returncode == 0,
                   'ROS master', 30)
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'],
                       check=True, env=env)
        assets, world, navigation = prepare_assets(run, env)
        server = ['rosrun', 'gazebo_ros',
                  'gzserver' if args.headless else 'gazebo',
                  '--verbose', str(world)]
        process.start('gazebo', server)
        wait_until(lambda: command(['rosservice', 'call',
                                     '/gazebo/get_world_properties'], env).returncode == 0,
                   'Gazebo world service', 70)
        process.start('traffic', [sys.executable,
            str(PROJECT / 'scripts/traffic_light_controller.py'),
            '_template_path:=' + str(assets / 'traffic_light/model.sdf')])
        wait_until(lambda: 'data: True' in command(['rostopic', 'echo', '-n1',
                                     '/traffic_light/ready'], env).stdout,
                   'traffic-light controller', 30)
        subprocess.run([sys.executable, str(PROJECT / 'scripts/record_plate_scene_truth.py'),
                        '--output', str(run / 'audit_inputs/plate_scene_truth.json')],
                       check=True, env=env)
        shutil.copy2(assets / 'car_standees/materials/scripts/car_standees.material',
                     run / 'ocr_scene_materials.txt')
        subprocess.run(['rosparam', 'set', '/cmd_vel_watchdog/enforce_white_lines', 'true'],
                       check=True, env=env)
        subprocess.run(['rosparam', 'set', '/cmd_vel_watchdog/enforce_traffic', 'true'],
                       check=True, env=env)
        subprocess.run(['rosparam', 'set', '/cmd_vel_watchdog/traffic_min_confidence', '.50'],
                       check=True, env=env)
        subprocess.run(['rosparam', 'set', '/cmd_vel_watchdog/expected_yolo_sha256',
                        'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'],
                       check=True, env=env)
        process.start('wheel_odom', [sys.executable,
            str(PROJECT / 'scripts/wheel_encoder_odom.py')])
        process.start('goal_sanitizer', [sys.executable,
            str(PROJECT / 'scripts/goal_sanitizer.py')])
        process.start('watchdog', [sys.executable,
            str(PROJECT / 'scripts/cmd_vel_watchdog.py'),
            '_ground_texture:=' + str(PROJECT / 'models/competition_ground/materials/textures/map.png')])
        process.start('navigation', ['roslaunch', str(navigation / 'navigation.launch'),
            'map_file:=' + str(PROJECT / 'maps/current_slam_preview_white_lines.yaml'),
            'initial_x:=1.714860', 'initial_y:=-1.599947', 'initial_yaw:=1.606236',
            'observation_sources:=laser depth', 'global_observation_sources:='])
        process.start('yolo', [sys.executable,
            str(PROJECT / 'scripts/yolo_inspector.py'),
            '_model:=' + str(PROJECT / 'weights/best.pt'),
            '_conf_threshold:=0.15', '_traffic_min_confidence:=0.50',
            '_process_hz:=5.5'])
        wait_until(lambda: all(command(['rosnode', 'ping', '-c', '1', '/' + name],
                                        env).returncode == 0
                               for name in ('map_server','amcl','move_base',
                                            'cmd_vel_watchdog','yolo_inspector')),
                   'navigation nodes', 90)
        process.start('plate_bridge', [sys.executable,
            str(PROJECT / 'scripts/plate_result_bridge.py'),
            '_photo_dir:=' + str(run),
            '_font:=' + str(PROJECT / 'assets/ocr_font.ttf'),
            '_latin_font:=' + str(PROJECT / 'assets/ocr_latin_font.ttf')])
        if not args.headless:
            process.start('judge_monitor', [sys.executable,
                str(PROJECT / 'scripts/judge_monitor.py'),
                '_photo_dir:=' + str(run)])
            process.start('rviz', ['rviz', '-d', str(PROJECT / 'scripts/competition_judge.rviz')])
            process.start('judge_image_view', ['rosrun', 'image_view', 'image_view',
                'image:=/inspection/judge_display_image',
                '_window_name:=JudgeVision', '_autosize:=false'])
        if args.external_ocr_worker:
            wait_until(lambda: (run / 'ocr/ready.json').exists(),
                       'external PaddleOCR worker', 240)
        else:
            ocr_python = os.environ.get('OCR_PYTHON', sys.executable)
            process.start('ocr_worker', [ocr_python,
                str(PROJECT / 'scripts/paddle_plate_worker.py'),
                '--run-dir', str(run), '--watch', '--compare-low',
                '--ready-file', str(run / 'ocr/ready.json')])
            wait_until(lambda: (run / 'ocr/ready.json').exists(),
                       'PaddleOCR worker', 240)
        process.start('rosbag', ['rosbag', 'record', '--lz4', '-O',
                                str(run / 'motion.bag')] + TOPICS)
        time.sleep(2)
        route = [sys.executable, '-u', str(PROJECT / 'scripts/route_executor.py'),
            '_start_waypoint:=0', '_park_only:=false', '_return_path:=auto',
            '_capture_photos:=true', '_strict_acceptance:=true', '_ocr_enabled:=true',
            '_ocr_worker_uid:=' + str(int(external_uid or os.getuid())),
            '_ocr_worker_gid:=' + str(int(os.environ.get('OCR_WORKER_GID',external_uid or os.getgid()))),
            '_photo_points_file:=' + str(PROJECT / 'navigation/standee_photo_route.json'),
            '_route_contract:=' + str(PROJECT / 'navigation/inner_route.yaml'),
            '_person_config:=' + str(PROJECT / 'navigation/person_reporting.json'),
            '_photo_waypoints:=POINT_1,POINT_2,POINT_3,POINT_4,POINT_5,POINT_6,POINT_7,POINT_8,POINT_9,POINT_10',
            '_photo_dir:=' + str(run), '_photo_position_tolerance:=0.03',
            '_photo_heading_tolerance:=0.04', '_photo_nav_xy_tolerance:=0.03',
            '_photo_nav_heading_tolerance:=0.04',
            '_point_3_photo_position_tolerance:=0.05',
            '_point_3_nav_xy_tolerance:=0.05',
            '_point_5_photo_position_tolerance:=0.025',
            '_point_5_photo_heading_tolerance:=0.025',
            '_point_5_nav_xy_tolerance:=0.025',
            '_point_5_nav_heading_tolerance:=0.025',
            '_point_7_photo_position_tolerance:=0.05',
            '_point_7_nav_xy_tolerance:=0.05',
            '_point_10_photo_position_tolerance:=0.05',
            '_point_10_nav_xy_tolerance:=0.05',
            '_no_progress_timeout:=8.0', '_photo_settle_seconds:=0.3',
            '_photo_burst_count:=4', '_photo_burst_interval:=0.15']
        route_process = process.start('route_executor', route)
        end = time.monotonic() + 900
        while time.monotonic() < end:
            summary = run / 'run_summary.json'
            if summary.exists():
                try:
                    status = json.loads(summary.read_text())['route_status']
                except (ValueError, KeyError):
                    status = 'RUNNING'
                if status in ('COMPLETE_PARKED', 'FAILED'):
                    break
            if route_process.poll() is not None:
                raise RuntimeError('route_executor ended before writing a final result; '
                                   'see runtime_data/<RUN_ID>/route_executor.log')
            time.sleep(.5)
        process.stop('rosbag')
        if status == 'COMPLETE_PARKED' and not args.external_audio:
            subprocess.run([sys.executable, str(PROJECT / 'scripts/announce_people.py'),
                '--report', str(run / 'person_report.json'),
                '--output', str(run / 'person_report.wav'), '--play'],
                check=True, env=env)
            (run / 'person_report.status.json').rename(run / 'person_report.audio.json')
        if status == 'COMPLETE_PARKED' and args.external_audio:
            wait_until(lambda: (run / 'person_report.audio.json').exists(),
                       'external person audio', 90)
        print(json.dumps({'run': str(run), 'status': status,
                          'bag_closed': (run / 'motion.bag').exists()}, ensure_ascii=False))
        return 0 if status == 'COMPLETE_PARKED' else 1
    finally:
        process.stop_all()


if __name__ == '__main__':
    sys.exit(main())
