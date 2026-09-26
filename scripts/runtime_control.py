#!/usr/bin/env python3
"""Small in-container lifecycle helper for mutually exclusive ROS modes."""
import argparse, os, re, signal, subprocess, time
NODES=('slam_gmapping','map_server','final_map_server','amcl','move_base','cmd_vel_watchdog','goal_sanitizer','explore','autonomous_mapper','waypoint_mapper','patrol_controller','route_executor','car_teleop','inner_loop_navigator','visual_inspector','model_state_odom','wheel_encoder_odom','map_to_odom_anchor','sim_tf_relay','amcl_tf_relay','chassis_to_lidar','chassis_to_lidar_slam','chassis_to_lidar_mapping','chassis_to_camera','camera_to_optical','map_to_odom_sim')
LAUNCH_RE=r'[r]oslaunch /root/navigation/(navigation|mapping_exploration|planner|robot)\.launch'
STALE_NAME_RE=re.compile(r'^/(?:rviz_|tf_echo_|tf_monitor_|rostopic_|cmd_vel_throttle_)')
def stop():
    # Persistent rosmasters retain names of dead Docker processes.  Purge
    # those registrations first; passing them to rosnode kill can block long
    # enough for Gazebo's diff-drive plugin to keep the last non-zero command.
    try:
        cleanup = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        cleanup.communicate(b'y\n', timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        try:
            cleanup.kill()
        except Exception:
            pass
    nodes=subprocess.run(['rosnode','list'],text=True,capture_output=True,check=False).stdout.splitlines()
    active=set(n.strip() for n in nodes)
    targets=['/'+n for n in NODES if '/'+n in active]
    targets.extend(n for n in ('/direct_route_driver', '/calibrated_navigation',
                               '/world_map_frame') if n in active)
    # Earlier route_executor releases used anonymous ROS names.  Match that
    # legacy prefix too, otherwise an old client retains its move_base goal
    # across a supposedly clean navigation restart.
    targets.extend(n for n in active if n.startswith('/route_executor_'))
    if targets:
        try:
            subprocess.run(['rosnode','kill']+targets,stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,check=False,timeout=15)
        except subprocess.TimeoutExpired:
            pass
    nodes=subprocess.run(['rosnode','list'],text=True,capture_output=True,check=False).stdout.splitlines()
    stale=[n.strip() for n in nodes if STALE_NAME_RE.match(n.strip())]
    if stale:
        try:
            subprocess.run(['rosnode','kill']+stale,stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,check=False,timeout=15)
        except subprocess.TimeoutExpired:
            pass
    subprocess.run(['pkill','-TERM','-x','rviz'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
    out=subprocess.run(['pgrep','-af',LAUNCH_RE],text=True,capture_output=True,check=False).stdout
    for line in out.splitlines():
        try: os.kill(int(line.split(None,1)[0]),signal.SIGTERM)
        except (ValueError,IndexError,ProcessLookupError): pass
    subprocess.run(['pkill','-TERM','-f',r'^python3 /root/(model_state_odom|autonomous_mapper|waypoint_mapper|patrol_controller|route_executor|inner_loop_navigator|visual_inspector|survey_mapper)\.py( |$)'],check=False)
    subprocess.run(['pkill','-TERM','-f',r'cmd_vel_throttle'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
    # Action goals survive a client crash unless explicitly cancelled; clear
    # them before a new mutually-exclusive mode starts.
    subprocess.run(['timeout','5','rostopic','pub','-1','/move_base/cancel',
                    'actionlib_msgs/GoalID','{}'],stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL,check=False)
    subprocess.run(['timeout','5','rostopic','pub','-1','/my_car/cmd_vel',
                    'geometry_msgs/Twist','{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                   stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
    time.sleep(1)
def reset():
    stop()
    r=subprocess.run(['rosservice','call','/gazebo/set_model_state',"{model_state: {model_name: my_car, pose: {position: {x: 1.714860, y: -1.599947, z: 0.035}, orientation: {x: 0.0, y: 0.0, z: 0.719390, w: 0.694607}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}"],text=True,capture_output=True,check=False)
    if r.returncode or 'success: True' not in r.stdout: raise SystemExit('Gazebo fixed birth reset failed: '+r.stdout.strip())
p=argparse.ArgumentParser();p.add_argument('action',choices=('stop','reset'));a=p.parse_args();reset() if a.action=='reset' else stop()
