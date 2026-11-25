#!/usr/bin/env python3
"""Fresh, isolated RGB-D -> physical pinch/lift/place validation."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--label',default='observed_arm')
    p.add_argument('--perception',choices=['yolo','sim_red'],default='sim_red')
    p.add_argument('--gui',action='store_true')
    p.add_argument('--timeout',type=float,default=180.)
    a=p.parse_args()
    root=Path(__file__).resolve().parents[2]
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out=root/'.namo_manip/arm_validation'/(stamp+'_'+a.label);out.mkdir(parents=True)
    setup=root/'.namo_manip/install/setup.bash'
    env=dict(os.environ,ROS_DOMAIN_ID='43',ROS_LOCALHOST_ONLY='1',GAZEBO_MASTER_URI='http://127.0.0.1:11356',
        ROS_LOG_DIR=str(out/'ros'),NAMO_RUN_DIR=str(out),PYTHONUNBUFFERED='1',
        NAMO_TF2_RUNTIME_LIB=str(root/'.namo_manip/runtime_fix/root/opt/ros/humble/lib'))
    config=yaml.safe_load((root/'namo_manipulation/config/perception.yaml').read_text())
    config['can_perception']['ros__parameters']['perception_mode']=a.perception
    config.update(yaml.safe_load((root/'namo_manipulation/config/arm.yaml').read_text()))
    config['physical_remover']['ros__parameters'].update(
        pick_topic='/namo/can_pose',place_frame='odom',
        place_xyz=[-.40,.14,.0675],require_collision_map=False)
    params=out/'parameters.yaml';params.write_text(yaml.safe_dump(config))
    processes=[]
    def start(args,name):
        with (out/name).open('w') as stream:
            child=subprocess.Popen(['bash','-c','source "$1"; shift; export LD_LIBRARY_PATH="$NAMO_TF2_RUNTIME_LIB:${LD_LIBRARY_PATH:-}"; exec "$@"',
                'namo',str(setup),*args],cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(child);return child
    def stop(sig):
        for child in reversed(processes):
            try:os.killpg(child.pid,sig)
            except ProcessLookupError:pass
    print('RUN_DIR='+str(out),flush=True)
    print('PERCEPTION='+a.perception.upper()+'; fixture disabled; Gazebo truth only in validation probe',flush=True)
    try:
        start(['ros2','launch','namo_manipulation','model.launch.py','spawn_can:=true',
            'can_x:=0.0','can_y:=0.34','gui:='+str(a.gui).lower()], 'model.log')
        start(['ros2','run','namo_manipulation','physical_remover','--ros-args','--params-file',str(params)],'arm.log')
        start(['ros2','run','namo_manipulation','can_perception','--ros-args','--params-file',str(params)],'perception.log')
        probe=start(['python3',str(root/'namo_manipulation/scripts/arm_probe.py'),'--operation','remove',
            '--prepare-first','--require-observation','--expected-place','-.40','.14','.0675',
            '--output',str(out),'--timeout',str(a.timeout)],'probe.log')
        deadline=time.monotonic()+a.timeout+45.
        while probe.poll() is None and time.monotonic()<deadline:
            for dependency in processes[:-1]:
                if dependency.poll() is not None:raise RuntimeError('Arm validation dependency exited; inspect '+str(out))
            time.sleep(.2)
        if probe.poll() is None:raise TimeoutError('Arm validation timeout')
        path=out/'remove_validation.json'
        if path.exists():
            d=json.loads(path.read_text())
            print(json.dumps({k:d.get(k) for k in ['executor_completed','physical_success','lift_above_initial','final_xy_error','final_z_error']},indent=2),flush=True)
            print('ARM_RESULT='+str(d.get('executor_status',{}).get('reason','')),flush=True)
        else:print((out/'probe.log').read_text()[-2000:],flush=True)
        return probe.returncode
    finally:
        stop(signal.SIGINT)
        deadline=time.monotonic()+8.
        while any(c.poll() is None for c in processes) and time.monotonic()<deadline:time.sleep(.2)
        stop(signal.SIGTERM)
        for child in processes:
            try:child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass


if __name__=='__main__':raise SystemExit(main())
