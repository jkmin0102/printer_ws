#!/usr/bin/env python3
"""Finite, isolated physical NAMO demo. All owned process groups are cleaned up."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import yaml
import xml.etree.ElementTree as ET


def process_identity(pid):
    """Parent, process group, birth tick; PID reuse must never target another run."""
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return int(fields[1]),int(fields[2]),int(fields[19])
    except (OSError,ValueError,IndexError):return None


def remember_descendants(roots,owned):
    snapshot={int(p.name):identity for p in Path('/proc').iterdir()
              if p.name.isdigit() and (identity:=process_identity(int(p.name))) is not None}
    parents=set(roots)|{pid for pid,old in owned.items()
                       if pid in snapshot and snapshot[pid][2]==old[2]}
    visited=set(parents)
    # A ROS launch child may create its own session, outside the parent's PGID.
    while parents:
        children={pid for pid,identity in snapshot.items() if identity[0] in parents}
        children-=visited
        for pid in children:owned[pid]=snapshot[pid]
        visited.update(children)
        parents=children


def signal_descendants(owned,sig):
    for pid,identity in owned.items():
        current=process_identity(pid)
        if current is not None and current[2]==identity[2]:
            try:os.kill(pid,sig)
            except ProcessLookupError:pass


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scenario',choices=['A','B'],default='A')
    p.add_argument('--stage',choices=['detect','plan','full'],default='full')
    p.add_argument('--perception',choices=['yolo','sim_red'],default='yolo')
    p.add_argument('--no-obstacle',action='store_true')
    p.add_argument('--can-x',type=float,default=0.)
    p.add_argument('--can-y',type=float,default=1.1)
    p.add_argument('--immovable',action='store_true')
    p.add_argument('--gui',action='store_true');p.add_argument('--rviz',action='store_true')
    output=p.add_mutually_exclusive_group()
    output.add_argument('--record',action='store_true')
    output.add_argument('--no-save',action='store_true',
                        help='No recording or retained run artifacts; remove temporary logs/configs on exit')
    p.add_argument('--camera',choices=['overview','closeup'],default='overview')
    p.add_argument('--timeout',type=float,default=600.)
    p.add_argument('--label',default='');p.add_argument('--domain',default='44');p.add_argument('--port',default='11357')
    a=p.parse_args()
    if a.record and not(a.gui and a.rviz):p.error('--record requires --gui --rviz')
    root=Path(__file__).resolve().parents[2]
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')
    run_id=stamp+'_'+(a.label or a.scenario)
    if a.no_save:
        with tempfile.TemporaryDirectory(prefix='namo_run_') as temporary:
            return run(a,root,run_id,Path(temporary))
    out=root/'.namo_manip/runs'/run_id;out.mkdir(parents=True)
    return run(a,root,run_id,out)


def run(a,root,run_id,out):
    env=dict(os.environ,ROS_DOMAIN_ID=a.domain,ROS_LOCALHOST_ONLY='1',GAZEBO_MASTER_URI='http://127.0.0.1:'+a.port,
             ROS_LOG_DIR=str(out/'ros'),NAMO_RUN_DIR=str(out),NAMO_RUN_ID=run_id,PYTHONUNBUFFERED='1')
    setup=root/'.namo_manip/install/setup.bash'
    if not setup.is_file():raise RuntimeError('Build first; see namo_manipulation/README.md')
    runtime=root/'.namo_manip/runtime_fix'
    if not (runtime/'manifest.json').is_file():
        raise RuntimeError('Prepare local TF fix first: python3 namo_manipulation/tools/prepare_tf2_runtime.py')
    runtime_info=json.loads((runtime/'manifest.json').read_text())
    runtime_lib=runtime/'root/opt/ros/humble/lib'
    for name,expected in runtime_info['libraries'].items():
        library=runtime_lib/name
        if not library.is_file() or hashlib.sha256(library.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Local TF runtime checksum mismatch; rerun tools/prepare_tf2_runtime.py')
    env['NAMO_TF2_RUNTIME_LIB']=str(runtime_lib)
    (out/'runtime_fix.json').write_text(json.dumps(runtime_info,indent=2))
    params=yaml.safe_load((root/'namo_manipulation/config/mission.yaml').read_text())
    params['mission']['ros__parameters'].update(scenario=a.scenario,stage=a.stage,perception_mode=a.perception,removable=not a.immovable)
    params.update(yaml.safe_load((root/'namo_manipulation/config/perception.yaml').read_text()))
    params['can_perception']['ros__parameters']['perception_mode']=a.perception
    if a.no_save:
        params['can_perception']['ros__parameters']['evidence_dir']=''
    arm_config=root/'namo_manipulation/config/arm.yaml'
    if arm_config.is_file():params.update(yaml.safe_load(arm_config.read_text()))
    params_path=out/'parameters.yaml';params_path.write_text(yaml.safe_dump(params,sort_keys=False))
    processes=[];launch_parents=set();owned_children={}
    def signal_group(proc, sig):
        # A launch parent may exit before its children after a startup error.
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:pass
    def launch(argv,log):
        with open(out/log,'w') as f:
            proc=subprocess.Popen(['bash','-c','source "$1"; shift; export LD_LIBRARY_PATH="$NAMO_TF2_RUNTIME_LIB:${LD_LIBRARY_PATH:-}"; exec "$@"','namo',str(setup),*argv],
                cwd=root,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc)
        if argv[:2]==['ros2','launch']:launch_parents.add(proc.pid)
        return proc
    print(('TEMP_RUN_DIR=' if a.no_save else 'RUN_DIR=')+str(out),flush=True)
    if a.no_save:
        print('NO_SAVE: 녹화·실행 기록을 보존하지 않습니다. 임시 파일은 종료 시 삭제됩니다.',flush=True)
    if a.perception=='sim_red':print('PERCEPTION=SIM_RED: explicit colour/depth cylinder experiment; not YOLO recognition',flush=True)
    try:
        source_world=root/'namo_manipulation/worlds'/f'{a.scenario}.world'
        scene=ET.fromstring(source_world.read_text())
        if a.camera=='closeup':
            scene.find('.//gui/camera/pose').text='0.75 0.15 1.65 0 0.98 2.35619'
        world=out/'world.sdf';world.write_text(ET.tostring(scene,encoding='unicode'))
        for source,name in [(root/'namo_manipulation/maps'/f'{a.scenario}_geometry.json','geometry.json'),
                            (root/'namo_manipulation/maps'/f'{a.scenario}.pgm','map.pgm'),
                            (root/'namo_manipulation/config/nav2.yaml','nav2.yaml'),
                            (root/'namo_manipulation/config/navigate.xml','navigate.xml'),
                            (root/'namo_manipulation/models/robot_sources.json','robot_sources.json'),
                            (root/'namo_manipulation/models/robot.urdf','robot.urdf')]:
            shutil.copyfile(source,out/name)
        rviz=yaml.safe_load((root/'namo_manipulation/rviz/manipulation.rviz').read_text())
        walls=json.loads((out/'geometry.json').read_text())['wall_rectangles']
        left=min(b[0] for b in walls);right=max(b[2] for b in walls)
        view=rviz['Visualization Manager']['Views']['Current']
        view.update(X=(left+right)/2,Scale=min(90.,380./(right-left)))
        rviz_path=out/'manipulation.rviz';rviz_path.write_text(yaml.safe_dump(rviz,sort_keys=False,allow_unicode=True))
        saved_map=yaml.safe_load((root/'namo_manipulation/maps'/f'{a.scenario}.yaml').read_text())
        saved_map['image']='map.pgm';(out/'map.yaml').write_text(yaml.safe_dump(saved_map))
        model=launch(['ros2','launch','namo_manipulation','model.launch.py','world_path:='+str(world),
            'rviz_config:='+str(rviz_path),'robot_x:=-1.1','robot_y:=0.0','robot_yaw:=0.0','spawn_can:='+str(not a.no_obstacle).lower(),
            'can_x:='+str(a.can_x),'can_y:='+str(a.can_y),'gui:='+str(a.gui).lower(),'rviz:='+str(a.rviz).lower()], 'model.log')
        launch(['ros2','launch','namo_manipulation','navigation.launch.py','scenario:='+a.scenario],'navigation.log')
        if not a.no_save:
            launch(['python3',str(root/'namo_manipulation/scripts/inspect_navigation.py'),str(out/'navigation_probe.jsonl')],'probe.log')
        launch(['ros2','run','namo_manipulation','physical_remover','--ros-args','--params-file',str(params_path),'-p','use_sim_time:=true'],'arm.log')
        launch(['ros2','run','namo_manipulation','can_perception','--ros-args','--params-file',str(params_path)],'perception.log')
        launch(['ros2','run','namo_manipulation','status_display'],'status.log')
        if a.gui and a.rviz:
            deadline=time.monotonic()+45.;gp=rp=None
            while time.monotonic()<deadline:
                log=(out/'model.log').read_text(errors='replace')
                gp=re.search(r'\[gzclient-\d+\]: process started with pid \[(\d+)\]',log)
                rp=re.search(r'\[rviz2-\d+\]: process started with pid \[(\d+)\]',log)
                if gp and rp:break
                time.sleep(.2)
            if not(gp and rp):raise RuntimeError('GUI process IDs not ready')
            r=subprocess.run(['python3',str(root/'namo_manipulation/scripts/arrange_physical_windows.py'),'--gazebo-pid',gp[1],'--rviz-pid',rp[1],'--rviz-title','manipulation.rviz'],
                             env=env,capture_output=True,text=True,timeout=45.)
            (out/'window_layout.log').write_text(r.stdout+r.stderr)
            if r.returncode:raise RuntimeError('Cannot arrange only the demo windows')
            rect=json.loads(r.stdout)
        if a.record:
            launch(['ffmpeg','-y','-video_size',f"{rect['width']}x{rect['height']}",'-framerate','20','-f','x11grab',
                '-i',env.get('DISPLAY',':0')+f"+{rect.get('capture_x',rect.get('x',0))},{rect.get('capture_y',rect.get('y',0))}",
                '-c:v','libx264','-preset','ultrafast','-pix_fmt','yuv420p',str(out/'screen.mp4')],'record.log')
        task=launch(['ros2','run','namo_manipulation','mission','--ros-args','--params-file',str(params_path)],'mission.log')
        end=time.monotonic()+a.timeout;position=0
        while task.poll() is None and time.monotonic()<end:
            remember_descendants([p.pid for p in processes if p.poll() is None],owned_children)
            if model.poll() is not None:raise RuntimeError('Gazebo launch exited')
            for dependency in processes[:-1]:
                if dependency.poll() is not None and dependency.returncode!=0:
                    raise RuntimeError(f'Required child process exited with {dependency.returncode}; inspect run logs')
            with open(out/'status.log') as f:
                f.seek(position);chunk=f.read();position=f.tell()
            if chunk:print(chunk,end='',flush=True)
            time.sleep(.25)
        if task.poll() is None:raise TimeoutError('Mission supervisor timeout')
        (out/'supervisor.json').write_text(json.dumps(dict(run_id=run_id,args=vars(a),exit_code=task.returncode),indent=2))
        if task.returncode:
            print((out/'mission.log').read_text(errors='replace')[-2500:],flush=True)
            # --no-save removes the temporary folder. Surface the underlying
            # planner/controller messages before cleanup, not just status=6.
            diagnostics=[line for line in (out/'navigation.log').read_text(errors='replace').splitlines()
                         if any(token in line for token in ('[WARN]', '[ERROR]', 'Aborting', 'Failed', 'failed'))]
            print('NAV2_DIAGNOSTICS (최근 경고·오류):\n'+'\n'.join(diagnostics[-18:]),flush=True)
            print('PERCEPTION_DIAGNOSTICS (최근 인식 상태):\n'+
                  '\n'.join((out/'perception.log').read_text(errors='replace').splitlines()[-12:]),flush=True)
        print('EXIT_CODE='+str(task.returncode),flush=True)
        return task.returncode
    finally:
        remember_descendants([p.pid for p in processes if p.poll() is None],owned_children)
        for proc in reversed(processes):
            # ROS launch forwards SIGINT itself. Signalling its entire group
            # duplicates the signal and interrupts child cleanup callbacks.
            if proc.pid in launch_parents and proc.poll() is None:
                try:os.kill(proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
            else:signal_group(proc,signal.SIGINT)
        end=time.monotonic()+10.
        while any(p.poll() is None for p in processes) and time.monotonic()<end:time.sleep(.2)
        for proc in processes:
            signal_group(proc,signal.SIGTERM)
        signal_descendants(owned_children,signal.SIGTERM)
        for proc in processes:
            try:proc.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                signal_group(proc,signal.SIGKILL)
                proc.wait(timeout=3.)
        # Launch can exit while a lifecycle child is blocked in a service call.
        # Give independently grouped descendants a bounded graceful interval.
        end=time.monotonic()+3.
        while time.monotonic()<end:
            remaining=[pid for pid,old in owned_children.items()
                       if (current:=process_identity(pid)) is not None and current[2]==old[2]]
            if not remaining:break
            time.sleep(.1)
        signal_descendants(owned_children,signal.SIGKILL)

if __name__=='__main__':sys.exit(main())
