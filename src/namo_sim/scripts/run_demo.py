#!/usr/bin/env python3
"""Isolated, finite demo supervisor. Relaunching is the repeatable full reset."""
import argparse
import datetime
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', choices=['A', 'B'], default='A')
    p.add_argument('--baseline', action='store_true')
    p.add_argument('--no-obstacle', action='store_true')
    p.add_argument('--gui', action='store_true')
    p.add_argument('--rviz', action='store_true')
    p.add_argument('--record', action='store_true')
    p.add_argument('--obstacle-x', default='2.4')
    p.add_argument('--obstacle-y', default='0.0')
    p.add_argument('--timeout', type=float, default=420)
    p.add_argument('--label', default='')
    p.add_argument('--stage', choices=['detect','plan','full'], default='full')
    p.add_argument('--immovable', action='store_true')
    p.add_argument('--remove-model', default='movable_obstacle', help='Executor handle; change only for failure injection')
    p.add_argument('--mock-seconds', type=float, default=None, help='Override BOTH cost and actual mock wait')
    p.add_argument('--domain', default='42')
    p.add_argument('--port', default='11355')
    args = p.parse_args()
    if args.record and not (args.gui and args.rviz):
        p.error('--record requires --gui --rviz to record the demo windows only')
    root = Path(__file__).resolve().parents[2]
    run_id = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')
    run_id += '_' + (args.label or ('baseline' if args.baseline else args.scenario))
    out = root / '.namo' / 'runs' / run_id
    out.mkdir(parents=True)
    env = dict(os.environ, ROS_DOMAIN_ID=args.domain, ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(out/'ros'), GAZEBO_MASTER_URI='http://127.0.0.1:'+args.port,
               NAMO_RUN_DIR=str(out), NAMO_RUN_ID=run_id)
    params=yaml.safe_load((root/'namo_sim/config/mission.yaml').read_text())
    params['mission']['ros__parameters'].update(scenario=args.scenario,stage=args.stage,removable=not args.immovable)
    params['mock_remover']['ros__parameters']['model_name']=args.remove_model
    if args.mock_seconds is not None:
        if args.mock_seconds<0:
            p.error('--mock-seconds must be nonnegative')
        params['mission']['ros__parameters']['mock_remove_seconds']=args.mock_seconds
        params['mock_remover']['ros__parameters']['mock_remove_seconds']=args.mock_seconds
    run_params=out/'parameters.yaml'
    run_params.write_text(yaml.safe_dump(params,sort_keys=False))
    setup = root/'.namo/install/setup.bash'
    if not setup.exists():
        sys.exit('Build first; see README.md')
    # Parameters go through argv, not shell interpolation.
    def launch(command, logfile):
        handle = open(out/logfile, 'w')
        proc = subprocess.Popen(['bash', '-c', 'source "$1"; shift; exec "$@"', 'namo', str(setup), *command],
                                env=env, cwd=root, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True)
        handle.close()
        return proc
    processes=[]
    print('RUN_DIR='+str(out), flush=True)
    try:
        processes.append(launch(['ros2', 'launch', 'namo_sim', 'sim.launch.py',
            'scenario:='+args.scenario, 'obstacle:='+str(not(args.baseline or args.no_obstacle)).lower(),
            'gui:='+str(args.gui).lower(), 'rviz:='+str(args.rviz).lower(),
            'mission:=false', 'obstacle_x:='+args.obstacle_x, 'obstacle_y:='+args.obstacle_y], 'launch.log'))
        processes.append(launch(['python3',str(root/'namo_sim/scripts/inspect_runtime.py'),str(out/'runtime.json')], 'runtime.log'))
        if args.record:
            # Tile only windows belonging to this launch; do not record unrelated desktop.
            gui_deadline=time.monotonic()+35
            gp=rp=None
            while time.monotonic()<gui_deadline:
                log=(out/'launch.log').read_text(errors='replace')
                gp=re.search(r'\[gzclient-\d+\]: process started with pid \[(\d+)\]',log)
                rp=re.search(r'\[rviz2-\d+\]: process started with pid \[(\d+)\]',log)
                if gp and rp: break
                time.sleep(.2)
            if not (gp and rp): raise RuntimeError('GUI process IDs unavailable; recording not started')
            arranged=subprocess.run(['python3',str(root/'namo_sim/scripts/arrange_windows.py'),
                '--gazebo-pid',gp[1],'--rviz-pid',rp[1]],env=env,capture_output=True,text=True,timeout=40)
            (out/'window_layout.log').write_text(arranged.stdout+arranged.stderr)
            if arranged.returncode: raise RuntimeError('Cannot safely arrange GUI; inspect window_layout.log')
            rect=json.loads(arranged.stdout)
            size=f"{rect['width']}x{rect['height']}"
            offset=f"+{rect.get('capture_x',rect.get('x',0))},{rect.get('capture_y',rect.get('y',0))}"
            processes.append(launch(['ffmpeg','-y','-video_size',size,'-framerate','20',
                '-f','x11grab','-i',env.get('DISPLAY', ':0')+offset,'-c:v','libx264',
                '-preset','ultrafast','-pix_fmt','yuv420p',str(out/'screen.mp4')], 'record.log'))
        if not args.baseline:
            processes.append(launch(['ros2','run','namo_sim','mock_remover','--ros-args','--params-file',
                str(run_params),'-p','use_sim_time:=true'], 'remover.log'))
        command=['ros2','run','namo_sim','baseline' if args.baseline else 'mission','--ros-args','-p','use_sim_time:=true']
        if args.baseline:
            command += ['-p','output:='+str(out/'baseline.json')]
        else:
            command += ['--params-file',str(run_params)]
        task = launch(command,'mission.log')
        processes.append(task)
        deadline=time.monotonic()+args.timeout
        while task.poll() is None and time.monotonic()<deadline:
            if processes[0].poll() is not None:
                raise RuntimeError('Simulation launch exited; inspect launch.log')
            time.sleep(0.5)
        if task.poll() is None:
            raise TimeoutError('Supervisor timeout')
        result=task.returncode
        (out/'supervisor.json').write_text(json.dumps(dict(exit_code=result,run_id=run_id,args=vars(args)),indent=2))
        print('EXIT_CODE='+str(result),flush=True)
        return result
    finally:
        # Only our process groups are signaled; no global pkill or unrelated Gazebo reset.
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
        end=time.monotonic()+12
        while any(proc.poll() is None for proc in processes) and time.monotonic()<end:
            time.sleep(0.2)
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
        for proc in processes:
            try: proc.wait(timeout=4)
            except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL)


if __name__=='__main__':
    sys.exit(main())
