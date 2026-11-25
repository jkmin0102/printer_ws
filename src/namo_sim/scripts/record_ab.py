#!/usr/bin/env python3
"""Record verified A then B, preserving original speed and mock wait time."""
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[2]
records=[]
for scenario in ('A','B'):
    label='final_'+scenario
    result=subprocess.run([sys.executable,str(root/'namo_sim/scripts/run_demo.py'),
        '--scenario',scenario,'--gui','--rviz','--record','--label',label],cwd=root)
    if result.returncode:
        raise SystemExit(result.returncode)
    paths=sorted((root/'.namo/runs').glob('*_'+label))
    record=paths[-1]
    data=json.loads((record/'result.json').read_text())
    if not data['success'] or not data['execution_completed']:
        raise SystemExit('Recording did not complete the mission')
    records.append(record)
subprocess.run([sys.executable,str(root/'namo_sim/scripts/summarize_runs.py')],check=True,cwd=root)
listing=root/'.namo/video_concat.txt'
listing.write_text(''.join("file '"+str(p/'screen.mp4').replace("'", "'\\''")+"'\n" for p in records))
subprocess.run(['ffmpeg','-y','-loglevel','error','-f','concat','-safe','0','-i',str(listing),
                '-c','copy','-movflags','+faststart',str(root/'.namo/demo_ab.mp4')],check=True)
(root/'.namo/video_manifest.json').write_text(json.dumps({
    'runs':[str(p) for p in records], 'output':str(root/'.namo/demo_ab.mp4'),
    'speed':1.0,'edited_wait':False},indent=2))
print('VIDEO='+str(root/'.namo/demo_ab.mp4'),flush=True)
