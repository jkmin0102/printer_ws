"""ROS launch children can have independent process groups and outlive launch."""
import importlib.util
from pathlib import Path
import signal
import subprocess
import sys
import time

path=Path(__file__).resolve().parents[1]/'scripts/run_demo.py'
spec=importlib.util.spec_from_file_location('physical_demo_runner',path)
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)


def test_track_new_session_child_after_parent_exits():
    code=('import subprocess,sys,time; '
          'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"],start_new_session=True); '
          'print(p.pid,flush=True); time.sleep(60)')
    parent=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,text=True)
    child=int(parent.stdout.readline());owned={}
    try:
        runner.remember_descendants([parent.pid],owned)
        assert child in owned
        assert owned[child][1]==child  # Independent child PGID.
        parent.terminate();parent.wait(timeout=3)
        assert runner.process_identity(child)[2]==owned[child][2]
        runner.signal_descendants(owned,signal.SIGTERM)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            proc=Path(f'/proc/{child}/stat')
            if not proc.exists() or proc.read_text().rsplit(')',1)[1].split()[0]=='Z':break
            time.sleep(.02)
        else:raise AssertionError('Orphaned child did not stop')
    finally:
        if parent.poll() is None:parent.kill();parent.wait()
        runner.signal_descendants(owned,signal.SIGKILL)
        parent.stdout.close()


def test_reused_pid_is_never_signalled(monkeypatch):
    calls=[]
    monkeypatch.setattr(runner,'process_identity',lambda pid:(1,pid,200))
    monkeypatch.setattr(runner.os,'kill',lambda *args:calls.append(args))
    runner.signal_descendants({123:(1,123,100)},signal.SIGKILL)
    assert calls==[]
