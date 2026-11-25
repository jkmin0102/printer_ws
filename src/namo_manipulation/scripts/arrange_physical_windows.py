#!/usr/bin/env python3
"""Reuse PID-safe layout checks and widen the two physical-demo windows."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gazebo-pid', type=int, required=True)
    parser.add_argument('--rviz-pid', type=int, required=True)
    parser.add_argument('--rviz-title', default='manipulation.rviz')
    args = parser.parse_args()
    args.display = os.environ.get('DISPLAY')
    args.gazebo_title = 'Gazebo'
    args.wait = 30.
    source = Path(__file__).resolve().parents[2]/'namo_sim/scripts/arrange_windows.py'
    spec = importlib.util.spec_from_file_location('namo_window_utils', source)
    layout = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(layout)
    result = layout.arrange(args)
    x11 = layout.X11(args.display)
    try:
        screen = x11.attributes(x11.root)
        areas=x11.property(x11.root, '_NET_WORKAREA')
        desktop=x11.property(x11.root, '_NET_CURRENT_DESKTOP')
        offset=4*(desktop[0] if desktop else 0)
        area=areas[offset:offset+4] if len(areas)>=offset+4 else [0,0,screen.width,screen.height]
        x,y=result['x'],result['y']
        width=(min(screen.width,area[0]+area[2])-x)//4*4
        height=(min(screen.height,area[1]+area[3])-y)//2*2
        if width<1280 or height<720:
            raise RuntimeError('Demo layout needs at least 1280 x 720 of desktop work area')
        selected = layout.select_windows(x11, args)
        if selected is None:
            raise RuntimeError('Verified demo windows disappeared')
        targets = [[x+width//2*i, y, width//2, height] for i in range(2)]
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            for target, kind in zip(targets, ('gazebo', 'rviz')):
                x11.place(selected[kind][0], target)
            time.sleep(.2)
            actual = [x11.rectangle(selected[k][0]) for k in ('gazebo', 'rviz')]
            if actual != targets:
                continue
            layout.verify_no_managed_overlap(x11, selected, actual)
            result.update(width=width,height=height)
            for kind, rectangle in zip(('gazebo', 'rviz'), actual):
                result['windows'][kind]['outer_rectangle'] = rectangle
            print(json.dumps(result))
            return 0
        raise RuntimeError(f'Window manager did not accept the requested {width//2} x {height} layout')
    finally:
        x11.close()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(json.dumps({'ready': False, 'reason': str(exc)}))
        raise SystemExit(1)
