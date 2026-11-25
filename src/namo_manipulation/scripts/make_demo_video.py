#!/usr/bin/env python3
"""Join successful recorded A/B runs with explicitly labelled playback speeds."""
import argparse
import json
import math
from pathlib import Path
import subprocess


def duration(path):
    return float(subprocess.check_output(['ffprobe', '-v', 'error', '-show_entries',
        'format=duration', '-of', 'default=noprint_wrappers=1:nokey=1', str(path)], text=True))


def dimensions(path):
    info=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
        '-show_entries','stream=width,height','-of','json',str(path)],text=True))
    return info['streams'][0]['width'],info['streams'][0]['height']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('a', type=Path)
    p.add_argument('b', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--grasp-inset', action='store_true',
                   help='Magnify the verified fixed closeup work area in the unused Gazebo sidebar')
    p.add_argument('--navigation-speed',type=float,default=3.)
    p.add_argument('--departure-speed',type=float,default=None,
                   help='Override post-placement playback speed (1 shows the new departure in real time)')
    p.add_argument('--crop-top',type=int,default=0,help='Exclude window title/tool bars from the edited video')
    p.add_argument('--crop-bottom',type=int,default=0,help='Exclude bottom window chrome from the edited video')
    args = p.parse_args()
    if not math.isfinite(args.navigation_speed) or args.navigation_speed<1.:
        p.error('--navigation-speed must be finite and at least 1')
    if args.departure_speed is not None and (not math.isfinite(args.departure_speed) or args.departure_speed<1.):
        p.error('--departure-speed must be finite and at least 1')
    results=[]
    for folder, choice in ((args.a, 'REMOVE'), (args.b, 'DETOUR')):
        r = json.loads((folder/'result.json').read_text())
        if not r['execution_completed'] or r['choice'] != choice:
            raise RuntimeError('Only completed REMOVE and DETOUR runs can enter the presentation video')
        results.append(r)
    events = [json.loads(s) for s in (args.a/'events.jsonl').read_text().splitlines()]
    # The recorder starts shortly before the mission process. Generous phase
    # boundaries keep all grasp/release motion in the slower playback section.
    start = max(0., next(e['wall_elapsed'] for e in events if e['state']=='PHYSICAL_REMOVE')-1.)
    end = next(e['wall_elapsed'] for e in events if e['state']=='VERIFY_CLEAR')+4.
    if args.departure_speed is not None:
        # Include the entire initial heading adjustment at the requested speed.
        # The recorder starts just before the mission; begin two seconds early.
        end = next(e['wall_elapsed'] for e in events if e['state']=='GOAL_AFTER_REMOVE')-2.
    total_a, total_b = duration(args.a/'screen.mp4'), duration(args.b/'screen.mp4')
    rate=args.navigation_speed
    departure_rate=args.departure_speed if args.departure_speed is not None else rate
    segments = [(0, 0., start, rate, f'A | 캔 앞 접근 | 주행 {rate:g}배속'),
                (0, start, min(end,total_a), 1.25, 'A | 실제 접촉으로 집기 → 옆 공간에 놓기 | 1.25배속'),
                (0, min(end,total_a), total_a, departure_rate, f'A | 현재 위치 → G 한 목표 · 경로 계속 갱신 | {departure_rate:g}배속'),
                (1, 0., total_b, rate, f'B | 캔을 유지하고 우회 → 목적지 | 주행 {rate:g}배속')]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    width,height=dimensions(args.a/'screen.mp4')
    width=width//2*2;height=height//2*2
    if min(args.crop_top,args.crop_bottom)<0 or args.crop_top+args.crop_bottom>=height-100:
        p.error('Crop must retain at least 100 pixels of source height')
    canvas_height=(height-args.crop_top-args.crop_bottom)//2*2
    if args.grasp_inset and (args.crop_top or args.crop_bottom):
        p.error('Legacy inset does not support window-chrome cropping')
    if args.grasp_inset and (width,height)!=(1600,720):
        p.error('--grasp-inset is only supported for the legacy 1600 x 720 sidebar layout')
    filters = []
    for i,(source,begin,finish,speed,title) in enumerate(segments):
        caption = args.output.parent/f'caption_{i}.txt'
        caption.write_text(title)
        prefix=(f'[{source}:v]trim=start={begin}:end={finish},setpts=(PTS-STARTPTS)/{speed},'
                f'scale={width}:{height}:force_original_aspect_ratio=decrease,'
                f'pad={width}:{height}:(ow-iw)/2:(oh-ih)/2,setsar=1,'
                f'crop={width}:{canvas_height}:0:{args.crop_top}')
        if args.grasp_inset and i==1:
            filters.append(prefix+'[grasp]')
            filters.append('[grasp]split[base][detail]')
            filters.append('[detail]crop=160:170:420:270,scale=250:266[zoom]')
            prefix='[base][zoom]overlay=10:170,'+f'drawtext=fontfile={font}:text=집기 확대 (실제 녹화):fontcolor=white:fontsize=19:x=10:y=140:box=1:boxcolor=black'
        filters.append(prefix+f',pad={width}:{canvas_height+44}:0:0:black,'
            f'drawtext=fontfile={font}:textfile={caption.resolve()}:fontcolor=white:fontsize=23:'
            f'x=16:y={canvas_height+6},fps=20[v{i}]')
    filters.append(''.join(f'[v{i}]' for i in range(4))+'concat=n=4:v=1:a=0[out]')
    graph = args.output.parent/'video_filter.txt'
    graph.write_text(';\n'.join(filters))
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'warning', '-filter_complex_threads', '2', '-i', str(args.a/'screen.mp4'),
        '-i', str(args.b/'screen.mp4'), '-filter_complex_script', str(graph), '-map', '[out]',
        '-an', '-threads', '2', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '22', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(args.output)], check=True)
    manifest = dict(a=str(args.a.resolve()), b=str(args.b.resolve()), segments=segments,
                    output_seconds=duration(args.output), raw_recordings_preserved=True,
                    simulation_video=True, perception={r['scenario']:r['perception_mode'] for r in results},
                    source_size=[width,height],content_crop=[0,args.crop_top,width,canvas_height],
                    yolo_success_claimed=results[0]['perception_mode']=='yolo',
                    grasp_inset=args.grasp_inset, inset_source_crop=[420,270,160,170] if args.grasp_inset else None)
    args.output.with_suffix('.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
