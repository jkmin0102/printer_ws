#!/usr/bin/env python3
"""Metric floor plan: a short vertical corridor and a finite arm-only side bay.

All walls and occupancy cells come from the same free rectangles. The 50 cm
bay mouth cannot accommodate the 56 cm navigation clearance diameter.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
RES = .025
START = [-1.10, 0., 0.]
GOAL = [0., 3.4]
CAN = [0., 1.10, .0675]
WORK = [-.12, .76, 1.5707963267948966]
PLACE = [-.43, .90, .0675]


def box(name, r, height=.6, color='.64 .68 .73 1'):
    x0,y0,x1,y1 = r
    return f'''<model name="{name}"><static>true</static><pose>{(x0+x1)/2} {(y0+y1)/2} {height/2} 0 0 0</pose>
    <link name="body"><collision name="collision"><geometry><box><size>{x1-x0} {y1-y0} {height}</size></box></geometry></collision>
    <visual name="visual"><geometry><box><size>{x1-x0} {y1-y0} {height}</size></box></geometry>
    <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'''


def generate(scenario, bypass_x):
    origin = [-1.75, -.65]
    nx,ny = round((bypass_x+.65-origin[0])/RES), round((4.05-origin[1])/RES)
    xs=origin[0]+(np.arange(nx)+.5)*RES
    ys=origin[1]+(np.arange(ny)+.5)*RES
    # Keep the manipulation lane narrow; give travelling/turning corridors
    # enough allowance for localization and rasterized LiDAR wall observations.
    free=[(-1.50,-.45,bypass_x+.45,.45),(-.35,-.45,.35,3.85),
          (bypass_x-.45,-.45,bypass_x+.45,3.85),(-.35,2.95,bypass_x+.45,3.85),
          (-.65,.55,-.35,1.05)]
    data=np.zeros((ny,nx),dtype=np.uint8)
    for x0,y0,x1,y1 in free:
        data[np.ix_((ys>=y0)&(ys<y1),(xs>=x0)&(xs<x1))]=254
    # Merge equal occupied runs across consecutive rows into physical boxes.
    active={}; rectangles=[]
    for iy,row in enumerate(data):
        changes=np.diff(np.r_[False,row==0,False].astype(int))
        spans=set(zip(np.flatnonzero(changes==1),np.flatnonzero(changes==-1)))
        for span,ybegin in list(active.items()):
            if span not in spans:
                rectangles.append((origin[0]+span[0]*RES,origin[1]+ybegin*RES,
                                   origin[0]+span[1]*RES,origin[1]+iy*RES));del active[span]
        for span in spans:
            active.setdefault(span,iy)
    for span,ybegin in active.items():
        rectangles.append((origin[0]+span[0]*RES,origin[1]+ybegin*RES,
                           origin[0]+span[1]*RES,origin[1]+ny*RES))
    Image.fromarray(np.flipud(data)).save(ROOT/'maps'/f'{scenario}.pgm')
    (ROOT/'maps'/f'{scenario}.yaml').write_text(yaml.safe_dump(dict(
        image=f'{scenario}.pgm',mode='trinary',resolution=RES,origin=origin+[0.],
        negate=0,occupied_thresh=.65,free_thresh=.25),sort_keys=False))
    geometry=dict(scenario=scenario,wall_rectangles=rectangles,free_rectangles=free,
                  corridor_width=.70,travel_corridor_width=.90,bay_mouth=.50,start=START,goal=GOAL,can_fixture=CAN,
                  work_hint=WORK,place=PLACE,bypass_x=bypass_x,can_diameter=.05,can_height=.135,
                  empty_can_mass_kg=.015,mass_basis='simulation assumption; not measured')
    (ROOT/'maps'/f'{scenario}_geometry.json').write_text(json.dumps(geometry,indent=2))
    world=f'''<sdf version="1.6"><world name="namo_arm_{scenario}">
    <gravity>0 0 -9.81</gravity><physics type="ode" name="default"><max_step_size>0.001</max_step_size>
    <real_time_update_rate>1000</real_time_update_rate><ode><solver><iters>100</iters></solver></ode></physics>
    <include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>
    <scene><ambient>.8 .8 .8 1</ambient><shadows>false</shadows></scene>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>20</update_rate></plugin>
    <gui fullscreen="0"><camera name="user_camera"><pose>{bypass_x/2} 1.5 6.5 0 1.5707 1.5707</pose><view_controller>orbit</view_controller></camera></gui>
    {''.join(box(f'wall_{i}',r) for i,r in enumerate(rectangles))}
    {box('place_pad',(-.50,.81,-.35,1.01),.002,'.2 .7 .3 1')}
    </world></sdf>'''
    ET.fromstring(world)
    (ROOT/'worlds'/f'{scenario}.world').write_text(world)


if __name__=='__main__':
    generate('A',9.5)
    generate('B',1.15)
