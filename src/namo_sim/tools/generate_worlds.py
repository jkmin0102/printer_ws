#!/usr/bin/env python3
"""Generate physical walls and occupancy maps from the same metric rectangles."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
RESOLUTION = 0.05
ORIGIN = (-2.8, -3.2)


def rectangles(detour_y):
    """Occupied boxes (xmin, ymin, xmax, ymax), corridor clear width 1.4 m."""
    return [
        (-1.1, -3.1, -0.7, detour_y + 1.1),
        (6.7, -3.1, 7.1, detour_y + 1.1),
        (-0.7, detour_y + 0.7, 6.7, detour_y + 1.1),
        (-0.7, -3.1, 6.7, -2.7),
        (0.7, -2.7, 6.7, -0.7),
        (0.7, 0.7, 5.3, detour_y - 0.7),
    ]


def box_model(name, rect, height=0.8, color='0.62 0.68 0.74 1'):
    x0, y0, x1, y1 = rect
    return f'''<model name="{name}"><static>true</static>
      <pose>{(x0+x1)/2} {(y0+y1)/2} {height/2} 0 0 0</pose>
      <link name="body"><collision name="collision"><geometry><box>
        <size>{x1-x0} {y1-y0} {height}</size>
      </box></geometry></collision><visual name="visual"><geometry><box>
        <size>{x1-x0} {y1-y0} {height}</size>
      </box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse>
      </material></visual></link></model>'''


def generate(scenario, detour_y):
    walls = rectangles(detour_y)
    width = round((7.6 - ORIGIN[0]) / RESOLUTION)
    height = round((detour_y + 1.6 - ORIGIN[1]) / RESOLUTION)
    xs = ORIGIN[0] + (np.arange(width) + 0.5) * RESOLUTION
    ys = ORIGIN[1] + (np.arange(height) + 0.5) * RESOLUTION
    grid = np.full((height, width), 254, dtype=np.uint8)
    for x0, y0, x1, y1 in walls:
        grid[np.ix_((ys >= y0) & (ys < y1), (xs >= x0) & (xs < x1))] = 0
    Image.fromarray(np.flipud(grid)).save(ROOT / 'maps' / f'{scenario}.pgm')
    (ROOT / 'maps' / f'{scenario}.yaml').write_text(yaml.safe_dump({
        'image': f'{scenario}.pgm', 'mode': 'trinary',
        'resolution': RESOLUTION, 'origin': [*ORIGIN, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
    }, sort_keys=False))
    world = f'''<?xml version="1.0"?>
<sdf version="1.6"><world name="namo_{scenario}">
  <gravity>0 0 -9.81</gravity>
  <physics name="default" type="ode"><max_step_size>0.001</max_step_size>
    <real_time_update_rate>1000</real_time_update_rate><real_time_factor>1</real_time_factor>
  </physics>
  <include><uri>model://sun</uri></include>
  <include><uri>model://ground_plane</uri></include>
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/gazebo</namespace></ros><update_rate>10</update_rate>
  </plugin>
  <scene><ambient>0.7 0.7 0.7 1</ambient><shadows>false</shadows></scene>
  <gui fullscreen="0"><camera name="user_camera">
    <pose>3 {detour_y/2} 13 0 1.5707 1.5707</pose><view_controller>orbit</view_controller>
  </camera></gui>
  {''.join(box_model(f'wall_{i}', rect) for i, rect in enumerate(walls))}
  {box_model('stash_pad', (-2.5,-2.5,-1.5,-1.5),0.01,'0.2 0.7 0.3 1')}
</world></sdf>'''
    ET.fromstring(world)
    (ROOT / 'worlds' / f'{scenario}.world').write_text(world)
    (ROOT / 'maps' / f'{scenario}_geometry.json').write_text(json.dumps({
        'scenario': scenario, 'wall_rectangles': walls, 'corridor_width': 1.4,
        'start': [0.0, -2.0, 1.5707963267948966], 'observation': [0.0, 0.0, 0.0],
        'goal': [6.0, 0.0, 0.0], 'detour_y': detour_y,
        'stash': [-2.0, -2.0, 0.3],
    }, indent=2) + '\n')


if __name__ == '__main__':
    generate('A', 5.0)
    generate('B', 1.8)
    obstacle = '''<?xml version="1.0"?><sdf version="1.6">
<model name="movable_obstacle"><static>false</static><link name="body">
<gravity>false</gravity><inertial><mass>100.0</mass><inertia>
<ixx>11.333333</ixx><iyy>4.6875</iyy><izz>10.020833</izz>
</inertia></inertial>
<collision name="collision"><geometry><box><size>0.45 1.0 0.6</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>0.45 1.0 0.6</size></box></geometry>
<material><ambient>0.9 0.12 0.04 1</ambient><diffuse>0.9 0.12 0.04 1</diffuse></material>
</visual></link></model></sdf>'''
    (ROOT / 'models' / 'obstacle.sdf').write_text(obstacle)
