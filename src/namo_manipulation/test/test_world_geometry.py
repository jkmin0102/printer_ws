"""Meaningful layout checks: a graspable can must really block the short route."""
from pathlib import Path
import math
import numpy as np
from PIL import Image
import yaml
from namo_sim.planning import Grid,path_length

ROOT=Path(__file__).resolve().parents[1]

def circle(x,y):
    return [(x+.025*math.cos(a),y+.025*math.sin(a)) for a in np.linspace(0,2*math.pi,40)]

def test_can_blocks_short_route_and_side_bay_is_not_a_robot_bypass():
    for scenario in 'AB':
        config=yaml.safe_load((ROOT/'maps'/f'{scenario}.yaml').read_text())
        pixels=np.flipud(np.array(Image.open(ROOT/'maps'/f'{scenario}.pgm')))
        grid=Grid(np.where(pixels==0,100,0),config['resolution'],config['origin'])
        blocked=grid.astar((0,0),(0,3.4),.28,circle(0,1.1))
        cleared=grid.astar((0,0),(0,3.4),.28,circle(-.43,.90))
        assert grid.is_free((-.12,.76),.28,circle(-.43,.90))
        assert max(p[0] for p in blocked)>.90  # Must enter the separate right corridor.
        assert max(abs(p[0]) for p in cleared)<.1
        assert path_length(blocked)>path_length(cleared)+1.5
        for dx in [-.015,0.,.015]:
            for dy in [-.015,0.,.015]:
                assert grid.is_free((-.12+dx,.76+dy),.28,circle(0,1.1))
