"""Check provenance and FK against the actual generated TF chain, not copied constants."""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from namo_manipulation.arm_kinematics import forward, STOW, RECOGNITION

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from model_sources import mount_defaults, origin_transform, transform


def chain(robot, target, values=None):
    values = values or {}
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    result = np.eye(4)
    while target != 'base_link':
        joint = joints[target]
        local = origin_transform(joint.find('origin'))
        if joint.get('type') == 'revolute':
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            turn = np.eye(4)
            turn[:3,:3] = Rotation.from_rotvec(axis*values.get(joint.get('name'),0.)).as_matrix()
            local = local @ turn
        result = local @ result
        target = joint.find('parent').get('link')
    return result


def test_pinned_official_source_and_mounts_survive_generation():
    manifest = json.loads((ROOT/'models/official/manifest.json').read_text())
    for file, digest in manifest['files'].items():
        assert hashlib.sha256((ROOT/'models/official'/file).read_bytes()).hexdigest() == digest
    robot = ET.parse(ROOT/'models/robot.urdf').getroot()
    for tag in ('joint','link'):
        names = [e.get('name') for e in robot.findall(tag)]
        assert len(names) == len(set(names))
    for joint, expected in [('base_fixed',[-.092,0,.091]),('scan_joint',[-.024,0,.122]),
                            ('end_effector_joint',[.126,0,0])]:
        assert np.fromstring(robot.find(f"joint[@name='{joint}']/origin").get('xyz'),sep=' ') == pytest.approx(expected)
    assert robot.find("link[@name='rear_ballast_link']") is None


@pytest.mark.parametrize('q',[STOW, RECOGNITION, np.deg2rad([55.,25.,-20.,20.])])
def test_analytic_fk_and_eated_hand_eye_match_generated_robot(q):
    robot = ET.parse(ROOT/'models/robot.urdf').getroot()
    joints = dict(zip(('joint1','joint2','joint3','joint4'),q))
    ee = chain(robot,'end_effector_link',joints)
    fk = forward(q)
    assert fk['end_effector'] == pytest.approx(ee[:3,3],abs=1e-10)
    assert fk['rotation'] == pytest.approx(ee[:3,:3],abs=1e-10)
    assert fk['tip'] == pytest.approx((ee @ [.014,0,0,1])[:3],abs=1e-10)
    mount = mount_defaults(ROOT.parent/'vision_pkg/launch/vision.launch.py')
    expected = transform([float(mount[k]) for k in ('x','y','z')],
                         [float(mount[k]) for k in ('roll','pitch','yaw')])
    actual = np.linalg.inv(ee) @ chain(robot,'d435i_link',joints)
    assert actual == pytest.approx(expected,abs=1e-10)
