"""Expand pinned ROBOTIS sources and import the existing EATED camera mount."""
import ast
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro


def mount_defaults(path):
    values = {}
    for node in ast.walk(ast.parse(Path(path).read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument' and node.args):
            name = ast.literal_eval(node.args[0])
            if name.startswith('camera_mount_'):
                value = next(k.value for k in node.keywords if k.arg == 'default_value')
                values[name.removeprefix('camera_mount_')] = ast.literal_eval(value)
    if set(values) != {'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'parent_frame', 'child_frame'}:
        raise ValueError('Incomplete EATED hand-eye definition')
    return values


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def origin_transform(element):
    return transform([float(v) for v in element.get('xyz', '0 0 0').split()],
                     [float(v) for v in element.get('rpy', '0 0 0').split()])


def expand(root):
    official = root / 'models/official'
    manifest = json.loads((official / 'manifest.json').read_text())
    # Resolve includes into an isolated temporary tree. Stored upstream files
    # remain byte-for-byte originals; installed/local modified Xacros are unused.
    with tempfile.TemporaryDirectory(prefix='namo_official_') as temp:
        for relative, digest in manifest['files'].items():
            source = official / relative
            if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
                raise ValueError('Official source checksum mismatch: ' + relative)
            target = Path(temp) / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            contents = source.read_text().replace(
                'filename="$(find turtlebot3_manipulation_description)/',
                'filename="' + temp + '/')
            target.write_text(contents)
        xml = xacro.process_file(str(root / 'models/robot.urdf.xacro'), mappings={
            'use_sim': 'true', 'official_description': temp,
        }).toxml()
    robot = ET.fromstring(xml)
    sources = [root.parent / 'vision_pkg/launch' / name
               for name in ('vision.launch.py', 'vision2.launch.py')]
    mounts = [mount_defaults(p) for p in sources]
    if mounts[0] != mounts[1]:
        raise ValueError('EATED vision launch files disagree about the hand-eye mount')
    mount = mounts[0]
    if mount['parent_frame'] != 'end_effector_link' or mount['child_frame'] != 'd435i_link':
        raise ValueError('Unexpected EATED camera mount frames')
    wanted = transform([float(mount[k]) for k in ('x', 'y', 'z')],
                       [float(mount[k]) for k in ('roll', 'pitch', 'yaw')])
    internal = origin_transform(robot.find("joint[@name='d435i_link_joint']/origin"))
    screw = wanted @ np.linalg.inv(internal)
    origin = robot.find("joint[@name='d435i_joint']/origin")
    origin.set('xyz', ' '.join(map(str, screw[:3, 3])))
    origin.set('rpy', ' '.join(map(str, Rotation.from_matrix(screw[:3, :3]).as_euler('xyz'))))
    assert np.allclose(origin_transform(origin) @ internal, wanted, atol=1e-12)
    manifest['hand_eye'] = dict(values=mount, sources={
        str(p.relative_to(root.parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    manifest['hand_eye']['calibration_status'] = 'existing configured transform; not measured calibration'
    return robot, manifest
