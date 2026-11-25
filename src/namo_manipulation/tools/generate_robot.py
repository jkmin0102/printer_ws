#!/usr/bin/env python3
"""Generate a physical robot from pinned ROBOTIS Xacros and RealSense assets.

All changes affect this generated copy only. No pose-setter or attach plugin is
included. Controller position commands are implemented by bounded force PID,
including both fingers, so object contact remains in the physics solver.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from model_sources import expand

ROOT = Path(__file__).resolve().parents[1]


def add(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rear-ballast-mass', type=float, default=0.,
                        help='Visible rear ballast mass in kg; simulation assumption, 0 disables')
    parser.add_argument('--wheel-acceleration', type=float, default=0.,
                        help='Gazebo diff-drive wheel-speed limiter; 0 uses direct bounded motor constraint')
    args = parser.parse_args()
    if not math.isfinite(args.rear_ballast_mass) or args.rear_ballast_mass < 0:
        parser.error('--rear-ballast-mass must be finite and nonnegative')
    if not math.isfinite(args.wheel_acceleration) or args.wheel_acceleration < 0:
        parser.error('--wheel-acceleration must be finite and nonnegative')
    robot, manifest = expand(ROOT)
    robot.set('name', 'namo_manipulator')
    # The upstream acceleration branch accumulates a wheel-speed instruction
    # from measured-speed error. Disable this extra integrator by default;
    # Gazebo still solves the finite-fmax velocity motor constraint physically.
    robot.find("gazebo/plugin[@name='diff_drive']/max_wheel_acceleration").text = str(args.wheel_acceleration)
    # fdir1 is fixed in the rotating collision frame. The upstream local X
    # therefore approaches the ground normal after a quarter wheel turn,
    # invalidating ODE's assumed tangential friction basis. Match the normal
    # TurtleBot3 Waffle Pi SDF: let ODE derive both tangents from the normal.
    for side in ('left', 'right'):
        robot.find(f"gazebo[@reference='wheel_{side}_link']/fdir1").text = '0 0 0'
    # ROBOTIS lists Waffle Pi as 1.8 kg including SBC, battery and sensors.
    # Remaining base mass after the separately modelled wheels, casters and LDS:
    # 1.8 - 2*.02849894 - 2*.005 - .114 = 1.61900212 kg.
    # COM and box inertia are explicit simulation assumptions at the chassis
    # collision-box centre, NOT measured hardware inertial calibration. The
    # upstream inertial origin at wheel x=0 tipped forward with the wrist load.
    base_mass = 1.8 - 2*.02849894 - 2*.005 - .114
    base = robot.find("link[@name='base_link']/inertial")
    base.find('origin').set('xyz', '-0.064 0 0.047')
    base.find('mass').set('value', str(base_mass))
    inertia = base.find('inertia')
    for axis, value in {'ixx': base_mass*(.266**2+.094**2)/12,
                        'iyy': base_mass*(.266**2+.094**2)/12,
                        'izz': base_mass*(2*.266**2)/12,
                        'ixy': 0., 'ixz': 0., 'iyz': 0.}.items():
        inertia.set(axis, str(value))
    if args.rear_ballast_mass > 0:
        # Visible, removable top/rear weight. No hidden support or world joint.
        mass = args.rear_ballast_mass
        ballast = add(robot, 'link', name='rear_ballast_link')
        for tag in ('visual', 'collision'):
            body = add(ballast, tag)
            add(add(body, 'geometry'), 'box', size='.06 .14 .04')
            if tag == 'visual':
                add(add(body, 'material', name='ballast_yellow'), 'color', rgba='1 .7 .05 1')
        inertial = add(ballast, 'inertial')
        add(inertial, 'mass', value=str(mass))
        add(inertial, 'inertia', ixx=str(mass*(.14**2+.04**2)/12),
            iyy=str(mass*(.06**2+.04**2)/12), izz=str(mass*(.06**2+.14**2)/12),
            ixy='0', ixz='0', iyz='0')
        joint = add(robot, 'joint', name='rear_ballast_joint', type='fixed')
        add(joint, 'parent', link='base_link')
        add(joint, 'child', link='rear_ballast_link')
        add(joint, 'origin', xyz='-.16 0 .13', rpy='0 0 0')
        add(add(robot, 'gazebo', reference='rear_ballast_link'), 'material', 'Gazebo/Yellow')
    control = robot.find('ros2_control')
    # Wheels are controlled only by gazebo_ros_diff_drive. Keeping their unused
    # ros2_control command handles would make hold_joints fight the drive plugin.
    for element in list(control):
        if element.tag == 'sensor' or element.get('name', '').startswith('wheel_'):
            control.remove(element)
    # Keep startup away from mechanical limits while controllers activate.
    # The mission commands and verifies STOW before enabling base movement.
    initial = {'joint1': 0.0, 'joint2': -0.8, 'joint3': -0.5, 'joint4': 1.3,
               'gripper_left_joint': 0.019, 'gripper_right_joint': 0.019}
    # The upstream URDF's generic 1 Nm limit saturates the shoulder with the
    # wrist camera extended. Use a finite 2 Nm simulation limit; this is not a
    # measured continuous rating (XM430-W350 specifies 4.1 Nm stall at 12 V).
    for name in ('joint1','joint2','joint3','joint4'):
        robot.find(f"joint[@name='{name}']/limit").set('effort','2.0')
    for joint in control.findall('joint'):
        name = joint.get('name')
        finger = name.startswith('gripper_')
        # Humble GazeboSystem copies mimic POSITION commands but not
        # POSITION_PID commands. Drive each jaw with a force PID and mirror
        # action goals in the removal executor; publish each measured joint.
        for parameter in list(joint.findall('param')):
            if parameter.get('name') in ('mimic', 'multiplier'):
                joint.remove(parameter)
        joint.find('command_interface').set('name', 'position_pid')
        for param, value in {'pos_kp': 400.0 if finger else 80.0,
                             # A derivative gain of 1 generated a saturated
                             # high-frequency limit cycle in the small wrist
                             # inertia at the 500 Hz explicit force loop.
                             'pos_kd': 0.10,
                             'pos_ki': 0.0, 'pos_min_integral_error': -1.0,
                             'pos_max_integral_error': 1.0}.items():
            add(joint, 'param', value, name=param)
        joint.find("state_interface[@name='position']/param").text = str(initial[name])
        add(joint, 'state_interface', name='effort')
    plugin = robot.find("gazebo/plugin[@name='gazebo_ros2_control']")
    plugin.find('parameters').text = '__NAMO_CONTROLLERS__'
    add(plugin, 'robot_param_node', 'robot_state_publisher')
    add(plugin, 'hold_joints', 'true')
    # Explicit pad surrogate is easier to verify than triangle-mesh contacts.
    # Original palm visual mesh is retained, contact/friction parameters are
    # assumptions (not measurements): 10 g pad, mu=.8, bounded 1 N closing force.
    for side in ('left', 'right'):
        name = 'gripper_' + side + '_link'
        link = robot.find(f"link[@name='{name}']")
        for collision in link.findall('collision'):
            link.remove(collision)
        # URDF→SDF appends '_collision' to an explicit collision name.
        collision = add(link, 'collision', name=name)
        add(collision, 'origin', xyz='0.0583 0 0', rpy='0 0 0')
        add(add(collision, 'geometry'), 'box', size='0.05 0.008 0.025')
        link.find('inertial/mass').set('value', '0.01')
        inertia = link.find('inertial/inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            inertia.set(axis, '0.000003')
        physical_joint = robot.find(f"joint[@name='gripper_{side}_joint']")
        for mimic in physical_joint.findall('mimic'):
            physical_joint.remove(mimic)
        physical_joint.find('limit').set('velocity', '0.06')
        physical_joint.find('dynamics').set('damping', '0.03')
        gazebo = robot.find(f"gazebo[@reference='{name}']")
        for key, value in {'mu1': .8, 'mu2': .8, 'kp': 100000.0,
                           'kd': 10.0, 'minDepth': .0001, 'maxVel': .02}.items():
            gazebo.find(key).text = str(value)
        sensor = add(gazebo, 'sensor', name=side+'_finger_contact', type='contact')
        add(sensor, 'always_on', 'true')
        add(sensor, 'update_rate', 100)
        add(add(sensor, 'contact'), 'collision', name+'_collision')
        bumper = add(sensor, 'plugin', name=side+'_finger_contact_ros', filename='libgazebo_ros_bumper.so')
        ros = add(bumper, 'ros')
        add(ros, 'namespace', '/namo')
        add(ros, 'remapping', 'bumper_states:='+side+'_finger_contacts')
        add(bumper, 'frame_name', name)
    # Retain official scan_joint, visual mesh and collision geometry. Replace
    # only the legacy LDS-01 sensor plugin with our LDS-02 simulation settings.
    gazebo = robot.find("gazebo[@reference='base_scan']")
    for sensor in gazebo.findall('sensor'):
        gazebo.remove(sensor)
    # The physical hand-eye camera is used in this experiment; avoid an unused
    # second Pi RGB stream while retaining the official Pi camera frame tree.
    for element in robot.findall("gazebo[@reference='camera_rgb_frame']"):
        robot.remove(element)
    sensor = add(gazebo, 'sensor', name='lds02', type='ray')
    add(sensor, 'always_on', 'true')
    add(sensor, 'visualize', 'true')
    add(sensor, 'update_rate', 10)
    ray = add(sensor, 'ray')
    horizontal = add(add(ray, 'scan'), 'horizontal')
    for key, value in {'samples': 360, 'resolution': 1, 'min_angle': -3.14159265,
                       'max_angle': 3.14159265}.items():
        add(horizontal, key, value)
    limits = add(ray, 'range')
    for key, value in {'min': .12, 'max': 8.0, 'resolution': .005}.items():
        add(limits, key, value)
    noise = add(ray, 'noise')
    add(noise, 'type', 'gaussian'); add(noise, 'mean', 0); add(noise, 'stddev', .002)
    plugin = add(sensor, 'plugin', name='lds02_ros', filename='libgazebo_ros_ray_sensor.so')
    add(add(plugin, 'ros'), 'remapping', '~/out:=/scan')
    add(plugin, 'output_type', 'sensor_msgs/LaserScan')
    add(plugin, 'frame_name', 'base_scan')
    # Render aligned RGB and depth at the SAME color-camera pose/intrinsics.
    # Gazebo's camera looks along +X; the existing nominal optical joint rotates
    # this into optical +Z, +X right, +Y down. No extra sensor rotation is needed.
    gazebo = add(robot, 'gazebo', reference='d435i_color_frame')
    sensor = add(gazebo, 'sensor', name='wrist_rgbd', type='depth')
    add(sensor, 'always_on', 'true'); add(sensor, 'update_rate', 15)
    add(sensor, 'visualize', 'false')
    camera = add(sensor, 'camera')
    add(camera, 'horizontal_fov', 1.211)
    image = add(camera, 'image')
    add(image, 'width', 640); add(image, 'height', 480); add(image, 'format', 'R8G8B8')
    clip = add(camera, 'clip'); add(clip, 'near', .04); add(clip, 'far', 4.0)
    add(camera, 'depth_camera')
    plugin = add(sensor, 'plugin', name='wrist_rgbd_ros', filename='libgazebo_ros_camera.so')
    ros = add(plugin, 'ros')
    add(ros, 'namespace', '/realsense/d435i')
    for old, new in {'color/image_raw': 'color/image_raw',
                     'color/camera_info': 'color/camera_info',
                     'color/depth/image_raw': 'aligned_depth_to_color/image_raw',
                     'color/depth/camera_info': 'aligned_depth_to_color/camera_info',
                     'color/points': 'depth/color/points'}.items():
        add(ros, 'remapping', old+':='+new)
    add(plugin, 'camera_name', 'color')
    add(plugin, 'frame_name', 'd435i_color_optical_frame')
    add(plugin, 'min_depth', .04); add(plugin, 'max_depth', 4.0)
    wheel_plugin = add(add(robot, 'gazebo'), 'plugin', name='wheel_joint_states',
                       filename='libgazebo_ros_joint_state_publisher.so')
    add(wheel_plugin, 'update_rate', 30)
    add(wheel_plugin, 'joint_name', 'wheel_left_joint')
    add(wheel_plugin, 'joint_name', 'wheel_right_joint')
    ET.indent(robot)
    (ROOT / 'models/robot.urdf').write_text(ET.tostring(robot, encoding='unicode'))
    manifest['generated_urdf_sha256'] = hashlib.sha256((ROOT/'models/robot.urdf').read_bytes()).hexdigest()
    manifest['simulation_overrides'] = dict(rear_ballast_mass=args.rear_ballast_mass,
        wheel_acceleration=args.wheel_acceleration, arm_effort_Nm=2.,
        note='Force PID, pad contact proxies, base inertia, LDS-02 and aligned RGB-D plugins; official mount joints unchanged')
    (ROOT/'models/robot_sources.json').write_text(json.dumps(manifest, indent=2)+'\n')
    def xyz(joint):
        return [float(v) for v in robot.find(f"joint[@name='{joint}']/origin").get('xyz').split()]
    shoulder = [sum(v) for v in zip(*(xyz(name) for name in ('base_fixed','joint1','joint2')))]
    camera_xyz = [float(manifest['hand_eye']['values'][key]) for key in ('x','y','z')]
    camera_rpy = [float(manifest['hand_eye']['values'][key]) for key in ('roll','pitch','yaw')]
    geometry = dict(shoulder=shoulder, end_effector=xyz('end_effector_joint')[0],
                    grasp_tcp=.140, camera_xyz=camera_xyz, camera_rpy=camera_rpy,
                    camera_collision_xyz=[float(v) for v in robot.find("link[@name='d435i_link']/collision/origin").get('xyz').split()],
                    official_commit=manifest['commit'])
    (ROOT/'namo_manipulation/robot_geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
    print('Generated', ROOT / 'models/robot.urdf')


if __name__ == '__main__':
    main()
