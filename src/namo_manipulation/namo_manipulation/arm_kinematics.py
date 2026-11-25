"""OpenMANIPULATOR-X FK/IK and conservative experiment trajectory checks.

Dimensions and limits are from the installed ROBOTIS manipulation URDF. Joint
position control remains Gazebo force/PID control: these functions only create
joint trajectories. They never write link/entity poses or attach an object.
"""
from dataclasses import dataclass
import math
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4')
LIMITS = np.array([[-.9*math.pi, .9*math.pi], [-.57*math.pi, .5*math.pi],
                   [-.3*math.pi, .44*math.pi], [-.57*math.pi, .65*math.pi]])
GEOMETRY = json.loads(Path(__file__).with_name('robot_geometry.json').read_text())
SHOULDER = np.array(GEOMETRY['shoulder'])
L1 = math.hypot(.024, .128)
L2 = .124
# Official end_effector_joint is 126 mm; physical pad centre is 140 mm
# from joint4. IK uses this grasp TCP, NOT the named end_effector_link.
END_EFFECTOR = GEOMETRY['end_effector']
TOOL = GEOMETRY['grasp_tcp']
CAMERA_ROTATION = Rotation.from_euler('xyz', GEOMETRY['camera_rpy']).as_matrix()
CAMERA_XYZ = np.array(GEOMETRY['camera_xyz'])
ALPHA = math.atan2(.128, .024)
RECOGNITION = np.deg2rad([0., -30., -41., 115.])
STOW = np.deg2rad([0., -101., 53., 70.])


class ArmPlanError(ValueError):
    pass


def rotation(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return np.array([[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0., cp]])


def forward(joints):
    q = np.asarray(joints, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ArmPlanError('Four finite arm joint positions required')
    shoulder = SHOULDER.copy()
    elbow = shoulder + rotation(q[0], q[1]) @ np.array([.024, 0., .128])
    wrist = elbow + rotation(q[0], q[1]+q[2]) @ np.array([L2, 0., 0.])
    orientation = rotation(q[0], sum(q[1:]))
    tip = wrist + orientation @ np.array([TOOL, 0., 0.])
    return {'shoulder': shoulder, 'elbow': elbow, 'wrist': wrist, 'tip': tip,
            'end_effector': wrist + orientation @ np.array([END_EFFECTOR, 0., 0.]),
            'rotation': orientation, 'pitch': float(sum(q[1:]))}


def inverse(xyz, pitch=math.pi/4):
    point = np.asarray(xyz, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or not math.isfinite(pitch):
        raise ArmPlanError('Finite XYZ and pitch required')
    dx, dy = point[0]-SHOULDER[0], point[1]
    q1 = math.atan2(dy, dx)
    radius = math.hypot(dx, dy)-TOOL*math.cos(pitch)
    height = point[2]-SHOULDER[2]+TOOL*math.sin(pitch)
    if radius < 0.:
        raise ArmPlanError('Target is inside the wrist/tool minimum radial workspace')
    cosine = (radius*radius+height*height-L1*L1-L2*L2)/(2*L1*L2)
    if abs(cosine) > 1.0+1e-9:
        raise ArmPlanError(f'Unreachable target {tuple(point)} at pitch {math.degrees(pitch):.1f} deg')
    delta = -math.acos(min(1., max(-1., cosine)))
    beta = math.atan2(height, radius)-math.atan2(L2*math.sin(delta), L1+L2*math.cos(delta))
    q2, q3 = ALPHA-beta, -ALPHA-delta
    q = np.array([q1, q2, q3, pitch-q2-q3])
    if np.any(q < LIMITS[:, 0]+1e-5) or np.any(q > LIMITS[:, 1]-1e-5):
        raise ArmPlanError(f'Joint limit violation: {np.rad2deg(q).round(2).tolist()} deg')
    if np.linalg.norm(forward(q)['tip']-point) > 1e-8:
        raise ArmPlanError('IK forward consistency check failed')
    return q


def segment_points(a, b, step=.008):
    count = max(1, math.ceil(float(np.linalg.norm(b-a))/step))
    return [a+(b-a)*i/count for i in range(count+1)]


def collision_spheres(joints, finger_position=.019):
    """Capsule samples for links/pads plus enclosing camera-body spheres.

    The finger opening is intentionally left empty; a pinch object may occupy
    that gap. The camera proxy includes its actual installed collision box.
    A runtime MoveIt mesh collision model remains preferable for arbitrary scenes.
    """
    f = forward(joints)
    wrist, r = f['wrist'], f['rotation']
    pieces = [('upper_arm', f['shoulder'], f['elbow'], .024),
              ('forearm', f['elbow'], wrist, .024),
              ('gripper_body', wrist, wrist+r@np.array([.092,0.,0.]), .026)]
    for sign in (-1., 1.):
        y = sign*(.021+finger_position)
        pieces.append(('finger', wrist+r@np.array([.115,y,0.]),
                       wrist+r@np.array([.165,y,0.]), .0135))
    samples = [(name, p, radius) for name, a, b, radius in pieces for p in segment_points(a,b)]
    # D435i collision: .02505 x .09 x .025 at EE+(-.065,.002,.052).
    # A short chain of overlapping spheres covers the wide camera body.
    for y in np.linspace(-.045,.045,13):
        camera_local = np.array(GEOMETRY['camera_collision_xyz']) + [0., y, 0.]
        p = f['end_effector']+r@(CAMERA_XYZ+CAMERA_ROTATION@camera_local)
        samples.append(('camera',p,.0185))
    return samples


def validate_configuration(joints, floor_z=-.01, obstacles=(), environment_check=None,
                           finger_position=.019):
    q = np.asarray(joints, dtype=float)
    if np.any(q < LIMITS[:,0]) or np.any(q > LIMITS[:,1]):
        raise ArmPlanError('Joint limit violation')
    spheres = collision_spheres(q, finger_position)
    # Waffle Pi base box; mounted fixed arm column is intentionally excluded.
    boxes = [(-.20,-.16,-.01,.14,.16,.10),
             (-.064,-.055,.09975,.046,.055,.13125), *obstacles]
    for name, point, radius in spheres:
        if point[2]-radius < floor_z+.002:
            raise ArmPlanError(f'{name} intersects floor clearance')
        for box in boxes:
            lower, upper = np.asarray(box[:3]), np.asarray(box[3:])
            distance = float(np.linalg.norm(point-np.maximum(lower,np.minimum(point,upper))))
            if distance < radius:
                raise ArmPlanError(f'{name} intersects base or configured collision box')
        if environment_check is not None and not environment_check(point, radius):
            raise ArmPlanError(f'{name} intersects static-map wall clearance')
    # Non-adjacent arm links: upper arm against gripper/camera/fingers.
    upper = [(p,r) for n,p,r in spheres if n=='upper_arm']
    distal = [(p,r) for n,p,r in spheres if n in ('gripper_body','finger','camera')]
    if any(np.linalg.norm(p-q) < r+s for p,r in upper for q,s in distal):
        raise ArmPlanError('Non-adjacent arm proxy collision')


@dataclass
class JointPlan:
    positions: list
    times: list

    @property
    def duration(self):
        return self.times[-1] if self.times else 0.


def joint_plan(start, goal, velocity=.45, minimum_duration=1.0, validator=None):
    start, goal = np.asarray(start,dtype=float), np.asarray(goal,dtype=float)
    if velocity <= 0:
        raise ArmPlanError('Positive joint velocity required')
    count = max(1, math.ceil(float(np.max(np.abs(goal-start)))/.025))
    duration = max(minimum_duration, float(np.max(np.abs(goal-start)))/velocity)
    positions, times = [], []
    # Quintic position profile gives zero velocity/acceleration at both ends.
    # Its peak derivative is 1.875; duration respects configured peak velocity.
    duration = max(duration, 1.875*float(np.max(np.abs(goal-start)))/velocity)
    for i in range(count+1):
        u=i/count
        s=10*u**3-15*u**4+6*u**5
        q=start+(goal-start)*s
        if validator: validator(q)
        positions.append(q.tolist());times.append(duration*u)
    return JointPlan(positions,times)


def cartesian_plan(start, end_xyz, pitch=math.pi/4, cartesian_speed=.025,
                   joint_velocity=.45, validator=None):
    start = np.asarray(start,dtype=float)
    start_xyz = forward(start)['tip']
    if abs(forward(start)['pitch']-pitch) > .03:
        raise ArmPlanError('Cartesian path requires matching starting tool pitch')
    end_xyz=np.asarray(end_xyz,dtype=float)
    distance=float(np.linalg.norm(end_xyz-start_xyz))
    count=max(2,math.ceil(distance/.004))
    q_values=[inverse(start_xyz+(end_xyz-start_xyz)*i/count,pitch) for i in range(count+1)]
    if any(np.max(np.abs(b-a)) > .12 for a,b in zip(q_values,q_values[1:])):
        raise ArmPlanError('Cartesian IK discontinuity')
    times=[0.]
    for i,q in enumerate(q_values):
        if validator: validator(q)
        if i:
            dt=max(distance/count/cartesian_speed,float(np.max(np.abs(q-q_values[i-1])))/joint_velocity,.02)
            times.append(times[-1]+dt)
    return JointPlan([q.tolist() for q in q_values],times)
