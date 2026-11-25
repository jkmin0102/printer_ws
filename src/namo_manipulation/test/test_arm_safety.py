"""Stop authorization and per-joint freshness without creating ROS nodes."""
import math
import time
from types import SimpleNamespace

from namo_manipulation.arm_executor import PhysicalRemover
from namo_manipulation.arm_kinematics import JOINT_NAMES, STOW


def fixture():
    now=time.monotonic()
    names=(*JOINT_NAMES,'gripper_left_joint','gripper_right_joint')
    return SimpleNamespace(busy=False,base_start_pose=None,odom_pose=None,gate_received=now,drive_enabled=False,odom_received=now,
        odom_speed=(0.,0.),odom_stamp=10.,joints={j:0. for j in names},
        joint_receipts={j:now for j in names},joint_stamps={j:10. for j in names},
        sim_now=lambda:10.,measured_velocity={j:0. for j in names})


def test_running_base_and_missing_gate_reject_arm_operation():
    node=fixture()
    assert PhysicalRemover.unsafe(node)==''
    node.drive_enabled=True
    assert 'gate' in PhysicalRemover.unsafe(node)
    node.drive_enabled=False
    node.odom_speed=(.05,0.)
    assert 'moving' in PhysicalRemover.unsafe(node)


def test_wheel_message_cannot_refresh_stale_arm_joints():
    node=fixture()
    node.joint_receipts['joint2']-=2.
    msg=SimpleNamespace(name=['wheel_left_joint','wheel_right_joint'],position=[0.,0.],
                        velocity=[0.,0.],header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
    PhysicalRemover.on_joints(node,msg)
    assert 'individual arm' in PhysicalRemover.unsafe(node)


def test_stale_finger_also_blocks_operation():
    node=fixture()
    node.joint_stamps['gripper_right_joint']=8.
    assert 'both finger' in PhysicalRemover.unsafe(node)


def test_stow_status_is_revalidated_against_measured_joints():
    node=fixture();node.state='STOWED';node.phase='STOW'
    node.joints.update(zip(JOINT_NAMES,STOW))
    assert PhysicalRemover.stowed_verified(node)
    node.joints['joint3']+=.10
    assert not PhysicalRemover.stowed_verified(node)
    node.joints.update(zip(JOINT_NAMES,STOW))
    node.joint_receipts['joint1']-=2.
    assert not PhysicalRemover.stowed_verified(node)


def test_small_velocity_cannot_hide_accumulated_base_drift():
    node=fixture();node.busy=True
    node.base_start_pose=(0.,0.,0.);node.odom_pose=(.02,0.,0.)
    node.p=lambda name:{'max_base_drift':.01,'max_base_yaw_drift':.03}[name]
    assert 'Base drift' in PhysicalRemover.unsafe(node)
