import math
import numpy as np
import pytest
from pathlib import Path
import yaml
from namo_manipulation.arm_kinematics import (
    ArmPlanError, RECOGNITION, STOW, cartesian_plan, forward, inverse,
    joint_plan, validate_configuration,
)


def test_fk_ik_real_urdf_dimensions():
    for xyz in ((.128,.292,.0675),(.128,.292,.1475),(-.3305,.17,.0675),(-.3305,.17,.1475)):
        q=inverse(xyz,math.radians(25))
        assert forward(q)['tip']==pytest.approx(xyz,abs=1e-9)
        assert forward(q)['pitch']==pytest.approx(math.radians(25))
        validate_configuration(q)


def test_top_down_far_drop_is_rejected_not_clipped():
    with pytest.raises(ArmPlanError): inverse((.12,.33,.0675),math.pi/2)
    with pytest.raises(ArmPlanError): inverse((.8,0,.1))


def test_known_stow_recognition_and_interpolation_checked():
    validate_configuration(STOW)
    validate_configuration(RECOGNITION)
    plan=joint_plan(STOW,RECOGNITION,validator=validate_configuration)
    assert plan.positions[0]==pytest.approx(STOW)
    assert plan.positions[-1]==pytest.approx(RECOGNITION)
    assert np.all(np.diff(plan.times)>0)


def test_floor_and_scene_obstacle_rejected():
    q=inverse((.128,.292,.0675),math.radians(25))
    with pytest.raises(ArmPlanError): validate_configuration(q,floor_z=.15)
    with pytest.raises(ArmPlanError):
        validate_configuration(q,obstacles=[(.10,.25,0.,.16,.33,.15)])


def test_carry_path_all_samples_safe_and_no_ik_jump():
    q=inverse((.128,.292,.1475),math.radians(25))
    plan=cartesian_plan(q,(-.3305,.17,.1475),math.radians(25),validator=validate_configuration)
    assert forward(plan.positions[-1])['tip']==pytest.approx((-.3305,.17,.1475))
    assert len(plan.positions)>20
    assert plan.duration>0


def test_configured_official_mount_pick_transfer_release_and_stow():
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/arm.yaml').read_text())['physical_remover']['ros__parameters']
    pitch=math.radians(cfg['tool_pitch_deg'])
    pick=np.array(cfg['nominal_pick_xyz'])+[0,0,cfg['grasp_height_offset']]
    place=np.array(cfg['nominal_place_xyz'])+[0,0,cfg['grasp_height_offset']+.002]
    points=[pick+[0,0,cfg['approach_height']],pick,
            pick+[0,0,cfg['lift_height']],place+[0,0,cfg['lift_height']],place]
    recognition=np.deg2rad(cfg['recognition_joints_deg'])
    joint_plan(STOW,recognition,validator=validate_configuration)
    q=inverse(points[0],pitch)
    joint_plan(recognition,q,validator=validate_configuration)
    for point in points[1:]:
        plan=cartesian_plan(q,point,pitch,validator=validate_configuration)
        q=np.array(plan.positions[-1])
    retreat=place-forward(q)['rotation'][:,0]*.08
    plan=cartesian_plan(q,retreat,pitch,validator=validate_configuration)
    joint_plan(plan.positions[-1],STOW,validator=validate_configuration)
