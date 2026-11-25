"""Bounded physical re-observation keeps confidence/freshness and arm locks."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from namo_manipulation.mission import PhysicalMission
from namo_manipulation.arm_executor import PhysicalRemover
from namo_manipulation.arm_kinematics import joint_plan,validate_configuration


@pytest.mark.parametrize('success_at',[0,1,2,None])
def test_view_retry_is_bounded_and_returns_only_after_fresh_recognition(success_at):
    node=object.__new__(PhysicalMission);node.report={}
    node.can_observation={'valid':False,'reason':'Target not detected'}
    trace=[];attempts=[]
    node.event=lambda *a,**k:None
    node.arm_operation=lambda operation,state:trace.append((operation,state))
    node.camera_ready=lambda after:len(attempts)-1==success_at
    def wait(predicate,timeout,reason):
        attempts.append(1)
        if not predicate():raise RuntimeError(reason)
    node.wait=wait
    if success_at is None:
        with pytest.raises(RuntimeError,match='FRESH_CAN_RECOGNITION_FAILED'):node.recognize_can()
    else:node.recognize_can()
    expected=3 if success_at is None else success_at+1
    assert len(attempts)==expected
    assert trace==[('VIEW','PREPARED')]*(expected-1)
    assert sum(a['success'] for a in node.report['recognition_attempts'])==(success_at is not None)


def test_other_failure_does_not_trigger_camera_motion():
    node=object.__new__(PhysicalMission);node.report={};node.event=lambda *a,**k:None
    node.wait=lambda *args:(_ for _ in ()).throw(RuntimeError('CLOCK_RESET'))
    node.arm_operation=lambda *args:pytest.fail('must not move')
    with pytest.raises(RuntimeError,match='CLOCK_RESET'):node.recognize_can()


@pytest.mark.parametrize('state,holding,index',[('STOWED',False,0),('SUCCEEDED',False,0),('PREPARED',True,0),('PREPARED',False,2)])
def test_adjust_view_requires_prepared_empty_arm_and_budget(state,holding,index):
    node=SimpleNamespace(state=state,busy=False,holding=holding,view_index=index)
    response=SimpleNamespace(success=True,message='')
    assert not PhysicalRemover.request(node,'VIEW',response).success


def test_alternate_view_paths_clear_robot_and_retain_recognition_height():
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/arm.yaml').read_text())['physical_remover']['ros__parameters']
    nominal=np.deg2rad(cfg['recognition_joints_deg']);previous=nominal
    for yaw in (-5.,5.):
        target=nominal.copy();target[0]+=np.deg2rad(yaw)
        plan=joint_plan(previous,target,validator=validate_configuration)
        assert np.allclose(plan.positions[-1],target)
        previous=target
