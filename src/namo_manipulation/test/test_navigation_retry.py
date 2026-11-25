"""Replanning recovery stops first, validates fresh observations, and keeps G."""
import pytest
from collections import deque
from rcl_interfaces.msg import Log
from namo_sim.mission import Mission
from namo_manipulation.mission import PhysicalMission


def fixture(state='DETOUR',path=True):
    trace=[]
    # Preserve the inheritance relation used by super(), without starting DDS.
    node=object.__new__(PhysicalMission)
    node.report={};node.state=state;node.scans=10;node.costmap_time=1.
    node.sim_now=lambda:5.;node.pose=lambda:(0.,0.,0.)
    node.record_navigation_abort=lambda *a:''
    node.stop=lambda:trace.append('stop')
    def wait(predicate,timeout,reason):
        assert trace[-1]=='stop'
        node.scans+=4;node.costmap_time+=1.
        assert predicate()
        trace.append('fresh_observation')
    node.wait=wait
    def plan(check_target):
        assert trace[-1]=='fresh_observation'
        trace.append('fresh_plan')
        return [(0,0),(0,3.4)] if path else None
    node.plan_detour=plan;node.event=lambda *a,**k:None
    def work_plan(xy):
        assert trace[-1]=='fresh_observation'
        trace.append('fresh_work_plan')
        return [(0,0),xy] if path else None
    node.plan_work_path=work_plan
    return node,trace


def test_aborted_navigation_rechecks_while_stopped_then_retries_same_goal(monkeypatch):
    node,trace=fixture()
    def navigate(self,xy,detect):
        trace.append('drive')
        if len(self.report['navigation_goals'])==1:raise RuntimeError('NAV_EXECUTION_FAILED status=6')
        return False
    monkeypatch.setattr(Mission,'navigate',navigate)
    assert PhysicalMission.navigate(node,(0,3.4)) is False
    assert trace==['drive','stop','fresh_observation','fresh_plan','drive']
    assert [g['goal'][:2] for g in node.report['navigation_goals']]==[[0.,3.4],[0.,3.4]]


@pytest.mark.parametrize('state,error,path,expected_attempts',[
    ('WORK_APPROACH','NAV_EXECUTION_FAILED status=6',True,3),
    ('WORK_APPROACH','NAV_EXECUTION_FAILED status=6',False,1),
    ('WORK_APPROACH','SENSOR_STALE',True,1),
    ('APPROACH','NAV_EXECUTION_FAILED status=6',True,1),
    ('DETOUR','SENSOR_STALE',True,1),
    ('DETOUR','NAV_EXECUTION_FAILED status=6',False,1),
    ('DETOUR','NAV_EXECUTION_FAILED status=6',True,3),
])
def test_no_unbounded_retry_or_sensor_failure_bypass(monkeypatch,state,error,path,expected_attempts):
    node,trace=fixture(state,path)
    def failed(*args,**kwargs):raise RuntimeError(error)
    monkeypatch.setattr(Mission,'navigate',failed)
    with pytest.raises(RuntimeError):PhysicalMission.navigate(node,(0,3.4))
    assert len(node.report['navigation_goals'])==expected_attempts


def test_work_retry_validates_work_goal_instead_of_destination(monkeypatch):
    node,trace=fixture('WORK_APPROACH')
    def navigate(self,xy,detect):
        trace.append('drive')
        if len(self.report['navigation_goals'])==1:raise RuntimeError('NAV_EXECUTION_FAILED status=6')
        return False
    monkeypatch.setattr(Mission,'navigate',navigate)
    assert PhysicalMission.navigate(node,(0,.74),heading=0.) is False
    assert trace==['drive','stop','fresh_observation','fresh_work_plan','drive']
    assert [g['goal'] for g in node.report['navigation_goals']]==[[0.,.74,0.],[0.,.74,0.]]


def test_humble_log_message_levels_and_logger_filter():
    node=object.__new__(PhysicalMission);node.nav_diagnostics=deque(maxlen=40)
    node.on_nav_log(Log(level=30,name='planner_server',msg='No valid path'))
    node.on_nav_log(Log(level=40,name='controller_server',msg='Failed to make progress'))
    node.on_nav_log(Log(level=20,name='planner_server',msg='Routine update'))
    node.on_nav_log(Log(level=40,name='unrelated_node',msg='Not navigation'))
    assert [d['message'] for d in node.nav_diagnostics]==['No valid path','Failed to make progress']
