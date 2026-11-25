"""A later YOLO result must not invalidate or substitute the selected image."""
from collections import deque
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseStamped
from namo_manipulation.arm_executor import PhysicalRemover
from namo_manipulation.arm_kinematics import ArmPlanError


def fixture():
    pose=PoseStamped();pose.header.frame_id='base_link'
    pose.header.stamp.sec=10;pose.header.stamp.nanosec=100000000
    pose.pose.position.y=.34;pose.pose.position.z=.0575
    selected=dict(valid=True,stable=True,mode='YOLO',frame_id='base_link',
                  stamp=dict(sec=10,nanosec=100000000),center_xyz=[0.,.34,.0575])
    newer=dict(selected,stamp=dict(sec=10,nanosec=900000000),center_xyz=[.001,.34,.0575])
    node=SimpleNamespace(target_pose=pose,observations=deque([selected,newer],maxlen=16),
        p=lambda k:{'allow_fixture_target':False,'observation_max_age':2.}[k],
        sim_now=lambda:11.,target_in_base=lambda p:[p.pose.position.x,p.pose.position.y,p.pose.position.z])
    return node


def test_newer_inference_does_not_replace_selected_frame_metadata():
    node=fixture()
    assert PhysicalRemover.observed_pick(node)==[0.,.34,.0575]
    assert node.source=='RGBD_YOLO'


@pytest.mark.parametrize('case',['missing','invalid','stale','frame','coordinates'])
def test_nonmatching_or_invalid_selected_observation_is_rejected(case):
    node=fixture()
    if case=='missing':node.observations.popleft()
    elif case=='invalid':node.observations[0]['stable']=False
    elif case=='stale':node.sim_now=lambda:13.
    elif case=='frame':node.observations[0]['frame_id']='odom'
    elif case=='coordinates':node.observations[0]['center_xyz']=[.1,.34,.0575]
    with pytest.raises(ArmPlanError):PhysicalRemover.observed_pick(node)
