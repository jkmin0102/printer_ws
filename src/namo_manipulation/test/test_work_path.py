"""An action success is insufficient: the live path must really reach work safely."""
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Path
from rclpy.task import Future
from namo_manipulation.mission import PhysicalMission


def completed(value):
    future=Future();future.set_result(value);return future


def fixture(points=((0.,0.),(0.,.74)),status=4):
    node=object.__new__(PhysicalMission)
    node.pose=lambda:(0.,0.,0.)
    def stamped(xy,heading=None):
        pose=PoseStamped();pose.header.frame_id='map'
        pose.pose.position.x=float(xy[0]);pose.pose.position.y=float(xy[1])
        pose.pose.orientation.w=1.;return pose
    node.stamped=stamped
    node.p=lambda key:{'planning_timeout':5.,'robot_radius':.28}[key]
    node.wait=lambda predicate,*args:predicate() or pytest.fail('unexpected wait')
    path=Path(poses=[stamped(p) for p in points])
    result=SimpleNamespace(status=status,result=SimpleNamespace(path=path))
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:completed(result))
    requests=[]
    def send(request):requests.append(request);return completed(handle)
    node.planner=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=send)
    cm=OccupancyGrid();cm.info.resolution=.025;cm.info.width=cm.info.height=100
    cm.info.origin.position.x=cm.info.origin.position.y=-1.
    cm.info.origin.orientation.w=1.;cm.data=[0]*10000;node.costmap=cm
    node.target={'points':[(0.,1.075)]};node.observation={'other_clusters':[]}
    return node,requests


def test_actual_work_request_uses_current_start_and_exact_work_goal():
    node,requests=fixture()
    assert node.plan_work_path((0.,.74))
    request=requests[0]
    assert request.use_start and request.planner_id=='GridBased'
    assert request.goal.pose.position.y==.74
    assert request.start.pose.position.y==0.


@pytest.mark.parametrize('case',['aborted','substituted_goal','blocked_middle','can_intersection','other_obstacle'])
def test_reject_invalid_work_path_even_when_server_claims_success(case):
    points=((0.,0.),(0.,.74))
    if case=='substituted_goal':points=((0.,0.),(0.,.70))
    if case=='can_intersection':points=((0.,0.),(0.,1.),(0.,.74))
    node,_=fixture(points,status=6 if case=='aborted' else 4)
    if case=='blocked_middle':node.costmap.data[56*100+40]=100
    if case=='other_obstacle':node.observation['other_clusters']=[{'points':[(.2,.4)]}]
    assert node.plan_work_path((0.,.74)) is None
