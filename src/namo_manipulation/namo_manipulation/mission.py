"""Sensor-driven navigation plus physical can pick/place. No simulator pose input.

The old mock demo stays unchanged. This class reuses its stamped scan, Nav2,
stop watchdog, conservative grid and sensor-clearance checks.
"""
import csv
import json
import math
import time
import copy
from collections import deque
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Log
from tf2_ros import Buffer,TransformListener
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker,MarkerArray
from namo_sim.mission import Mission,DURABLE,grid_from_msg,densify
from namo_sim.planning import path_length,PlanError


KOREAN={
    'WAIT_READY':'지도·센서·제어기 준비 중','STOW':'팔 주행 자세 확인 중',
    'APPROACH':'판단 지점 접근 중','OBSERVE':'LiDAR 장애물 관측 중',
    'STOP_OBSERVE':'장애물 확인, 베이스 정지','PLAN':'제거·우회 시간 비교 중',
    'DECISION':'행동 선택 완료','WORK_APPROACH':'캔 앞 작업 위치로 이동 중',
    'PREPARE':'베이스 정지, 카메라 인식 자세로 이동 중','RECOGNIZE':'카메라로 콜라 캔 위치 확인 중',
    'REOBSERVE':'베이스 정지, 카메라 관측 각도 조정 중',
    'PHYSICAL_REMOVE':'팔로 집기·옆 공간에 놓기 수행 중','VERIFY_CLEAR':'적치 후 통로 확보 확인 중',
    'GOAL_AFTER_REMOVE':'현재 위치에서 목적지까지 경로 갱신·주행 중','DETOUR':'장애물을 유지하고 우회 중',
    'GOAL_CLEAR':'장애물 없는 경로로 이동 중','DONE':'목적지 도착','FAILED':'실패, 주행 잠금',
    'DETECTION_VERIFIED':'LiDAR 감지·정지 검증 완료'}

ARM_KOREAN={'IDLE':'대기','STOWED':'주행 자세 확인','PREPARED':'인식 자세 도달',
    'RUNNING':'동작 중','BUSY':'동작 중','SETTLE':'자세 안정 확인','SUCCEEDED':'작업 완료','FAILED':'작업 실패',
    'STOW':'팔 접기','PREPARE':'인식 자세로 이동','OPEN':'그리퍼 벌리기',
    'VIEW':'카메라 관측 각도 조정',
    'PREGRASP':'캔 위 접근','APPROACH_CAN':'파지 위치로 내리기','CLOSE':'그리퍼 닫기',
    'VERIFY_PINCH':'양쪽 손가락 접촉 확인','LIFT':'캔 들어 올리기',
    'VERIFY_HOLD':'파지 유지 확인','TRANSFER':'옆 적치 공간으로 운반',
    'LOWER':'적치 위치로 내리기','RELEASE':'캔 놓기','PLACE_SETTLE':'캔이 바닥에 놓이도록 대기',
    'VERIFY_RELEASE':'캔과 손가락 분리 확인',
    'RETREAT':'손 빼기','':'대기'}

def failure_label(reason):
    labels={'SENSOR_STALE':'센서 관측 갱신이 늦어 주행을 중지했습니다.',
            'FRESH_CAN_RECOGNITION_FAILED':'안정적인 콜라 캔 인식을 확인하지 못했습니다.',
            'CLEARANCE_UNCONFIRMED':'적치 후 통로가 비었다는 관측이 부족합니다.',
            'SHORT_PASSAGE_NOT_VERIFIED':'적치 후 짧은 경로를 확인하지 못했습니다.',
            'NO_FEASIBLE_ACTION':'제거와 우회 모두 가능한 경로가 없습니다.',
            'ACTUAL_STOP_TIMEOUT':'베이스가 실제로 정지했는지 확인하지 못했습니다.',
            'READINESS_TIMEOUT':'지도·센서·주행 제어기 준비가 끝나지 않았습니다.'}
    for prefix,label in labels.items():
        if reason.startswith(prefix):return label
    for operation in ('REMOVE','PREPARE','STOW','VIEW'):
        if reason.startswith(operation+'_FAILED'):return '팔 작업이 중지되었습니다. '+reason
    return reason


def latest_ready_scan(scans, last_stamp, now, transform_ready):
    """A new scan can precede its TF by milliseconds; retain the previous scan."""
    for msg in reversed(scans):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if last_stamp < stamp <= now+.05 and now-stamp <= .75 and transform_ready(msg):
            return msg
    return None


class PhysicalMission(Mission):
    def __init__(self):
        self.arm_status={};self.can_observation={};self.can_pose=None
        self.arm_received=-math.inf;self.can_received=-math.inf
        self.heading=math.pi/2
        self.gate_status={};self.gate_received=-math.inf
        self.raw_scan_received=-math.inf;self.raw_scan_count=0;self.raw_scan_stamp=-math.inf
        self.scan_mask_error='';self.scan_diagnostic_logged=-math.inf
        self.scan_queue=deque(maxlen=8)
        self.nav_diagnostics=deque(maxlen=40);self.navigation_notice=''
        super().__init__()
        # Keep high-rate arm/odometry TF ingestion independent of mission
        # polling and grid work. Every scan still uses its original timestamp.
        self.listener.unregister()
        self.tf_node=Node('namo_mission_tf',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.tf=Buffer(node=self.tf_node,cache_time=Duration(seconds=20))
        self.listener=TransformListener(self.tf,self.tf_node,spin_thread=True,
            qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
        for name,value in dict(place_x=-.40,place_y=.90,place_z=.0675,work_standoff=.34,
                               work_left_offset=0.,work_heading_deg=0.,can_radius=.025,can_height=.135,
                               alignment_estimate=2.,operation_timeout=120.,perception_mode='yolo',
                               v_eff_basis='configured initial assumption',precision_goal_seconds=16.,
                               precision_goal_basis='configured initial assumption').items():
            self.declare_parameter(name,value)
        if not math.isfinite(self.p('precision_goal_seconds')) or self.p('precision_goal_seconds')<0:
            raise ValueError('precision_goal_seconds must be finite and nonnegative')
        self.status_pub=self.create_publisher(String,'/namo/status',DURABLE)
        self.pick_pub=self.create_publisher(PoseStamped,'/namo/pick_pose',DURABLE)
        self.controller_parameters=self.create_client(SetParameters,'/controller_server/set_parameters')
        self.arm_clients={name:self.create_client(Trigger,topic) for name,topic in
                          [('STOW','/namo/stow_arm'),('PREPARE','/namo/prepare_grasp'),
                           ('VIEW','/namo/adjust_grasp_view'),('REMOVE','/namo/physical_remove')]}
        self.create_subscription(String,'/namo/physical_removal_status',self.on_arm,DURABLE)
        self.create_subscription(String,'/namo/can_observation',self.on_observation,DURABLE)
        self.create_subscription(PoseStamped,'/namo/can_pose',self.on_can,10)
        self.create_subscription(String,'/namo/drive_gate_status',self.on_gate,10)
        self.create_subscription(Log,'/rosout',self.on_nav_log,
            QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.25,self.publish_status)
        self.report.update(execution_mode='physical_contact_simulation',perception_mode=self.p('perception_mode'),
                           can_diameter_m=.05,can_height_m=.135,can_mass_kg=.015,
                           can_mass_basis='empty-can simulation assumption; not a measurement',
                           time_basis='trajectory estimate plus measured execution; no target total time')

    def on_map(self,msg):
        super().on_map(msg)
        # A 5 cm can is deliberately accepted; old mock detector required 25 cm.
        self.detector.min_span=.015
        self.detector.cluster_distance=.075

    def on_scan(self,msg):
        self.raw_scan_received=time.monotonic();self.raw_scan_count+=1
        self.raw_scan_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        # Sensor-derived self mask for the folded mobile manipulator; this does
        # not edit the raw topic used by Nav2 or claim nearby space is clear.
        try:
            tr=self.tf.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time())
            q=tr.transform.rotation
            c=1-2*(q.y*q.y+q.z*q.z);s=2*(q.w*q.z+q.x*q.y)
            ranges=np.array(msg.ranges)
            a=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
            x=tr.transform.translation.x+ranges*(c*np.cos(a)-s*np.sin(a))
            y=tr.transform.translation.y+ranges*(s*np.cos(a)+c*np.sin(a))
            self_hits=(np.abs(x)<.245)&(np.abs(y)<.185)&np.isfinite(ranges)
            filtered=copy.deepcopy(msg)
            filtered.ranges=[float('nan') if hit else float(r) for r,hit in zip(ranges,self_hits)]
            super().on_scan(filtered)
            self.scan_queue.append(filtered)
        except Exception as exc:
            # No detector input until the fixed sensor mount is available.
            self.scan_mask_error=str(exc)
            return

    def process_scan(self):
        started=time.monotonic()
        ready=latest_ready_scan(self.scan_queue,self.scan_stamp,self.sim_now(),
            lambda msg:self.tf.can_transform('map',msg.header.frame_id,
                                              rclpy.time.Time.from_msg(msg.header.stamp)))
        if ready is not None:
            self.pending_scan=ready
        super().process_scan()
        while self.scan_queue:
            stamp=self.scan_queue[0].header.stamp
            if stamp.sec+stamp.nanosec/1e9>self.scan_stamp:break
            self.scan_queue.popleft()
        now=time.monotonic();pending=self.pending_scan
        diagnostic=dict(raw_count=self.raw_scan_count,processed_count=self.scans,
            raw_receipt_age=now-self.raw_scan_received,raw_stamp=self.raw_scan_stamp,
            processed_age=now-self.scan_time,processed_stamp=self.scan_stamp,
            odom_age=now-self.odom_time,odom_stamp=self.odom_stamp,tf_errors=self.tf_errors,
            process_seconds=now-started,mask_error=self.scan_mask_error,pending_stamp=None)
        if pending is not None:
            diagnostic['pending_stamp']=pending.header.stamp.sec+pending.header.stamp.nanosec/1e9
            try:
                tr=self.tf.lookup_transform('map',pending.header.frame_id,rclpy.time.Time.from_msg(pending.header.stamp))
                diagnostic['pending_tf_ready']=True
            except Exception as exc:
                diagnostic['pending_tf_ready']=False;diagnostic['pending_tf_error']=str(exc)
        self.report['sensor_diagnostics']=diagnostic
        if self.enabled and (diagnostic['processed_age']>.7 or diagnostic['odom_age']>.7) and now-self.scan_diagnostic_logged>1.:
            self.get_logger().warning('SENSOR_DIAGNOSTIC '+json.dumps(diagnostic));self.scan_diagnostic_logged=now

    def on_arm(self,msg):
        try:
            self.arm_status=json.loads(msg.data);self.arm_received=time.monotonic()
        except ValueError:
            self.arm_status={'state':'FAILED','reason':'Malformed arm status'}

    def on_observation(self,msg):
        try:self.can_observation=json.loads(msg.data)
        except ValueError:self.can_observation={'valid':False,'reason':'Malformed observation'}

    def on_can(self,msg):
        self.can_pose=msg;self.can_received=time.monotonic()

    def on_gate(self,msg):
        try:
            self.gate_status=json.loads(msg.data);self.gate_received=time.monotonic()
        except ValueError:self.gate_status={}

    def on_nav_log(self,msg):
        # Humble generates the byte WARN constant as bytes, but level as int.
        if msg.level>=30 and msg.name in (
                'planner_server','controller_server','bt_navigator',
                'bt_navigator_navigate_to_pose_rclcpp_node'):
            self.nav_diagnostics.append(dict(received=time.monotonic(),logger=msg.name,message=msg.msg))

    def record_navigation_abort(self,started,xy,attempt):
        details=[d for d in self.nav_diagnostics if d['received']>=started]
        self.report.setdefault('navigation_aborts',[]).append(dict(
            state=self.state,attempt=attempt,goal=list(xy),pose=list(self.pose()),
            sim=self.sim_now(),details=details,drive_gate=dict(self.gate_status)))
        useful=[d for d in details if 'Aborting handle' not in d['message']
                and d['message']!='Goal failed' and 'node halt' not in d['message']]
        detail=' | '.join(d['message'] for d in (useful or details)[-3:])
        self.navigation_notice='Nav2 중단: '+(detail or '상세 메시지 미수신; navigation.log 확인')
        self.get_logger().warning(self.navigation_notice)
        return detail

    def event(self,state,**data):
        super().event(state,**data)
        print(f'[NAMO] {KOREAN.get(state,state)}'+(' | '+json.dumps(data,ensure_ascii=False,default=str) if data else ''),flush=True)
        if hasattr(self,'status_pub'):self.publish_status()

    def publish_status(self):
        arm=self.arm_status
        status=dict(state=self.state,label=KOREAN.get(self.state,self.state),choice=self.choice,
                    scenario=self.p('scenario'),drive_requested=self.enabled,
                    drive_enabled=self.gate_status.get('allowed',False) and time.monotonic()-self.gate_received<.75,
                    drive_reason=self.gate_status.get('reason','상태 수신 대기'),base_stopped=self.stopped(),
                    perception_mode=self.p('perception_mode'),arm_state=arm.get('state','준비 중'),
                    arm_stage=arm.get('phase',''),arm_elapsed=arm.get('elapsed_sim_seconds'),
                    arm_state_label=ARM_KOREAN.get(arm.get('state'),arm.get('state','준비 중')),
                    arm_stage_label=ARM_KOREAN.get(arm.get('phase',''),arm.get('phase','')),
                    L_clear=self.report.get('L_clear'),L_detour=self.report.get('L_detour'),
                    stop_pose=self.report.get('stop_pose'),observed_surface=self.report.get('observed_surface'),
                    v_eff=self.p('v_eff'),estimated_precision_seconds=self.report.get('estimated_precision_seconds'),
                    T_remove=self.report.get('T_remove'),T_detour=self.report.get('T_detour'),
                    reason=self.report.get('decision_reason',''),failure=self.failure,
                    navigation_notice=self.navigation_notice,
                    failure_label=failure_label(self.failure),
                    elapsed=None if self.started_sim is None else self.sim_now()-self.started_sim,
                    camera_valid=self.can_observation.get('valid',False),
                    camera_reason=self.can_observation.get('reason','관측 수신 대기'),
                    prediction_label='판단 당시 예상 총시간',execution_mode='팔 접촉 물리 시뮬레이션')
        self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False)))

    def show_markers(self):
        if not hasattr(self,'markers'):return
        a=MarkerArray()
        place=(self.p('place_x'),self.p('place_y')) if self.has_parameter('place_x') else (-.40,.90)
        labels=[('S',-1.1,0.),('P',0.,0.),('G',0.,3.4),('PLACE',*place)]
        for i,(label,x,y) in enumerate(labels):
            m=Marker();m.header.frame_id='map';m.ns='places';m.id=i;m.type=Marker.TEXT_VIEW_FACING
            m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.35;m.pose.orientation.w=1.
            m.scale.z=.22;m.color.r=m.color.g=.12;m.color.b=.2;m.color.a=1.;m.text=label;a.markers.append(m)
        for i,(point,color) in enumerate(((self.target['center'] if self.target else None,(1.,.25,.2)),
                                         (self.stop_pose,(.95,.9,.2)))):
            if point is None:continue
            m=Marker();m.header.frame_id='map';m.ns='observations';m.id=i;m.type=Marker.SPHERE
            m.pose.position.x=float(point[0]);m.pose.position.y=float(point[1]);m.pose.position.z=.16
            m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=.10
            m.color.r,m.color.g,m.color.b=color;m.color.a=1.;a.markers.append(m)
        self.markers.publish(a)

    def stamped(self,xy,heading=None):
        return super().stamped(xy,self.heading if heading is None else heading)

    def navigate(self,xy,detect=False,heading=math.pi/2):
        self.heading=heading
        for attempt in range(3):
            started=time.monotonic()
            self.report.setdefault('navigation_goals',[]).append(dict(
                state=self.state,attempt=attempt,sim=self.sim_now(),start_pose=list(self.pose()),
                goal=[float(xy[0]),float(xy[1]),float(heading)]))
            try:return super().navigate(xy,detect)
            except RuntimeError as exc:
                if str(exc)!='NAV_EXECUTION_FAILED status=6':raise
                detail=self.record_navigation_abort(started,xy,attempt)
                if attempt==2 or self.state not in ('DETOUR','GOAL_AFTER_REMOVE','WORK_APPROACH'):
                    raise RuntimeError(str(exc)+(': '+detail if detail else '')) from exc
                # A changing costmap can briefly reject a replanning tick.
                # Parent navigate has closed the gate and measured a stop.
                # Do not continue on an old path or clear the observed map.
                self.stop();count=self.scans;before=self.costmap_time
                self.wait(lambda:self.scans>=count+4 and self.costmap_time>before,
                          5.,'NAV_RETRY_FRESH_OBSERVATION_TIMEOUT')
                self.stop_pose=self.pose()
                fresh=(self.plan_work_path(xy) if self.state=='WORK_APPROACH'
                       else self.plan_detour(check_target=self.state=='DETOUR'))
                if not fresh:raise RuntimeError('NAV_RETRY_NO_PATH') from exc
                self.navigation_notice=f'정지·센서 갱신·경로 검증 완료: 동일 목표 재시도 {attempt+1}/2'
                self.event(self.state,replanning_retry=attempt+1,
                           reason=self.navigation_notice)

    def plan_work_path(self,xy):
        """Ask the actual Nav2 planner for this work waypoint, never for G.

        Validate the returned path against the live inflated costmap and the
        observed can. No obstacle clearing, stale-path reuse or goal snapping.
        """
        if not self.planner.server_is_ready():raise RuntimeError('PLANNER_UNAVAILABLE')
        here=self.pose()
        request=ComputePathToPose.Goal(goal=self.stamped(xy),
            start=self.stamped(here[:2],here[2]),planner_id='GridBased',use_start=True)
        ack=self.planner.send_goal_async(request)
        self.wait(ack.done,self.p('planning_timeout'),'WORK_PLAN_ACK_TIMEOUT')
        handle=ack.result()
        if not handle.accepted:raise RuntimeError('WORK_PLAN_REJECTED')
        future=handle.get_result_async()
        try:self.wait(future.done,self.p('planning_timeout'),'WORK_PLAN_TIMEOUT')
        except Exception:
            handle.cancel_goal_async();raise
        result=future.result()
        if result.status!=GoalStatus.STATUS_SUCCEEDED:return None
        points=[(p.pose.position.x,p.pose.position.y) for p in result.result.path.poses]
        if not points or math.dist(points[0],here[:2])>.05 or math.dist(points[-1],xy)>.001:
            return None
        points=[here[:2]]+points
        cm=grid_from_msg(self.costmap,99)
        observed=[p for c in self.observation['other_clusters'] for p in c['points']]
        if self.target is not None:observed+=list(self.target['points'])
        for point in densify(points):
            if not cm.is_free(point,0.):return None
            if observed and min(math.dist(point,p) for p in observed)<self.p('robot_radius'):return None
        return points

    def arm_operation(self,operation,finished):
        self.stop()
        client=self.arm_clients[operation]
        self.wait(client.service_is_ready,20.,f'{operation}_SERVER_UNAVAILABLE')
        self.wait(lambda:self.arm_status.get('ready_for_operation',False),30.,f'{operation}_CONTROLLERS_NOT_READY')
        before=time.monotonic();start=self.sim_now()
        ack=client.call_async(Trigger.Request())
        self.wait(ack.done,10.,f'{operation}_ACK_TIMEOUT')
        if not ack.result().success:raise RuntimeError(f'{operation}_REJECTED: '+ack.result().message)
        self.wait(lambda:self.arm_received>before and self.arm_status.get('operation')==operation
                  and self.arm_status.get('state') in (finished,'FAILED'),self.p('operation_timeout'),f'{operation}_TIMEOUT')
        self.report[f'{operation.lower()}_result']=dict(self.arm_status)
        if self.arm_status.get('state')=='FAILED':
            raise RuntimeError(f"{operation}_FAILED[{self.arm_status.get('phase')}]: {self.arm_status.get('reason')}")
        self.report[f'{operation.lower()}_sim_seconds']=self.sim_now()-start
        self.report[f'{operation.lower()}_wall_seconds']=time.monotonic()-before

    def set_goal_precision(self,tolerance):
        self.wait(self.controller_parameters.service_is_ready,5.,'GOAL_PRECISION_SERVER_UNAVAILABLE')
        request=SetParameters.Request(parameters=[Parameter('general_goal_checker.xy_goal_tolerance',value=tolerance).to_parameter_msg()])
        future=self.controller_parameters.call_async(request)
        self.wait(future.done,5.,'GOAL_PRECISION_UPDATE_TIMEOUT')
        response=future.result()
        if len(response.results)!=1 or not response.results[0].successful:
            raise RuntimeError('GOAL_PRECISION_UPDATE_REJECTED')

    def placed_obstacle(self):
        return [(self.p('place_x')+self.p('can_radius')*math.cos(a),
                 self.p('place_y')+self.p('can_radius')*math.sin(a)) for a in np.linspace(0,2*math.pi,32)]

    def camera_ready(self,after):
        if self.can_pose is None or self.can_received<=after or time.monotonic()-self.can_received>.8:return False
        stamp=self.can_pose.header.stamp.sec+self.can_pose.header.stamp.nanosec/1e9
        if self.sim_now()-stamp>.8 or self.sim_now()-stamp<-.05:return False
        if self.can_pose.header.frame_id!='base_link':return False
        p=self.can_pose.pose.position
        observed_stamp=self.can_observation.get('stamp',{})
        return (self.can_observation.get('valid',False) and self.can_observation.get('stable',False)
                and observed_stamp.get('sec')==self.can_pose.header.stamp.sec
                and observed_stamp.get('nanosec')==self.can_pose.header.stamp.nanosec
                and .10<math.hypot(p.x,p.y)<.45 and -.01<p.z<.18)

    def recognize_can(self):
        """Try the fixed view, then two physically checked alternate views."""
        for view in range(3):
            if view:
                self.event('REOBSERVE',view_attempt=view,
                           previous_reason=self.can_observation.get('reason','관측 수신 없음'))
                self.arm_operation('VIEW','PREPARED')
            self.event('RECOGNIZE',view_attempt=view)
            after=time.monotonic()
            try:
                self.wait(lambda:self.camera_ready(after),12.,'FRESH_CAN_RECOGNITION_FAILED')
            except RuntimeError as exc:
                if str(exc)!='FRESH_CAN_RECOGNITION_FAILED':raise
                self.report.setdefault('recognition_attempts',[]).append(dict(
                    view=view,success=False,observation=dict(self.can_observation)))
                if view==2:raise
            else:
                self.report.setdefault('recognition_attempts',[]).append(dict(
                    view=view,success=True,observation=dict(self.can_observation)))
                return

    def refine_work_goal(self,initial):
        # AMCL can correct map->odom during the intersection turn. Reobserve
        # after the staging approach instead of approaching an old map point.
        count=self.scans;before=self.costmap_time
        self.wait(lambda:self.scans>=count+3 and self.observation.get('target') is not None
                  and self.costmap_time>before,5.,'WORK_REOBSERVATION_TIMEOUT')
        here=self.pose();surface=np.array(self.target['center'])
        direction=surface-np.array(here[:2]);direction/=max(1e-9,np.linalg.norm(direction))
        center=surface+self.p('can_radius')*direction
        cm=grid_from_msg(self.costmap,99)
        extra=[p for c in self.observation['other_clusters'] for p in c['points']]
        for backoff in (0.,.01,.02):
            candidate=(float(center[0]-self.p('work_left_offset')),
                       float(center[1]-self.p('work_standoff')-backoff))
            if math.dist(candidate,initial)>.12 or not cm.is_free(candidate,0.):continue
            try:
                self.grid.astar(here[:2],candidate,self.p('robot_radius'),
                                extra_points=extra+list(self.target['points']))
                self.grid.astar(candidate,(self.p('goal_x'),self.p('goal_y')),
                                self.p('robot_radius'),extra_points=extra+self.placed_obstacle())
            except PlanError:continue
            if not self.plan_work_path(candidate):continue
            self.report.update(initial_work_goal=list(initial),work_goal=list(candidate),
                               refreshed_can_center=center.tolist(),work_refinement_backoff_m=backoff)
            self.frozen_target=copy.deepcopy(self.target)
            return candidate
        raise RuntimeError('NO_FRESH_COLLISION_FREE_WORK_GOAL')

    def run(self):
        success=False
        try:
            self.wait(self.ready,self.p('ready_timeout'),'READINESS_TIMEOUT')
            self.started_sim=self.sim_now();self.mission_started_wall=time.monotonic()
            self.event('STOW');self.arm_operation('STOW','STOWED')
            self.event('APPROACH')
            if not self.navigate((self.p('observation_x'),self.p('observation_y')),True):
                self.event('OBSERVE');n=self.scans
                self.wait(lambda:self.target is not None or self.scans-n>=8,8.,'OBSERVATION_TIMEOUT')
            if self.target is None:
                self.choice='CLEAR';self.event('GOAL_CLEAR')
                self.navigate((self.p('goal_x'),self.p('goal_y')),True)
            if self.target is not None:
                self.event('STOP_OBSERVE',observed_surface=list(self.target['center']))
                self.stop();self.stop_pose=self.pose()
                self.report.update(stop_pose=list(self.stop_pose),observed_surface=list(self.target['center']))
                self.frozen_target={k:np.array(v).copy() if isinstance(v,(list,tuple,np.ndarray)) else v for k,v in self.target.items()}
                if self.p('stage')=='detect':
                    self.event('DETECTION_VERIFIED');success=True;return True
                self.event('PLAN');self.wait(self.costmap_contains_target,8.,'COSTMAP_MISSING_CAN')
                # Small upright cylinder observed from south: surface-to-centre
                # offset uses known geometry, never the spawn pose.
                surface=np.array(self.target['center'])
                viewing_direction=surface-np.array(self.stop_pose[:2])
                viewing_direction/=max(1e-9,np.linalg.norm(viewing_direction))
                center=surface+self.p('can_radius')*viewing_direction
                work=(float(center[0]-self.p('work_left_offset')),float(center[1]-self.p('work_standoff')))
                extra=[p for cluster in self.observation['other_clusters'] for p in cluster['points']]
                self.clear_points=[];rejections=[]
                # A small observation/raster shift can put the parked can
                # inside the base's departure clearance. Check both phases at
                # the nominal working pose and a bounded 1.5 cm backoff.
                for backoff in (0.,.015):
                    candidate=(work[0],work[1]-backoff)
                    try:
                        # Finish the intersection turn on its centerline before
                        # shifting toward the arm bay. Every actual waypoint
                        # and its connecting path is included in the cost.
                        waypoints=[(self.p('observation_x'),self.p('observation_y')),
                                   (float(center[0]),candidate[1]-.08),candidate]
                        approach=[];previous=self.stop_pose[:2]
                        for waypoint in waypoints:
                            segment=self.grid.astar(previous,waypoint,self.p('robot_radius'),extra_points=extra+list(self.target['points']))
                            approach+=segment if not approach else segment[1:]
                            previous=waypoint
                        onward=self.grid.astar(candidate,(self.p('goal_x'),self.p('goal_y')),
                                               self.p('robot_radius'),extra_points=extra+self.placed_obstacle())
                        self.clear_points=approach+onward[1:];work=candidate
                        self.report['work_backoff_m']=backoff
                        self.report['work_approach_waypoints']=waypoints
                        self.report['departure_mode']='current_pose_to_goal_continuous_replanning'
                        break
                    except PlanError as exc:
                        if exc.code not in ('NO_PATH','INVALID_START','INVALID_GOAL'):raise
                        rejections.append(str(exc))
                if not self.clear_points:
                    self.report['remove_path_unavailable']=rejections
                    self.get_logger().warn('Remove candidate unavailable: '+str(rejections))
                self.detour_points=self.plan_detour() or []
                self.wait(lambda:isinstance(self.arm_status.get('nominal_remove_seconds'),(int,float)),10.,'ARM_TIME_ESTIMATE_UNAVAILABLE')
                motion=float(self.arm_status['nominal_remove_seconds'])
                preparation=float(self.arm_status.get('nominal_prepare_seconds',0.))
                if preparation<=0:raise RuntimeError('ARM_PREPARATION_TIME_UNAVAILABLE')
                if not math.isfinite(motion) or motion<=0:raise RuntimeError('INVALID_ARM_TIME_ESTIMATE')
                lc=path_length(self.clear_points) if self.clear_points else None
                ld=path_length(self.detour_points) if self.detour_points else None
                precision_goals=len(self.report.get('work_approach_waypoints',[]))
                precision_seconds=precision_goals*self.p('precision_goal_seconds')
                tr=lc/self.p('v_eff')+precision_seconds+motion+preparation+self.p('alignment_estimate') if lc is not None else None
                td=ld/self.p('v_eff')+self.p('switch_seconds') if ld is not None else None
                self.choice='REMOVE' if self.p('removable') and tr is not None and (td is None or tr+self.p('decision_margin')<td) else 'DETOUR'
                if self.choice=='DETOUR' and td is None:raise RuntimeError('NO_FEASIBLE_ACTION')
                self.report.update(work_goal=list(work),work_heading_deg=self.p('work_heading_deg'),place_goal=[self.p('place_x'),self.p('place_y'),self.p('place_z')],
                                   L_clear=lc,L_detour=ld,T_remove=tr,T_detour=td,estimated_arm_seconds=motion,
                                   estimated_preparation_seconds=preparation,estimated_precision_seconds=precision_seconds,
                                   precision_goal_count=precision_goals,precision_goal_basis=self.p('precision_goal_basis'),
                                   decision_reason=('우회 경로 없음, 제거 후보 실행' if td is None else '집기 후보 예상시간 + 여유 < 우회 예상시간') if self.choice=='REMOVE'
                                       else ('제거 불가 설정, 우회 선택' if not self.p('removable') else '작업 접근 또는 적치 후 경로 없음, 우회 선택' if tr is None else '우회 예상시간이 더 짧거나 차이가 판단 여유 이내'),
                                   removability_basis='single known can; camera and grasp feasibility confirmed after approach')
                self.publish_path(self.clear_pub,self.clear_points);self.publish_path(self.detour_pub,self.detour_points)
                self.times.update(decision_sim=self.sim_now(),decision_wall=time.monotonic())
                self.event('DECISION',choice=self.choice,T_remove=tr,T_detour=td,arm_estimate=motion)
                if self.p('stage')=='plan':success=True;return True
                if self.choice=='REMOVE':
                    self.event('WORK_APPROACH');t=self.sim_now()
                    self.set_goal_precision(.012)
                    for waypoint in self.report['work_approach_waypoints'][:-1]:
                        self.navigate(waypoint)
                    work=self.refine_work_goal(work)
                    self.report['work_approach_waypoints'][-1]=list(work)
                    self.navigate(work,heading=math.radians(self.p('work_heading_deg')))
                    self.set_goal_precision(.035)
                    work_error=math.dist(self.pose()[:2],work)
                    self.report['work_position_error']=work_error
                    if work_error>.025:raise RuntimeError('WORK_POSITION_ERROR: precise approach not confirmed')
                    self.report['work_approach_sim_seconds']=self.sim_now()-t
                    self.event('PREPARE');self.arm_operation('PREPARE','PREPARED')
                    self.recognize_can()
                    self.report['camera_observation']=dict(self.can_observation)
                    self.pick_pub.publish(self.can_pose)
                    # Allow transient-local target delivery before the request.
                    sent=time.monotonic();self.wait(lambda:time.monotonic()-sent>.3,2.,'TARGET_DELIVERY_TIMEOUT')
                    self.event('PHYSICAL_REMOVE');self.arm_operation('REMOVE','SUCCEEDED')
                    self.stop_pose=self.pose();self.removal_ack_sim=self.sim_now();self.clear_streak=0
                    self.event('VERIFY_CLEAR');after=self.costmap_time
                    self.wait(lambda:self.clear_streak>=3 and self.costmap_time>after
                              and self.costmap_stamp>self.removal_ack_sim and self.costmap_clear(),
                              self.p('clear_timeout'),'CLEARANCE_UNCONFIRMED')
                    fresh=self.plan_detour(check_target=False)
                    if not fresh or path_length(fresh)>path_length(onward)+.6:raise RuntimeError('SHORT_PASSAGE_NOT_VERIFIED')
                    self.report['verified_clear_nav2_length']=path_length(fresh)
                    self.event('GOAL_AFTER_REMOVE')
                    self.report['departure_start_pose']=list(self.pose())
                    self.report['departure_goal']=[self.p('goal_x'),self.p('goal_y')]
                    self.report['departure_intermediate_goal_count']=0
                else:
                    self.event('DETOUR')
                    switch_end=self.sim_now()+self.p('switch_seconds')
                    self.wait(lambda:self.sim_now()>=switch_end,self.p('switch_seconds')*3+2.,'SWITCH_TIMEOUT')
                self.navigate((self.p('goal_x'),self.p('goal_y')))
            final=self.pose();error=math.dist(final[:2],(self.p('goal_x'),self.p('goal_y')))
            if error>.15:raise RuntimeError('FINAL_POSITION_ERROR')
            self.report.update(final_pose=list(final),final_position_error=error)
            self.times.update(arrival_sim=self.sim_now(),arrival_wall=time.monotonic())
            success=True;self.event('DONE',choice=self.choice,position_error=error)
        except Exception as exc:
            self.failure=str(exc);self.enabled=False;self.drive_pub.publish(Bool(data=False))
            self.report['last_camera_observation']=dict(self.can_observation)
            if self.failure=='FRESH_CAN_RECOGNITION_FAILED':
                self.failure+=': '+self.can_observation.get('reason','관측 수신 없음')
            if self.active_handle is not None:self.active_handle.cancel_goal_async()
            self.event('FAILED',reason=self.failure)
        finally:
            self.enabled=False;self.drive_pub.publish(Bool(data=False));self.finish(success)
            end=time.monotonic()+2.
            while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
        return success

    def finish(self,success):
        sim=self.sim_now();wall=time.monotonic()
        self.report.update(success=success,choice=self.choice,failure_reason=self.failure,
            execution_completed=success and self.p('stage')=='full',scans=self.scans,
            total_sim_seconds=None if self.started_sim is None or self.clock_fault else sim-self.started_sim,
            total_wall_seconds=None if self.mission_started_wall is None else wall-self.mission_started_wall,
            selected_execution_sim_seconds=None if 'decision_sim' not in self.times or self.clock_fault or self.p('stage')!='full' else self.times.get('arrival_sim',sim)-self.times['decision_sim'],
            selected_execution_wall_seconds=None if 'decision_wall' not in self.times or self.p('stage')!='full' else self.times.get('arrival_wall',wall)-self.times['decision_wall'],
            clear_scan_streak=self.clear_streak,v_eff=self.p('v_eff'),v_eff_basis=self.p('v_eff_basis'),decision_margin=self.p('decision_margin'))
        (self.output/'result.json').write_text(json.dumps(self.report,indent=2,ensure_ascii=False,default=str))
        with open(self.output/'result.csv','w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(self.report));w.writeheader();w.writerow(self.report)
        self.events_file.close();self.trajectory_file.flush()


def main(args=None):
    rclpy.init(args=args);node=PhysicalMission()
    try:ok=node.run()
    finally:
        node.listener.executor.shutdown()
        node.listener.dedicated_listener_thread.join(timeout=2.)
        node.listener.unregister();node.tf_node.destroy_node()
        node.trajectory_file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
