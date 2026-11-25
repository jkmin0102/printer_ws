"""Contact-based, force-controlled simulated OpenMANIPULATOR-X pick and place.

No entity teleport, attachment, or simulator ground-truth pose enters execution.
The can target is a fresh RGB-D observation supplied by the mission. Explicit
fixture targets are allowed only for the separate stationary arm calibration
experiment and are labeled in every status/report.
"""
import json
import math
import os
from pathlib import Path
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.ndimage import distance_transform_edt

from .arm_kinematics import (ArmPlanError, JOINT_NAMES, RECOGNITION, STOW,
    cartesian_plan, forward, inverse, joint_plan, validate_configuration)


DURABLE = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def quaternion_matrix(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    length=math.sqrt(x*x+y*y+z*z+w*w)
    if length<1e-9: raise ArmPlanError('Invalid zero quaternion')
    x,y,z,w=x/length,y/length,z/length,w/length
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class PhysicalRemover(Node):
    def __init__(self):
        super().__init__('physical_remover')
        defaults={
            'pick_topic':'/namo/pick_pose', 'place_frame':'base_link',
            'recognition_joints_deg':[0.,-20.,-30.,115.],
            'place_xyz':[.12,.33,.0575], 'tool_pitch_deg':45.,
            'grasp_height_offset':.01, 'approach_height':.05, 'lift_height':.05,
            'release_gap':.002, 'joint_velocity':.45, 'cartesian_speed':.025,
            'gripper_open':.019, 'gripper_close':-.009, 'gripper_effort':1.,
            'gripper_expected_seconds':2., 'grasp_settle_seconds':.35,
            'release_settle_seconds':.7, 'floor_z':-.01,
            'observation_max_age':2., 'action_timeout_margin':6.,
            'target_contact_name':'namo_can', 'require_collision_map':False,
            'wall_height':1., 'collision_boxes_base':'[]',
            'max_base_drift':.01, 'max_base_yaw_drift':.03,
            'allow_fixture_target':False, 'fixture_frame':'base_link', 'fixture_pick_xyz':[.30,0.,.0575],
            'nominal_pick_xyz':[.30,0.,.0575], 'nominal_place_xyz':[.12,.33,.0575],
        }
        for name,value in defaults.items():
            self.declare_parameter(name,value)
        self.p=lambda key:self.get_parameter(key).value
        self.pitch=math.radians(self.p('tool_pitch_deg'))
        self.recognition=np.deg2rad(self.p('recognition_joints_deg'))
        validate_configuration(self.recognition)
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.arm=ActionClient(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        self.gripper=ActionClient(self,GripperCommand,'/gripper_controller/gripper_cmd')
        self.right_gripper=ActionClient(self,GripperCommand,'/right_gripper_controller/gripper_cmd')
        self.status_pub=self.create_publisher(String,'/namo/physical_removal_status',DURABLE)
        self.lock_pub=self.create_publisher(Bool,'/namo/manipulation_lock',10)
        # Control authorization needs the newest measurement; queued reliable
        # 500 Hz states can otherwise age behind the live simulation clock.
        # Planning services and the action state machine remain mutually exclusive.
        # Sensor callbacks have their own executor group so collision/IK planning
        # cannot starve the stopped-base lease or joint/contact measurements.
        self.sensor_group=MutuallyExclusiveCallbackGroup()
        latest_sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState,'/joint_states',self.on_joints,latest_sensor,callback_group=self.sensor_group)
        self.create_subscription(Odometry,'/odom',self.on_odom,10,callback_group=self.sensor_group)
        self.create_subscription(Bool,'/namo/drive_enabled',self.on_gate,10,callback_group=self.sensor_group)
        self.create_subscription(PoseStamped,self.p('pick_topic'),self.on_pick,10,callback_group=self.sensor_group)
        self.create_subscription(String,'/namo/can_observation',self.on_observation,DURABLE,callback_group=self.sensor_group)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,DURABLE,callback_group=self.sensor_group)
        self.create_subscription(ContactsState,'/namo/left_finger_contacts',lambda m:self.on_contact('left',m),10,callback_group=self.sensor_group)
        self.create_subscription(ContactsState,'/namo/right_finger_contacts',lambda m:self.on_contact('right',m),10,callback_group=self.sensor_group)
        self.create_service(Trigger,'/namo/physical_remove',lambda q,r:self.request('REMOVE',r))
        self.create_service(Trigger,'/namo/prepare_grasp',lambda q,r:self.request('PREPARE',r))
        self.create_service(Trigger,'/namo/adjust_grasp_view',lambda q,r:self.request('VIEW',r))
        self.create_service(Trigger,'/namo/stow_arm',lambda q,r:self.request('STOW',r))
        self.create_service(Trigger,'/namo/plan_remove',self.preview)
        self.joints={}
        self.joint_receipts={}
        self.joint_stamps={}
        self.measured_velocity={}
        self.joint_received=self.odom_received=self.gate_received=-math.inf
        self.odom_speed=(math.inf,math.inf)
        self.odom_pose=None
        self.odom_stamp=-math.inf
        self.base_start_pose=None
        self.drive_enabled=True
        self.target_pose=None
        self.observation={}
        self.observations=deque(maxlen=16)
        self.view_index=0
        self.contacts={side:dict(active=False,received=-math.inf,stamp=-math.inf,pairs=[]) for side in ('left','right')}
        self.static_map=None
        self.busy=False
        self.state='IDLE'
        self.operation=''
        self.phase=''
        self.reason='Awaiting stopped base and observed target'
        self.source=''
        self.stages=[]
        self.stage_index=0
        self.stage_started=False
        self.goal_future=self.active_handle=self.result_future=None
        self.right_future=self.right_handle=self.right_result=None
        self.deadline=0.
        self.settle_until=0.
        self.estimated=0.
        self.operation_start_sim=self.operation_start_wall=None
        self.operation_end_sim=self.operation_end_wall=None
        self.last_sim=0.
        self.last_clock_wall=time.monotonic()
        self.contact_lost_since=None
        self.holding=False
        self.evidence=[]
        self.report={}
        self.nominal_estimate=None
        self.nominal_stages=[]
        self.nominal_prepare=self.p('gripper_expected_seconds')+.3+joint_plan(STOW,self.recognition,self.p('joint_velocity'),validator=validate_configuration).duration+.15
        try:
            nominal=self.build_remove_plan(self.recognition,self.p('nominal_pick_xyz'),self.p('nominal_place_xyz'),nominal=True)
            self.nominal_estimate=sum(s['estimate'] for s in nominal)
            self.nominal_stages=[{'phase':s['name'],'kind':s['kind'],'estimated_seconds':s['estimate']} for s in nominal]
        except ArmPlanError as exc:
            self.reason='Nominal preview unavailable: '+str(exc)
        folder=Path(os.environ.get('NAMO_RUN_DIR','/tmp/namo_manipulation'))
        folder.mkdir(parents=True,exist_ok=True)
        self.folder=folder
        self.events=open(folder/'arm_events.jsonl','a',buffering=1)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def sim_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_joints(self,msg):
        now=time.monotonic()
        stamp=stamp_seconds(msg.header.stamp)
        for index,(name,position) in enumerate(zip(msg.name,msg.position)):
            if not math.isfinite(position): continue
            self.joints[name]=position
            self.joint_receipts[name]=now
            self.joint_stamps[name]=stamp
            if index<len(msg.velocity): self.measured_velocity[name]=msg.velocity[index]
        required=(*JOINT_NAMES,'gripper_left_joint','gripper_right_joint')
        self.joint_received=min((self.joint_receipts.get(j,-math.inf) for j in required))

    def on_odom(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        if stamp<=self.odom_stamp: return
        self.odom_stamp=stamp
        v=msg.twist.twist
        self.odom_speed=(math.hypot(v.linear.x,v.linear.y),abs(v.angular.z))
        self.odom_received=time.monotonic()
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        self.odom_pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def on_gate(self,msg):
        self.drive_enabled=msg.data
        self.gate_received=time.monotonic()

    def on_pick(self,msg):
        self.target_pose=msg

    def on_observation(self,msg):
        try: self.observation=json.loads(msg.data)
        except (ValueError,TypeError): self.observation={'valid':False,'stable':False}
        self.observations.append(self.observation)

    def on_contact(self,side,msg):
        needle=self.p('target_contact_name')
        pairs=[(s.collision1_name,s.collision2_name) for s in msg.states
               if needle in s.collision1_name or needle in s.collision2_name]
        self.contacts[side]=dict(active=bool(pairs),received=time.monotonic(),
                                 stamp=stamp_seconds(msg.header.stamp),pairs=pairs)

    def on_map(self,msg):
        data=np.array(msg.data).reshape(msg.info.height,msg.info.width)
        blocked=(data<0)|(data>=65)
        distance=distance_transform_edt(np.pad(~blocked,1,constant_values=False))[1:-1,1:-1]*msg.info.resolution
        self.static_map=(msg,distance)

    def both_contacts(self):
        return all(v['active'] and time.monotonic()-v['received']<.6
                   and self.sim_now()-v['stamp']<.6 for v in self.contacts.values())

    def contacts_clear(self):
        return all(not v['active'] and time.monotonic()-v['received']<.6 for v in self.contacts.values())

    def unsafe(self):
        now=time.monotonic()
        if now-self.gate_received>.75 or self.drive_enabled:
            return 'Drive gate must be closed with a fresh heartbeat'
        if now-self.odom_received>.75 or self.sim_now()-self.odom_stamp>.75 or not all(math.isfinite(v) for v in self.odom_speed):
            return 'Fresh finite odometry required'
        if self.odom_speed[0]>=.01 or self.odom_speed[1]>=.02:
            return f'Base is moving: linear/angular={self.odom_speed}'
        if self.busy and self.base_start_pose is not None and self.odom_pose is not None:
            distance=math.hypot(self.odom_pose[0]-self.base_start_pose[0],self.odom_pose[1]-self.base_start_pose[1])
            heading=abs(math.atan2(math.sin(self.odom_pose[2]-self.base_start_pose[2]),math.cos(self.odom_pose[2]-self.base_start_pose[2])))
            if distance>self.p('max_base_drift') or heading>self.p('max_base_yaw_drift'):
                return f'Base drift during arm operation: {distance:.4f}m, {heading:.4f}rad'
        required=(*JOINT_NAMES,'gripper_left_joint','gripper_right_joint')
        if any(now-self.joint_receipts.get(j,-math.inf)>.75 or self.sim_now()-self.joint_stamps.get(j,-math.inf)>.75 for j in required):
            return 'Fresh individual arm and both finger joint states required'
        return ''

    def target_in_base(self,pose):
        xyz=np.array([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z])
        if not np.isfinite(xyz).all(): raise ArmPlanError('Non-finite target')
        if pose.header.frame_id=='base_link': return xyz
        tr=self.tf.lookup_transform('base_link',pose.header.frame_id,
                                     rclpy.time.Time.from_msg(pose.header.stamp))
        t=tr.transform.translation
        return quaternion_matrix(tr.transform.rotation)@xyz+np.array([t.x,t.y,t.z])

    def observed_pick(self):
        if self.p('allow_fixture_target'):
            self.source='EXPLICIT_FIXTURE_CALIBRATION_ONLY'
            pose=PoseStamped()
            pose.header.frame_id=self.p('fixture_frame')
            pose.header.stamp=rclpy.time.Time().to_msg()
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,self.p('fixture_pick_xyz'))
            pose.pose.orientation.w=1.
            return self.target_in_base(pose)
        pose=self.target_pose
        if pose is None: raise ArmPlanError('No mission-supplied observed pick pose')
        age=self.sim_now()-stamp_seconds(pose.header.stamp)
        if age<-.1 or age>self.p('observation_max_age'):
            raise ArmPlanError(f'Observed pick pose stale: {age:.3f}s')
        key=(pose.header.stamp.sec,pose.header.stamp.nanosec)
        # A newer inference may arrive during the mission's target delivery.
        # Validate the chosen image's metadata, never a different latest image.
        observation=next((o for o in reversed(tuple(self.observations))
                          if (o.get('stamp',{}).get('sec'),o.get('stamp',{}).get('nanosec'))==key),None)
        if observation is None:
            raise ArmPlanError('Exact pose/observation timestamp pair unavailable')
        if not observation.get('valid') or not observation.get('stable'):
            raise ArmPlanError('RGB-D observation is invalid or not stable')
        p=pose.pose.position
        try: coordinates=np.asarray(observation.get('center_xyz',[]),dtype=float)
        except (TypeError,ValueError) as exc:raise ArmPlanError('Invalid observation coordinates') from exc
        if (observation.get('frame_id')!=pose.header.frame_id
                or coordinates.shape!=(3,)
                or not np.allclose(coordinates,[p.x,p.y,p.z],atol=1e-8,rtol=0)):
            raise ArmPlanError('Pose/observation coordinates do not agree')
        self.source='RGBD_'+str(observation.get('mode','UNKNOWN'))
        return self.target_in_base(pose)

    def configured_place(self):
        p=PoseStamped()
        p.header.frame_id=self.p('place_frame')
        # Place is a fixed world coordinate; use latest stopped-base transform.
        p.header.stamp=rclpy.time.Time().to_msg()
        p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,self.p('place_xyz'))
        p.pose.orientation.w=1.
        return self.target_in_base(p)

    def validator(self,nominal=False):
        flat=list(json.loads(self.p('collision_boxes_base')))
        if len(flat)%6: raise ArmPlanError('collision_boxes_base must have 6 values per AABB')
        boxes=[flat[i:i+6] for i in range(0,len(flat),6)]
        check=None
        if not nominal and self.static_map is not None:
            msg,distance=self.static_map
            tr=self.tf.lookup_transform(msg.header.frame_id,'base_link',rclpy.time.Time())
            r=quaternion_matrix(tr.transform.rotation)
            t=tr.transform.translation
            translation=np.array([t.x,t.y,t.z])
            origin=msg.info.origin
            mr=quaternion_matrix(origin.orientation).T
            mo=np.array([origin.position.x,origin.position.y,origin.position.z])
            def check(point,radius):
                world=r@point+translation
                if world[2]-radius>self.p('wall_height'): return True
                local=mr@(world-mo)
                x,y=math.floor(local[0]/msg.info.resolution),math.floor(local[1]/msg.info.resolution)
                return (0<=x<msg.info.width and 0<=y<msg.info.height
                        and distance[y,x]>radius+msg.info.resolution/math.sqrt(2))
        elif not nominal and self.p('require_collision_map'):
            raise ArmPlanError('Static collision map unavailable')
        return lambda q:validate_configuration(q,floor_z=self.p('floor_z'),obstacles=boxes,
                                               environment_check=check)

    def motion_stage(self,name,plan,holding=False):
        return dict(name=name,kind='arm',plan=plan,estimate=plan.duration+.15,holding=holding)

    def gripper_stage(self,name,position,check=''):
        return dict(name=name,kind='gripper',position=position,
                    estimate=self.p('gripper_expected_seconds'),check=check)

    def settle_stage(self,name,seconds,check=''):
        return dict(name=name,kind='settle',seconds=seconds,estimate=seconds,check=check)

    def build_remove_plan(self,current,pick,place,nominal=False):
        validate=self.validator(nominal)
        pick=np.asarray(pick,dtype=float)+[0.,0.,self.p('grasp_height_offset')]
        place=np.asarray(place,dtype=float)+[0.,0.,self.p('grasp_height_offset')+self.p('release_gap')]
        pre=pick+[0.,0.,self.p('approach_height')]
        lifted=pick+[0.,0.,self.p('lift_height')]
        placed_high=place+[0.,0.,self.p('lift_height')]
        try:
            qpre=inverse(pre,self.pitch)
            initial_plan=joint_plan(current,qpre,self.p('joint_velocity'),validator=validate)
        except ArmPlanError as exc:
            raise ArmPlanError('PREGRASP: '+str(exc)) from exc
        stages=[self.gripper_stage('OPEN',self.p('gripper_open')),
                self.motion_stage('PREGRASP',initial_plan)]
        q=qpre
        def line(name,xyz,holding=False):
            nonlocal q
            try:
                plan=cartesian_plan(q,xyz,self.pitch,self.p('cartesian_speed'),self.p('joint_velocity'),validate)
            except ArmPlanError as exc:
                raise ArmPlanError(name+': '+str(exc)) from exc
            stages.append(self.motion_stage(name,plan,holding))
            q=np.array(plan.positions[-1])
        line('APPROACH_CAN',pick)
        stages.append(self.gripper_stage('CLOSE',self.p('gripper_close'),'pinch'))
        stages.append(self.settle_stage('VERIFY_PINCH',self.p('grasp_settle_seconds'),'pinch'))
        line('LIFT',lifted,True)
        stages.append(self.settle_stage('VERIFY_HOLD',self.p('grasp_settle_seconds'),'hold'))
        line('TRANSFER',placed_high,True)
        line('LOWER',place,True)
        stages.append(self.gripper_stage('RELEASE',self.p('gripper_open')))
        stages.append(self.settle_stage('PLACE_SETTLE',self.p('release_settle_seconds')))
        # An open pad can still touch the upright can until the hand withdraws.
        # Retract along negative tool X (back and up), then require actual
        # contact separation before folding or allowing the base to move.
        retreat=place-forward(q)['rotation'][:,0]*.08
        line('RETREAT',retreat)
        stages.append(self.settle_stage('VERIFY_RELEASE',.25,'release'))
        stages.append(self.motion_stage('STOW',joint_plan(q,STOW,self.p('joint_velocity'),validator=validate)))
        return stages

    def preview(self,request,response):
        response.success=self.nominal_estimate is not None
        response.message=json.dumps(dict(estimated_motion_seconds=self.nominal_estimate,
            estimate_basis='nominal_trajectory_and_settle_sum',
            nominal_pick_xyz=self.p('nominal_pick_xyz'),nominal_place_xyz=self.p('nominal_place_xyz'),
            stage_estimates=self.nominal_stages,gripper_timing='Configured response estimate; no artificial wait substituted for grasp',
            note='Geometry and success are assumptions; execution replans from fresh RGB-D target'))
        return response

    def request(self,operation,response):
        if self.state=='FAILED':
            response.success=False;response.message='Failure lock is latched; restart the isolated simulation';return response
        if self.busy:
            response.success=False;response.message='Another arm operation is active';return response
        if operation=='VIEW' and (self.state!='PREPARED' or self.holding or self.view_index>=2):
            response.success=False;response.message='View adjustment requires PREPARED, empty gripper and remaining attempts';return response
        reason=self.unsafe()
        if reason:
            response.success=False;response.message=reason;return response
        if not self.arm.server_is_ready() or not self.gripper.server_is_ready() or not self.right_gripper.server_is_ready():
            response.success=False;response.message='Arm/gripper action servers unavailable';return response
        request_info=dict(operation=operation)
        try:
            current=np.array([self.joints[j] for j in JOINT_NAMES])
            request_info=dict(operation=operation,current_joints=current.tolist(),safety=self.safety_snapshot())
            if operation=='REMOVE':
                pick,place=self.observed_pick(),self.configured_place()
                request_info.update(pick_xyz_base=pick.tolist(),place_xyz_base=place.tolist(),target_source=self.source)
                self.stages=self.build_remove_plan(current,pick,place)
                self.report=dict(pick_xyz_base=pick.tolist(),place_xyz_base=place.tolist(),target_source=self.source)
            else:
                if operation=='PREPARE':self.view_index=0
                goal=self.recognition.copy() if operation in ('PREPARE','VIEW') else STOW
                if operation=='VIEW':
                    # A physical viewpoint change, with the same swept collision
                    # checks as PREPARE. Never accept a low-confidence box instead.
                    goal[0]+=math.radians((-5.,5.)[self.view_index])
                self.stages=[self.gripper_stage('OPEN',self.p('gripper_open')),
                             self.motion_stage(operation,joint_plan(current,goal,self.p('joint_velocity'),validator=self.validator())),
                             self.settle_stage('SETTLE',.3)]
                self.source='JOINT_POSE_COMMAND'
                self.report=dict(view_index=self.view_index+(operation=='VIEW'),view_joints=goal.tolist())
        except Exception as exc:
            request_info.update(reason=str(exc),physical_success=False)
            (self.folder/'arm_request_rejected.json').write_text(json.dumps(request_info,indent=2))
            response.success=False;response.message='Trajectory rejected: '+str(exc);return response
        if operation=='VIEW':self.view_index+=1
        self.operation=operation
        self.phase=''
        self.operation_end_sim=self.operation_end_wall=None
        self.state='BUSY';self.reason='Physical joint trajectory execution'
        self.busy=True;self.holding=False
        self.base_start_pose=self.odom_pose
        self.stage_index=0;self.stage_started=False
        self.goal_future=self.active_handle=self.result_future=None
        self.right_future=self.right_handle=self.right_result=None
        self.operation_start_sim=self.sim_now();self.operation_start_wall=time.monotonic()
        self.last_sim=self.sim_now();self.last_clock_wall=time.monotonic()
        self.estimated=sum(s['estimate'] for s in self.stages)
        self.report['stage_estimates']=[{'phase':s['name'],'kind':s['kind'],'estimated_seconds':s['estimate']} for s in self.stages]
        self.report['gripper_timing']='Configured response estimate; actual controller result determines completion'
        self.evidence=[]
        self.publish(event=True)
        response.success=True
        response.message=json.dumps(dict(accepted=True,operation=operation,
            estimated_motion_seconds=self.estimated,estimate_basis='arm_trajectory_plus_gripper_response_estimate_and_settle'))
        return response

    def check_evidence(self,kind):
        if kind in ('pinch','hold'):
            gap=[self.joints.get(j,float('nan')) for j in ('gripper_left_joint','gripper_right_joint')]
            if not self.both_contacts(): raise RuntimeError(f'{kind}: two-sided can contact unconfirmed')
            if not all(math.isfinite(v) and v>-.006 for v in gap):
                raise RuntimeError(f'{kind}: fingers fully closed or state unavailable: {gap}')
            self.holding=True
        elif kind=='release':
            if not self.contacts_clear(): raise RuntimeError('release: finger/can separation unconfirmed')
            self.holding=False
        q=[self.joints[j] for j in JOINT_NAMES]
        item=dict(check=kind,sim_seconds=self.sim_now(),tool_xyz_base=forward(q)['tip'].tolist(),
                  joint_positions=q,contacts=self.contacts.copy())
        self.evidence.append(item)
        self.publish(event=True,extra={'evidence':item})

    def begin_stage(self,stage):
        self.phase=stage['name']
        self.stage_started=True
        self.goal_future=self.active_handle=self.result_future=None
        self.right_future=self.right_handle=self.right_result=None
        self.contact_lost_since=None
        if stage['kind']=='settle':
            self.settle_until=self.sim_now()+stage['seconds']
            self.deadline=time.monotonic()+stage['seconds']*5+5
        elif stage['kind']=='arm':
            goal=FollowJointTrajectory.Goal()
            goal.trajectory.joint_names=list(JOINT_NAMES)
            for q,t in zip(stage['plan'].positions,stage['plan'].times):
                point=JointTrajectoryPoint()
                point.positions=[float(x) for x in q]
                point.time_from_start=Duration(sec=int(t),nanosec=int((t-int(t))*1e9))
                goal.trajectory.points.append(point)
            goal.goal_time_tolerance=Duration(sec=3)
            self.goal_future=self.arm.send_goal_async(goal)
            self.deadline=time.monotonic()+stage['plan'].duration*4+self.p('action_timeout_margin')
        else:
            goal=GripperCommand.Goal()
            goal.command.position=stage['position'];goal.command.max_effort=self.p('gripper_effort')
            self.goal_future=self.gripper.send_goal_async(goal)
            self.right_future=self.right_gripper.send_goal_async(goal)
            self.deadline=time.monotonic()+self.p('gripper_expected_seconds')*4+self.p('action_timeout_margin')
            if self.phase=='RELEASE': self.holding=False
        self.publish(event=True)

    def advance(self):
        self.stage_index+=1
        self.stage_started=False
        if self.stage_index==len(self.stages):
            self.busy=False
            self.operation_end_sim,self.operation_end_wall=self.sim_now(),time.monotonic()
            self.state={'PREPARE':'PREPARED','VIEW':'PREPARED','STOW':'STOWED','REMOVE':'SUCCEEDED'}[self.operation]
            self.reason='Physical motion completed; '+('lift contact and release separation verified' if self.operation=='REMOVE' else 'measured joints reached pose')
            self.write_report()
            self.publish(event=True)

    def fail(self,reason):
        self.report['failure_safety_snapshot']=self.safety_snapshot()
        if self.active_handle is not None:
            self.active_handle.cancel_goal_async()
        elif self.goal_future is not None and not self.goal_future.done():
            def cancel_late(future):
                try:
                    result=future.result()
                    if result is not None and result.accepted: result.cancel_goal_async()
                except Exception: pass
            self.goal_future.add_done_callback(cancel_late)
        if self.right_handle is not None:
            self.right_handle.cancel_goal_async()
        elif self.right_future is not None and not self.right_future.done():
            def cancel_right_late(future):
                try:
                    result=future.result()
                    if result is not None and result.accepted: result.cancel_goal_async()
                except Exception: pass
            self.right_future.add_done_callback(cancel_right_late)
        self.busy=False;self.state='FAILED';self.reason=reason
        self.operation_end_sim,self.operation_end_wall=self.sim_now(),time.monotonic()
        # Failure keeps the manipulation lock latched. A human/mission reset is
        # needed; never reopen the gripper or move the base as blind recovery.
        self.write_report()
        self.get_logger().error(reason)
        self.publish(event=True)

    def tick(self):
        self.lock_pub.publish(Bool(data=self.busy or self.state=='FAILED'))
        if not self.busy:
            self.publish();return
        sim,wall=self.sim_now(),time.monotonic()
        if sim<self.last_sim-.01:
            self.fail('Simulation clock reset during physical manipulation');return
        if sim>self.last_sim+1e-9: self.last_clock_wall=wall
        self.last_sim=sim
        if wall-self.last_clock_wall>3.:
            self.fail('Simulation clock stalled');return
        reason=self.unsafe()
        if reason: self.fail(reason);return
        try:
            stage=self.stages[self.stage_index]
            if not self.stage_started:
                self.begin_stage(stage);return
            if wall>self.deadline: raise RuntimeError(self.phase+': action/settle timeout')
            if stage.get('holding'):
                if not self.both_contacts():
                    if self.contact_lost_since is None: self.contact_lost_since=wall
                    elif wall-self.contact_lost_since>.6: raise RuntimeError(self.phase+': grasp contact lost')
                else: self.contact_lost_since=None
            if stage['kind']=='settle':
                if sim>=self.settle_until:
                    if stage.get('check'): self.check_evidence(stage['check'])
                    self.advance()
            elif stage['kind']=='gripper':
                for side in ('left','right'):
                    future=self.goal_future if side=='left' else self.right_future
                    handle=self.active_handle if side=='left' else self.right_handle
                    if handle is None and future.done():
                        handle=future.result()
                        if not handle.accepted: raise RuntimeError(self.phase+': '+side+' gripper goal rejected')
                        if side=='left': self.active_handle=handle;self.result_future=handle.get_result_async()
                        else: self.right_handle=handle;self.right_result=handle.get_result_async()
                if self.result_future is not None and self.right_result is not None and self.result_future.done() and self.right_result.done():
                    results=[self.result_future.result(),self.right_result.result()]
                    if stage.get('check')=='pinch':
                        if any(r.status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_ABORTED) for r in results):
                            raise RuntimeError('CLOSE action canceled or unknown result')
                        self.check_evidence('pinch')
                    elif any(r.status!=GoalStatus.STATUS_SUCCEEDED or not r.result.reached_goal for r in results):
                        raise RuntimeError(self.phase+': one or both grippers did not open')
                    self.advance()
            elif self.active_handle is None:
                if self.goal_future.done():
                    self.active_handle=self.goal_future.result()
                    if not self.active_handle.accepted: raise RuntimeError(self.phase+': action rejected')
                    self.result_future=self.active_handle.get_result_async()
            elif self.result_future.done():
                result=self.result_future.result()
                if stage['kind']=='arm':
                    if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:
                        raise RuntimeError(self.phase+f': arm failure status={result.status}, code={result.result.error_code}, {result.result.error_string}')
                    measured=np.array([self.joints[j] for j in JOINT_NAMES])
                    if np.max(np.abs(measured-np.array(stage['plan'].positions[-1])))>.06:
                        raise RuntimeError(self.phase+': measured joints did not reach trajectory endpoint')
                elif stage.get('check')=='pinch':
                    # A force-limited grasp normally stalls before the commanded
                    # empty-close target; bilateral physical contact is decisive.
                    if result.status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_ABORTED):
                        raise RuntimeError('CLOSE action canceled or unknown result')
                    self.check_evidence('pinch')
                elif result.status!=GoalStatus.STATUS_SUCCEEDED or not result.result.reached_goal:
                    raise RuntimeError(self.phase+f': gripper did not reach open position, status={result.status}')
                self.advance()
            self.publish()
        except Exception as exc:
            self.fail(str(exc))

    def stowed_verified(self):
        if not (self.state=='STOWED' or (self.state=='SUCCEEDED' and self.phase=='STOW')):
            return False
        now=time.monotonic()
        if any(now-self.joint_receipts.get(j,-math.inf)>.75 or self.sim_now()-self.joint_stamps.get(j,-math.inf)>.75 for j in JOINT_NAMES):
            return False
        return all(abs(self.joints.get(j,math.inf)-target)<.06 and abs(self.measured_velocity.get(j,math.inf))<.04 for j,target in zip(JOINT_NAMES,STOW))

    def safety_snapshot(self):
        wall,sim=time.monotonic(),self.sim_now()
        def age(value):
            return value if math.isfinite(value) else None
        return dict(sim_seconds=sim,
            joint_positions={j:self.joints.get(j) for j in (*JOINT_NAMES,'gripper_left_joint','gripper_right_joint')},
            joint_wall_ages={j:age(wall-self.joint_receipts.get(j,-math.inf)) for j in (*JOINT_NAMES,'gripper_left_joint','gripper_right_joint')},
            joint_sim_ages={j:age(sim-self.joint_stamps.get(j,-math.inf)) for j in (*JOINT_NAMES,'gripper_left_joint','gripper_right_joint')},
            odom_wall_age=age(wall-self.odom_received),odom_sim_age=age(sim-self.odom_stamp),
            gate_wall_age=age(wall-self.gate_received),odom_speed=list(self.odom_speed),
            odom_pose=self.odom_pose,base_start_pose=self.base_start_pose)

    def status(self):
        return dict(state=self.state,operation=self.operation,phase=self.phase,busy=self.busy,
            reason=self.reason,readiness_reason=self.unsafe(),target_source=self.source,
            estimated_motion_seconds=self.estimated if self.operation else self.nominal_estimate,
            nominal_remove_seconds=self.nominal_estimate,nominal_prepare_seconds=self.nominal_prepare,estimate_basis='arm_trajectory_plus_gripper_response_estimate_and_settle',
            elapsed_sim_seconds=None if self.operation_start_sim is None else (self.operation_end_sim if self.operation_end_sim is not None else self.sim_now())-self.operation_start_sim,
            elapsed_wall_seconds=None if self.operation_start_wall is None else (self.operation_end_wall if self.operation_end_wall is not None else time.monotonic())-self.operation_start_wall,
            bilateral_contact=self.both_contacts(),holding_contact=self.holding,stowed_verified=self.stowed_verified(),
            stow_joint_position_error={j:self.joints.get(j,0.)-float(target) for j,target in zip(JOINT_NAMES,STOW)},
            stow_joint_velocity={j:self.measured_velocity.get(j) for j in JOINT_NAMES},
            remaining_stages=max(0,len(self.stages)-self.stage_index),
            ready_for_operation=(self.state!='FAILED' and not self.busy and not self.unsafe() and self.arm.server_is_ready() and self.gripper.server_is_ready() and self.right_gripper.server_is_ready()),
            physical_simulation=True,attachment_used=False,teleport_used=False)

    def publish(self,event=False,extra=None):
        value=self.status()
        if extra: value.update(extra)
        self.status_pub.publish(String(data=json.dumps(value)))
        if event and hasattr(self,'events'):
            value['sim_seconds']=self.sim_now()
            self.events.write(json.dumps(value)+'\n')
            self.get_logger().info(json.dumps(value))

    def write_report(self):
        self.report.update(self.status(),evidence=self.evidence)
        (self.folder/f'arm_{self.operation.lower()}_result.json').write_text(json.dumps(self.report,indent=2))


def main(args=None):
    # Keep the context alive long enough to deliver action cancellation on
    # Ctrl-C; the default rclpy handler shuts it down before our cleanup runs.
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=PhysicalRemover()
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt:
        if node.busy and rclpy.ok():
            node.fail('Executor interrupted during manipulation')
            deadline=time.monotonic()+.5
            while rclpy.ok() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
    finally:
        if rclpy.ok(): node.lock_pub.publish(Bool(data=True))
        executor.shutdown(timeout_sec=3.)
        node.events.close();node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
