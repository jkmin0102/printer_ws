"""Finite NAMO mission driven by stamped LiDAR, static map, and real Nav2.

All waits run in main, spinning callbacks; no callback sleeps. Only the mock
executor knows the Gazebo model handle. This node never subscribes to truth.
"""
import csv
import json
import math
import os
from pathlib import Path as FilePath
import time
import uuid

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .planning import Grid, PlanError, choose_action, path_length
from .perception import ScanDetector, clearance_evidence


DURABLE = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def grid_from_msg(msg, threshold=65):
    o = msg.info.origin
    return Grid(np.array(msg.data).reshape(msg.info.height, msg.info.width),
                msg.info.resolution, (o.position.x, o.position.y, yaw(o.orientation)), threshold)


def densify(points, step=0.025):
    output = []
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist(a, b)/step))
        output.extend((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(n))
    return output+[points[-1]] if points else []


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        defaults = dict(scenario='A', goal_x=6., goal_y=0., observation_x=0., observation_y=0.,
                        robot_radius=.28, wall_tolerance=.13, min_points=5, persistence=8,
                        removable=True, v_eff=.20, mock_remove_seconds=22., switch_seconds=0.,
                        decision_margin=2., ready_timeout=90., action_timeout=180.,
                        planning_timeout=12., clear_timeout=15., stage='full')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = lambda name: self.get_parameter(name).value
        self.tf = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.remove = self.create_client(Trigger, '/namo/remove')
        self.drive_pub = self.create_publisher(Bool, '/namo/drive_enabled', 10)
        self.state_pub = self.create_publisher(String, '/namo/state', DURABLE)
        self.clear_pub = self.create_publisher(Path, '/namo/clear_path', DURABLE)
        self.detour_pub = self.create_publisher(Path, '/namo/detour_path', DURABLE)
        self.markers = self.create_publisher(MarkerArray, '/namo/markers', DURABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, DURABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, DURABLE)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/namo/removal_status', self.on_removal, DURABLE)
        self.enabled = False
        self.state = 'WAIT_READY'
        self.grid = self.costmap = self.detector = None
        self.reference = []
        self.observation = None
        self.last_points=np.empty((0,2))
        self.pending_scan = None
        self.scan_time = self.odom_time = self.costmap_time = -math.inf
        self.scan_stamp = self.odom_stamp = self.costmap_stamp = -math.inf
        self.removal_ack_sim = math.inf
        self.scans = self.tf_errors = self.clear_streak = 0
        self.odom_speed = (math.inf, math.inf)
        self.target = self.frozen_target = None
        self.detected_sim = self.detected_wall = None
        self.stop_pose = None
        self.clear_points = self.detour_points = []
        self.removal_status = {}
        self.removal_received = -math.inf
        self.active_handle = self.active_result = None
        self.life_future = None
        self.life_active = False
        self.failure = ''
        self.choice = ''
        self.times = {}
        self.started_wall = time.monotonic()
        self.started_sim = None
        self.mission_started_wall = None
        self.last_sim = 0.
        self.clock_fault = False
        self.output = FilePath(os.environ.get('NAMO_RUN_DIR', '/tmp/namo_runs/'+uuid.uuid4().hex[:10]))
        self.output.mkdir(parents=True, exist_ok=True)
        self.run_id = os.environ.get('NAMO_RUN_ID', self.output.name)
        self.events_file = open(self.output/'events.jsonl', 'a', buffering=1)
        self.trajectory_file = open(self.output/'trajectory.csv','w',buffering=1)
        self.trajectory_file.write('sim_seconds,wall_elapsed,x,y,yaw,state,drive_enabled\n')
        self.route_y=[]
        self.report = dict(run_id=self.run_id, scenario=self.p('scenario'), stage=self.p('stage'))
        self.create_timer(.1, self.tick)
        self.event('WAIT_READY')

    def sim_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def event(self, state, **data):
        self.state = state
        value = dict(state=state, sim_seconds=self.sim_now(), wall_elapsed=time.monotonic()-self.started_wall, **data)
        self.events_file.write(json.dumps(value, default=lambda a: np.asarray(a).tolist())+'\n')
        self.state_pub.publish(String(data=json.dumps(value, default=lambda a: np.asarray(a).tolist())))
        self.get_logger().info(json.dumps(value, default=lambda a: np.asarray(a).tolist()))
        self.show_markers()

    def on_map(self, msg):
        if self.grid is None:
            self.grid = grid_from_msg(msg)
            self.detector = ScanDetector(self.grid, robot_radius=self.p('robot_radius'),
                wall_tolerance=self.p('wall_tolerance'), min_points=self.p('min_points'),
                persistence=self.p('persistence'))

    def on_costmap(self, msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp <= self.costmap_stamp:
            return
        self.costmap = msg
        self.costmap_stamp=stamp
        self.costmap_time = time.monotonic()

    def on_odom(self, msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp <= self.odom_stamp:
            return
        self.odom_stamp=stamp
        v = msg.twist.twist
        self.odom_speed = (math.hypot(v.linear.x, v.linear.y), abs(v.angular.z))
        self.odom_time = time.monotonic()

    def on_scan(self, msg):
        self.pending_scan = msg

    def on_removal(self, msg):
        try:
            self.removal_status = json.loads(msg.data)
            self.removal_received = time.monotonic()
        except ValueError:
            self.removal_status = dict(state='FAILED', reason='Malformed removal status')

    def tick(self):
        now = self.sim_now()
        if now+0.05 < self.last_sim:
            self.clock_fault = True
            self.enabled = False
        self.last_sim = now
        self.drive_pub.publish(Bool(data=self.enabled and not self.clock_fault))
        self.process_scan()
        self.show_markers()
        try:
            pos=self.pose()
            self.trajectory_file.write(f'{now},{time.monotonic()-self.started_wall},{pos[0]},{pos[1]},{pos[2]},{self.state},{self.enabled}\n')
            if self.enabled and 'decision_sim' in self.times:
                self.route_y.append(pos[1])
        except Exception:
            pass

    def process_scan(self):
        msg = self.pending_scan
        if msg is None or self.detector is None or not self.reference:
            return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp <= self.scan_stamp or self.sim_now()-stamp > .75:
            self.pending_scan=None
            return
        try:
            tr = self.tf.lookup_transform('map', msg.header.frame_id,
                                          rclpy.time.Time.from_msg(msg.header.stamp))
        except Exception:
            self.tf_errors += 1
            return  # Retry latest scan until timestamped transform arrives.
        self.pending_scan = None
        self.scan_stamp=stamp
        q=tr.transform.rotation
        # Project the full stamped 3D TF rotation into map XY (not a yaw approximation).
        rotation=np.array([[1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.w*q.z)],
                           [2*(q.x*q.y+q.w*q.z),1-2*(q.x*q.x+q.z*q.z)]])
        origin = np.array([tr.transform.translation.x, tr.transform.translation.y])
        ranges = np.array(msg.ranges)
        bearings = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges < msg.range_max)
        points = origin+(np.column_stack((np.cos(bearings[valid]), np.sin(bearings[valid])))*ranges[valid, None])@rotation.T
        rays_valid = valid | np.isposinf(ranges) | (np.isfinite(ranges) & (ranges == msg.range_max))
        ray_ranges = np.where(np.isposinf(ranges), msg.range_max, ranges)[rays_valid]
        endpoints = origin+(np.column_stack((np.cos(bearings[rays_valid]),np.sin(bearings[rays_valid])))*ray_ranges[:, None])@rotation.T
        self.observation = self.detector.update(points, self.reference)
        self.last_points=points
        self.scans += 1
        self.scan_time = time.monotonic()
        if self.observation['target'] is not None:
            if self.target is None:
                self.detected_sim, self.detected_wall = self.sim_now(), time.monotonic()
            self.target = self.observation['target']
        if self.frozen_target is not None and self.state == 'VERIFY_CLEAR' and stamp > self.removal_ack_sim:
            evidence = clearance_evidence(origin, endpoints, self.frozen_target['bounds'])
            self.clear_streak = self.clear_streak+1 if evidence['clear'] else 0

    def pose(self):
        tr = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        if self.sim_now()-(tr.header.stamp.sec+tr.header.stamp.nanosec/1e9)>1.:
            raise RuntimeError('TF_STALE')
        return (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))

    def stamped(self, xy, heading=0.):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(xy[0]), float(xy[1])
        msg.pose.orientation.z = math.sin(heading/2)
        msg.pose.orientation.w = math.cos(heading/2)
        return msg

    def wait(self, predicate, timeout, reason):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.clock_fault:
                raise RuntimeError('CLOCK_RESET: restart whole run; no mixed epochs')
            if self.enabled and (time.monotonic()-self.odom_time>1.5 or time.monotonic()-self.scan_time>2.):
                raise RuntimeError('SENSOR_STALE: closing drive gate')
            if predicate():
                return
        raise RuntimeError(reason)

    def ready(self):
        if self.life_future is not None and self.life_future.done():
            self.life_active = self.life_future.result().current_state.id == 3
            self.life_future = None
        if not self.life_active and self.life_future is None and self.lifecycle.service_is_ready():
            self.life_future = self.lifecycle.call_async(GetState.Request())
        if not (self.grid and self.life_active and self.nav.server_is_ready() and self.planner.server_is_ready()
                and self.costmap is not None and time.monotonic()-self.odom_time<1.):
            return False
        try:
            current = self.pose()
            if not self.reference:
                self.reference = self.grid.astar(current[:2], (self.p('goal_x'),self.p('goal_y')), self.p('robot_radius'))
            return self.scans >= 4
        except Exception:
            return False

    def stopped(self):
        return (time.monotonic()-self.odom_time<.5 and self.sim_now()-self.odom_stamp<.5
                and self.odom_speed[0]<.01 and self.odom_speed[1]<.02)

    def stop(self):
        self.enabled = False
        self.drive_pub.publish(Bool(data=False))
        if self.active_handle is not None and not self.active_result.done():
            canceled = self.active_handle.cancel_goal_async()
            self.wait(canceled.done, 6., 'CANCEL_ACK_TIMEOUT')
            self.wait(self.active_result.done, 6., 'CANCEL_RESULT_TIMEOUT')
        self.active_handle = self.active_result = None
        stable = [None]
        def stationary():
            if not self.stopped(): stable[0] = None
            elif stable[0] is None: stable[0] = time.monotonic()
            return stable[0] is not None and time.monotonic()-stable[0]>.35
        self.wait(stationary, 8., 'ACTUAL_STOP_TIMEOUT')

    def navigate(self, xy, detect=False):
        # AMCL postdates map->odom. Require that transform to cover planning and
        # dispatch, rather than accepting a merely recent latest-common pose.
        self.wait(lambda:self.tf.can_transform('odom','map',self.get_clock().now()+Duration(seconds=.2)),
                  5.,'FRESH_LOCALIZATION_TF_TIMEOUT')
        future = self.nav.send_goal_async(NavigateToPose.Goal(pose=self.stamped(xy)))
        try:
            self.wait(future.done, 10., 'NAV_GOAL_ACK_TIMEOUT')
        except Exception:
            # A late accepted goal is canceled; the gate remains closed.
            future.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result().accepted else None)
            raise
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('NAV_GOAL_REJECTED')
        self.active_handle, self.active_result = handle, handle.get_result_async()
        self.enabled = True
        self.wait(lambda: self.active_result.done() or (detect and self.target is not None),
                  self.p('action_timeout'), 'NAV_EXECUTION_TIMEOUT')
        detected = detect and self.target is not None
        status = self.active_result.result().status if self.active_result.done() else None
        self.stop()
        if not detected and status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'NAV_EXECUTION_FAILED status={status}')
        return detected

    def costmap_contains_target(self):
        if self.target is None or self.costmap is None or time.monotonic()-self.costmap_time>1.:
            return False
        grid = grid_from_msg(self.costmap, 99)
        for point in self.target['points']:
            x, y = grid.world_to_cell(point)
            if grid.in_bounds((x,y)) and grid.data[y,x] >= 99:
                return True
        return False

    def costmap_clear(self):
        if self.costmap is None or time.monotonic()-self.costmap_time>1.:
            return False
        # Verify the observed object region, not the exact counterfactual A* cells.
        # Static-wall scan noise may legitimately move the preferred corner route;
        # a fresh Nav2 plan below separately checks full footprint-safe passage.
        grid = grid_from_msg(self.costmap, 100)
        x0,y0,x1,y1=self.frozen_target['bounds']
        blocked=0
        for x in np.arange(x0-.06,x1+.061,grid.resolution/2):
            for y in np.arange(y0-.06,y1+.061,grid.resolution/2):
                if self.grid.distance_to_static((x,y))>self.p('wall_tolerance') and not grid.is_free((x,y),0.):
                    blocked+=1
        self.report['clearance_remaining_lethal_samples']=blocked
        return blocked==0

    def plan_detour(self, check_target=True):
        if not self.planner.server_is_ready():
            raise RuntimeError('PLANNER_UNAVAILABLE')
        goal = ComputePathToPose.Goal(goal=self.stamped((self.p('goal_x'),self.p('goal_y'))),
                                      start=self.stamped(self.stop_pose[:2], self.stop_pose[2]),
                                      planner_id='GridBased', use_start=True)
        ack = self.planner.send_goal_async(goal)
        self.wait(ack.done, self.p('planning_timeout'), 'PLAN_ACK_TIMEOUT')
        handle = ack.result()
        if not handle.accepted:
            raise RuntimeError('PLAN_REJECTED')
        future = handle.get_result_async()
        try:
            self.wait(future.done, self.p('planning_timeout'), 'PLAN_RESULT_TIMEOUT')
        except Exception:
            handle.cancel_goal_async()
            raise
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            # Humble has no error_code. Independently establish genuine no-path
            # on the same observed inflated grid before allowing REMOVE-only.
            try:
                grid_from_msg(self.costmap,99).astar(self.stop_pose[:2],
                    (self.p('goal_x'),self.p('goal_y')),radius=0.)
            except PlanError as exc:
                if exc.code == 'NO_PATH': return None
            raise RuntimeError(f'PLANNER_ABORTED_OR_ERROR status={result.status}')
        points = [(p.pose.position.x,p.pose.position.y) for p in result.result.path.poses]
        if len(points)<2:
            raise RuntimeError('PLANNER_EMPTY_PATH')
        if math.dist(points[0],self.stop_pose[:2])>.15 or math.dist(points[-1],(self.p('goal_x'),self.p('goal_y')))>.2:
            raise RuntimeError('PLANNER_ENDPOINT_MISMATCH')
        points[0] = self.stop_pose[:2]  # exact same start; bridge <=.15m checked below
        dense = densify(points)
        samples = np.array(self.target['points'])
        for point in dense:
            if check_target and np.min(np.linalg.norm(samples-np.array(point),axis=1)) < self.p('robot_radius'):
                raise RuntimeError('DETOUR_INTERSECTS_OBSERVED_OBSTACLE')
        cm = grid_from_msg(self.costmap,99)
        if not all(cm.is_free(point,0.) for point in dense):
            raise RuntimeError('DETOUR_INTERSECTS_LETHAL_COSTMAP')
        return points

    def publish_path(self, pub, points):
        msg=Path()
        msg.header.frame_id='map'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.poses=[self.stamped(p) for p in points]
        pub.publish(msg)

    def show_markers(self):
        array=MarkerArray()
        text=Marker()
        text.header.frame_id='map'
        text.ns='state'
        text.id=0
        text.type=Marker.TEXT_VIEW_FACING
        text.action=Marker.ADD
        text.pose.position.x=3.
        text.pose.position.y=-1.4
        text.pose.position.z=1.
        text.pose.orientation.w=1.
        text.scale.z=.24
        text.color.r=text.color.g=text.color.b=text.color.a=1.
        costs=''
        if 'T_remove' in self.report:
            fmt=lambda v:'unavailable' if v is None else f'{v:.1f}s'
            costs=f"\nT_remove={fmt(self.report['T_remove'])}\nT_detour={fmt(self.report['T_detour'])}"
        remaining=self.removal_status.get('remaining',0)
        text.text=f'{self.p("scenario")} | {self.state}  {self.choice}{costs}'
        if self.state=='MOCK_REMOVE': text.text+=f'\nMOCK REMOVE (simulation) {remaining:.1f}s'
        if costs:
            text.text+='\nBLUE: hypothetical after removal\nORANGE: obstacle retained'
        array.markers.append(text)
        for index,(label,x,y) in enumerate((('S',0.,-2.),('P',0.,0.),('G',6.,0.),('STASH',-2.,-2.)),10):
            m=Marker()
            m.header.frame_id='map';m.ns='places';m.id=index
            m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
            m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.5
            m.pose.orientation.w=1.;m.scale.z=.25
            m.color.r=m.color.g=m.color.b=m.color.a=1.;m.text=label
            array.markers.append(m)
        for index,(point,color) in enumerate(((self.target['center'] if self.target else None,(1.,0.,0.)),
                                               (self.stop_pose,(0.,1.,0.))),1):
            if point is None: continue
            m=Marker()
            m.header.frame_id='map'
            m.ns='observations'
            m.id=index
            m.type=Marker.SPHERE
            m.action=Marker.ADD
            m.pose.position.x=float(point[0]);m.pose.position.y=float(point[1]);m.pose.position.z=.25
            m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=.18
            m.color.r,m.color.g,m.color.b=color;m.color.a=1.
            array.markers.append(m)
        self.markers.publish(array)

    def run(self):
        success=False
        try:
            self.wait(self.ready,self.p('ready_timeout'),'READINESS_TIMEOUT(map/TF/scan/Nav2 lifecycle/costmap)')
            self.started_sim=self.sim_now()
            self.mission_started_wall=time.monotonic()
            self.event('APPROACH')
            detected=self.navigate((self.p('observation_x'),self.p('observation_y')),detect=True)
            if not detected:
                self.event('OBSERVE')
                start_scans=self.scans
                self.wait(lambda: self.target is not None or self.scans-start_scans>=8,5.,'OBSERVATION_TIMEOUT')
            if self.target is None:
                self.choice='CLEAR'
                self.event('GOAL_CLEAR', scans=self.scans)
                self.navigate((self.p('goal_x'),self.p('goal_y')),detect=True)
                if self.target is None:
                    final=self.pose()
                    if math.dist(final[:2],(self.p('goal_x'),self.p('goal_y')))>.30:
                        raise RuntimeError('GOAL_POSITION_OUTSIDE_TOLERANCE')
                    self.report['final_pose']=list(final)
                    self.times['arrival_sim']=self.sim_now()
                    self.times['arrival_wall']=time.monotonic()
                    success=True
                    self.event('DONE',choice='CLEAR')
                    return True
                # A later real scan may still reveal a blocker.
            detected_sim=self.detected_sim
            detected_wall=self.detected_wall
            self.event('STOP_OBSERVE', observed_center=self.target['center'], points=len(self.target['points']))
            self.stop()
            self.stop_pose=self.pose()
            self.report['stop_pose']=list(self.stop_pose)
            self.report['observed_center']=list(map(float,self.target['center']))
            self.report['stop_sim_seconds']=self.sim_now()-detected_sim
            self.report['stop_wall_seconds']=time.monotonic()-detected_wall
            self.report['approach_until_detection_sim']=detected_sim-self.started_sim
            self.report['approach_until_detection_wall']=detected_wall-self.mission_started_wall
            if self.p('stage')=='detect':
                self.event('DETECTION_VERIFIED', stopped=self.stopped())
                success=True
                return True
            self.event('PLAN')
            plan_sim,plan_wall=self.sim_now(),time.monotonic()
            self.wait(self.costmap_contains_target,6.,'GLOBAL_COSTMAP_MISSING_OBSTACLE')
            if self.p('removable'):
                self.wait(lambda:'mock_remove_seconds' in self.removal_status,5.,'REMOVER_CONFIGURATION_UNAVAILABLE')
                if abs(self.removal_status['mock_remove_seconds']-self.p('mock_remove_seconds'))>1e-6:
                    raise RuntimeError('MOCK_DURATION_MISMATCH(cost vs executor)')
            extra=[p for cluster in self.observation['other_clusters'] for p in cluster['points']]
            try:
                self.clear_points=self.grid.astar(self.stop_pose[:2],(self.p('goal_x'),self.p('goal_y')),
                                                self.p('robot_radius'),extra_points=extra)
            except PlanError as exc:
                if exc.code!='NO_PATH': raise
                self.clear_points=[]
            self.detour_points=self.plan_detour() or []
            clear_length=path_length(self.clear_points) if self.clear_points else None
            detour_length=path_length(self.detour_points) if self.detour_points else None
            decision=choose_action(clear_length,detour_length,removable=self.p('removable'),
                v_eff=self.p('v_eff'),mock_remove=self.p('mock_remove_seconds'),
                switch=self.p('switch_seconds'),margin=self.p('decision_margin'))
            self.choice=decision.action
            self.report.update(L_clear=clear_length,L_detour=detour_length,T_remove=decision.t_remove,
                               T_detour=decision.t_detour,decision_reason=decision.reason,
                               planning_sim_seconds=self.sim_now()-plan_sim,
                               planning_wall_seconds=time.monotonic()-plan_wall,
                               extra_clusters=len(self.observation['other_clusters']))
            self.publish_path(self.clear_pub,self.clear_points)
            self.publish_path(self.detour_pub,self.detour_points)
            self.event('DECISION',choice=self.choice,**{k:self.report[k] for k in ('L_clear','L_detour','T_remove','T_detour','decision_reason')})
            self.times['decision_sim']=self.sim_now()
            self.times['decision_wall']=time.monotonic()
            self.report['detection_to_decision_sim']=self.sim_now()-detected_sim
            self.report['detection_to_decision_wall']=time.monotonic()-detected_wall
            if self.choice=='STOP': raise RuntimeError(decision.reason)
            if self.p('stage')=='plan':
                success=True
                return True
            if self.choice=='REMOVE':
                self.frozen_target={k:(np.array(v).copy() if isinstance(v,(list,tuple,np.ndarray)) else v)
                                    for k,v in self.target.items()}
                self.event('MOCK_REMOVE')
                self.wait(self.remove.service_is_ready,5.,'REMOVE_SERVER_UNAVAILABLE')
                request_wall=time.monotonic()
                ack=self.remove.call_async(Trigger.Request())
                self.wait(ack.done,5.,'REMOVE_ACK_TIMEOUT')
                if not ack.result().success: raise RuntimeError('REMOVE_REJECTED: '+ack.result().message)
                self.wait(lambda:self.removal_received>request_wall and self.removal_status.get('state') in ('SUCCEEDED','FAILED'),
                          self.p('mock_remove_seconds')*3+20,'MOCK_REMOVE_TIMEOUT')
                if self.removal_status['state']=='FAILED': raise RuntimeError('REMOVE_FAILED: '+str(self.removal_status))
                self.removal_ack_sim=self.sim_now()
                self.event('VERIFY_CLEAR')
                self.clear_streak=0
                after=self.costmap_time
                self.wait(lambda:self.clear_streak>=3 and self.costmap_time>after and self.costmap_stamp>self.removal_ack_sim and self.costmap_clear(),
                          self.p('clear_timeout'),'CLEARANCE_UNCONFIRMED(scan and costmap)')
                verified_path=self.plan_detour(check_target=False)
                if not verified_path or path_length(verified_path)>clear_length+1.0:
                    raise RuntimeError('SHORT_PASSAGE_NOT_CONFIRMED_BY_NAV2')
                self.report['verified_clear_nav2_length']=path_length(verified_path)
                self.event('GOAL_AFTER_MOCK_REMOVE',clear_scan_streak=self.clear_streak)
            else:
                self.event('DETOUR')
                switch_end=self.sim_now()+self.p('switch_seconds')
                self.wait(lambda:self.sim_now()>=switch_end,self.p('switch_seconds')*3+2,'SWITCH_TIMEOUT')
            self.navigate((self.p('goal_x'),self.p('goal_y')))
            final=self.pose()
            if math.dist(final[:2],(self.p('goal_x'),self.p('goal_y')))>.30:
                raise RuntimeError('GOAL_POSITION_OUTSIDE_TOLERANCE')
            self.report['final_pose']=list(final)
            self.times['arrival_sim']=self.sim_now()
            self.times['arrival_wall']=time.monotonic()
            success=True
            self.event('DONE',choice=self.choice,final_pose=final)
        except Exception as exc:
            self.failure=str(exc)
            self.enabled=False
            self.drive_pub.publish(Bool(data=False))
            if self.active_handle is not None:
                self.active_handle.cancel_goal_async()
            self.event('FAILED',reason=self.failure)
        finally:
            self.enabled=False
            self.drive_pub.publish(Bool(data=False))
            self.finish(success)
            # Keep final latched state visible and flush publications before exit.
            display_end=time.monotonic()+2.
            while rclpy.ok() and time.monotonic()<display_end:
                rclpy.spin_once(self,timeout_sec=.05)
        return success

    def finish(self,success):
        sim=self.sim_now()
        wall=time.monotonic()
        self.report.update(success=success,failure_reason=self.failure,choice=self.choice,
            execution_completed=success and self.p('stage')=='full',
            scans=self.scans,tf_retry_count=self.tf_errors,
            total_sim_seconds=None if self.started_sim is None or self.clock_fault else sim-self.started_sim,
            total_wall_seconds=None if self.mission_started_wall is None else wall-self.mission_started_wall,
            process_wall_seconds=wall-self.started_wall,
            startup_wall_seconds=None if self.mission_started_wall is None else self.mission_started_wall-self.started_wall,
            selected_execution_sim_seconds=None if 'decision_sim' not in self.times or self.clock_fault or self.p('stage')!='full' else self.times.get('arrival_sim',sim)-self.times['decision_sim'],
            selected_execution_wall_seconds=None if 'decision_wall' not in self.times or self.p('stage')!='full' else self.times.get('arrival_wall',wall)-self.times['decision_wall'],
            v_eff=self.p('v_eff'),mock_remove_seconds=self.p('mock_remove_seconds'),switch_seconds=self.p('switch_seconds'),
            decision_margin=self.p('decision_margin'),time_parameters='assumed',removable_assumption=self.p('removable'))
        self.report['executed_route_max_y']=max(self.route_y) if self.route_y else None
        self.report['clear_scan_streak']=self.clear_streak
        if self.frozen_target is not None:
            self.report['target_bounds']=list(map(float,self.frozen_target['bounds']))
        if self.costmap is not None:
            cm=self.costmap
            np.savez_compressed(self.output/'last_observations.npz',
                costmap=np.array(cm.data).reshape(cm.info.height,cm.info.width),
                origin=[cm.info.origin.position.x,cm.info.origin.position.y],resolution=cm.info.resolution,
                scan_points=self.last_points,
                static_map=self.grid.data if self.grid is not None else np.empty((0,0)))
        (self.output/'result.json').write_text(json.dumps(self.report,indent=2))
        with open(self.output/'result.csv','w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(self.report))
            writer.writeheader();writer.writerow(self.report)
        self.events_file.close()
        self.trajectory_file.flush()


def main(args=None):
    rclpy.init(args=args)
    node=Mission()
    try:
        ok=node.run()
    finally:
        node.trajectory_file.close()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
