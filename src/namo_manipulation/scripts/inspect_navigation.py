#!/usr/bin/env python3
"""Read-only validation logger; truth is never published to any control node."""
import json
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,PointStamped
from nav_msgs.msg import Odometry,Path
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool,String
from tf2_ros import Buffer,TransformListener
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class Probe(Node):
    def __init__(self,path):
        super().__init__('namo_navigation_probe',parameter_overrides=[Parameter('use_sim_time',value=True)]);self.data={};self.stream=open(path,'w',buffering=1)
        self.counts={};self.received={};self.plan_stamp=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.client=self.create_client(GetEntityState,'/gazebo/get_entity_state');self.future=None
        self.create_subscription(Odometry,'/odom',lambda m:self.odom(m),10)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.cmd('cmd_vel',m),10)
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.cmd('cmd_vel_nav',m),10)
        self.create_subscription(Bool,'/namo/drive_enabled',lambda m:self.data.update(gate=m.data),10)
        self.create_subscription(Path,'/plan',lambda m:self.path('plan',m),10)
        self.create_subscription(Path,'/received_global_plan',lambda m:self.path('received_global_plan',m),10)
        self.create_subscription(PointStamped,'/lookahead_point',self.lookahead,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(String,'/namo/drive_gate_status',self.gate_status,10)
        self.create_subscription(String,'/namo/physical_removal_status',self.arm_status,10)
        self.create_subscription(ModelStates,'/gazebo/model_states',self.models,10)
        self.create_timer(.25,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def mark(self,key):
        self.counts[key]=self.counts.get(key,0)+1;self.received[key]=time.monotonic()
    def scan(self,m):self.mark('scan');self.data['scan_stamp']=self.stamp(m)
    def gate_status(self,m):
        try:self.data['drive_gate_status']=json.loads(m.data)
        except ValueError:pass
    def arm_status(self,m):
        try:self.data['arm_status']=json.loads(m.data)
        except ValueError:pass
    def models(self,m):
        if 'namo_can' in m.name:
            pose=m.pose[m.name.index('namo_can')];p=pose.position;q=pose.orientation
            self.data['validation_only_can_world_pose']=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.data.update(odom=[p.x,p.y,p.z,q.x,q.y,q.z,q.w],odom_twist=[m.twist.twist.linear.x,m.twist.twist.angular.z])
        self.mark('odom');self.data['odom_stamp']=self.stamp(m)
    def cmd(self,k,m):self.data[k]=[m.linear.x,m.angular.z];self.mark(k)
    def path(self,key,m):
        self.mark(key)
        if m.poses:
            self.data[key]=dict(frame=m.header.frame_id,stamp=self.stamp(m),count=len(m.poses),
                first=[m.poses[0].pose.position.x,m.poses[0].pose.position.y],
                last=[m.poses[-1].pose.position.x,m.poses[-1].pose.position.y],
                tail=[[p.pose.position.x,p.pose.position.y] for p in m.poses[-4:]])
        if key=='received_global_plan':self.plan_stamp=rclpy.time.Time.from_msg(m.header.stamp)
    def lookahead(self,m):
        self.mark('lookahead');self.data['lookahead']=dict(frame=m.header.frame_id,stamp=self.stamp(m),xyz=[m.point.x,m.point.y,m.point.z])
    def tick(self):
        if self.future and self.future.done():
            result=self.future.result()
            if result.success:
                p=result.state.pose.position;q=result.state.pose.orientation
                self.data['validation_only_world_pose']=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            self.future=None
        if self.future is None and self.client.service_is_ready():
            self.future=self.client.call_async(GetEntityState.Request(name='namo_robot',reference_frame='world'))
        for parent,child in [('map','base_footprint'),('map','odom')]:
            try:
                tr=self.tf.lookup_transform(parent,child,rclpy.time.Time());p=tr.transform.translation;q=tr.transform.rotation
                self.data[parent+'_'+child]=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                self.data[parent+'_'+child+'_stamp']=self.stamp(tr)
            except Exception:pass
        if self.plan_stamp is not None:
            try:
                tr=self.tf.lookup_transform('map','base_footprint',self.plan_stamp);p=tr.transform.translation
                self.data['map_base_at_plan_stamp']=[p.x,p.y,p.z]
            except Exception as e:self.data['map_base_at_plan_stamp_error']=str(e)
        now=time.monotonic()
        self.stream.write(json.dumps(dict(wall=time.time(),sim=self.get_clock().now().nanoseconds*1e-9,
            counts=self.counts,receipt_age={k:now-v for k,v in self.received.items()},**self.data))+'\n')

rclpy.init();n=Probe(sys.argv[1])
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
    n.stream.close();n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
