#!/usr/bin/env python3
"""Isolated stationary arm test: publishes 20Hz zero Twist and closed gate.

Run only in the dedicated arm-test ROS domain, never beside a navigation mission.
Gazebo truth is recorded for validation and never sent to the arm executor.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self,args):
        super().__init__('physical_arm_validation_probe_'+args.operation,parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.args=args
        self.pub=self.create_publisher(Bool,'/namo/drive_enabled',10)
        self.zero=self.create_publisher(Twist,'/cmd_vel',10)
        def hold_base():
            self.pub.publish(Bool(data=False))
            self.zero.publish(Twist())
        self.create_timer(.05,hold_base,clock=Clock(clock_type=ClockType.STEADY_TIME))
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/namo/physical_removal_status',self.on_status,qos)
        self.create_subscription(String,'/namo/can_observation',self.on_observation,qos)
        self.create_subscription(ModelStates,'/gazebo/model_states',self.on_models,10)
        self.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',self.on_arm_state,10)
        self.client=self.create_client(Trigger,{'prepare':'/namo/prepare_grasp','stow':'/namo/stow_arm','remove':'/namo/physical_remove'}[args.operation])
        self.status={};self.status_received=-math.inf;self.can=None;self.samples=[]
        self.last_sample=-math.inf
        self.observation={}
        self.arm_samples=[];self.last_arm_sample=-math.inf

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def on_status(self,msg):
        self.status=json.loads(msg.data);self.status_received=time.monotonic()

    def on_observation(self,msg):
        try:self.observation=json.loads(msg.data)
        except (TypeError,ValueError):self.observation={}

    def on_models(self,msg):
        if self.args.can_name not in msg.name:return
        p=msg.pose[msg.name.index(self.args.can_name)].position
        self.can=[p.x,p.y,p.z]
        now=self.now()
        if now-self.last_sample>=.05:
            self.samples.append(dict(sim_seconds=now,x=p.x,y=p.y,z=p.z,
                state=self.status.get('state',''),phase=self.status.get('phase',''),
                bilateral_contact=self.status.get('bilateral_contact',False)))
            self.last_sample=now

    def on_arm_state(self,msg):
        now=self.now()
        if now-self.last_arm_sample<.05:return
        row=dict(sim_seconds=now,stamp_seconds=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                 state=self.status.get('state',''),phase=self.status.get('phase',''))
        for index,name in enumerate(msg.joint_names):
            for prefix,values in [('actual',msg.actual.positions),('velocity',msg.actual.velocities),
                                  ('desired',msg.desired.positions),('error',msg.error.positions)]:
                row[prefix+'_'+name]=values[index] if index<len(values) else None
        self.arm_samples.append(row);self.last_arm_sample=now

    def wait(self,predicate,seconds):
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            if predicate(): return True
        return False

    def run(self):
        output=Path(self.args.output);output.mkdir(parents=True,exist_ok=True)
        if not self.wait(lambda:self.client.service_is_ready() and self.can is not None and self.status.get('ready_for_operation',False),30):
            raise RuntimeError('Arm not ready or validation-only model states unavailable: '+json.dumps(self.status))
        if self.args.prepare_first:
            if self.args.operation!='remove':
                raise RuntimeError('--prepare-first is only valid for remove')
            prepare=self.create_client(Trigger,'/namo/prepare_grasp')
            if not self.wait(prepare.service_is_ready,10):
                raise RuntimeError('Prepare service unavailable')
            started=time.monotonic()
            future=prepare.call_async(Trigger.Request())
            if not self.wait(future.done,20):
                raise RuntimeError('Prepare acknowledgement timeout')
            response=future.result()
            if not response.success:raise RuntimeError('Prepare rejected: '+response.message)
            if not self.wait(lambda:self.status_received>started and self.status.get('operation')=='PREPARE'
                             and self.status.get('state') in ('PREPARED','FAILED'),self.args.timeout):
                raise RuntimeError('Prepare completion timeout')
            (output/'chained_prepare_status.json').write_text(json.dumps(self.status,indent=2))
            if self.status.get('state')!='PREPARED':
                raise RuntimeError('Prepare failed: '+self.status.get('reason','unknown'))
            print('PREPARED; proceeding directly to REMOVE with uninterrupted zero-Twist hold',flush=True)
        if self.args.require_observation:
            prepared_at=self.now()
            def observed():
                m=self.observation;s=m.get('stamp',{})
                stamp=s.get('sec',0)+s.get('nanosec',0)*1e-9
                return m.get('valid',False) and m.get('stable',False) and stamp>=prepared_at and 0<=self.now()-stamp<1.5
            if not self.wait(observed,20):
                raise RuntimeError('Fresh post-prepare stable RGB-D observation unavailable: '+json.dumps(self.observation))
            (output/'observed_target.json').write_text(json.dumps(self.observation,indent=2))
        self.wait(lambda:False,.05 if self.args.require_observation else 1.)
        initial=list(self.can)
        started=time.monotonic()
        future=self.client.call_async(Trigger.Request())
        if not self.wait(future.done,20):raise RuntimeError('Arm request acknowledgement timeout')
        response=future.result()
        if not response.success:raise RuntimeError('Arm request rejected: '+response.message)
        print('ACK '+response.message,flush=True)
        terminal={'prepare':'PREPARED','stow':'STOWED','remove':'SUCCEEDED'}[self.args.operation]
        if not self.wait(lambda:self.status_received>started and self.status.get('operation')==self.args.operation.upper()
                         and self.status.get('state') in (terminal,'FAILED'),self.args.timeout):
            raise RuntimeError('Arm completion timeout')
        completed=self.status.get('state')==terminal
        self.wait(lambda:False,2.)
        final=list(self.can)
        result=dict(operation=self.args.operation,executor_completed=completed,executor_status=self.status,
            initial_can_world=initial,final_can_world=final,
            max_can_z=max(s['z'] for s in self.samples),
            lift_above_initial=max(s['z'] for s in self.samples)-initial[2],
            validation_truth_only=True,expected_place_world=self.args.expected_place,
            final_xy_error=math.hypot(final[0]-self.args.expected_place[0],final[1]-self.args.expected_place[1]),
            final_z_error=abs(final[2]-self.args.expected_place[2]))
        if self.args.operation=='remove':
            result['physical_success']=(completed and result['lift_above_initial']>.025
                and result['final_xy_error']<.04 and result['final_z_error']<.02
                and not self.status.get('bilateral_contact',True))
        else:result['physical_success']=completed
        (output/f'{self.args.operation}_validation.json').write_text(json.dumps(result,indent=2))
        with (output/f'{self.args.operation}_can_trajectory.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(self.samples[0]));writer.writeheader();writer.writerows(self.samples)
        if self.arm_samples:
            with (output/f'{self.args.operation}_arm_tracking.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(self.arm_samples[0]));writer.writeheader();writer.writerows(self.arm_samples)
        print(json.dumps(result,indent=2),flush=True)
        if self.args.hold_after>0: self.wait(lambda:False,self.args.hold_after)
        return result['physical_success']


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--operation',choices=['prepare','stow','remove'],default='prepare')
    parser.add_argument('--output',default=os.environ.get('NAMO_RUN_DIR','/tmp/namo_arm_validation'))
    parser.add_argument('--expected-place',nargs=3,type=float,default=[.12,.33,.0675])
    parser.add_argument('--can-name',default='namo_can')
    parser.add_argument('--timeout',type=float,default=180.)
    parser.add_argument('--hold-after',type=float,default=0.,help='Continue 20Hz stationary hold after storing result')
    parser.add_argument('--prepare-first',action='store_true',help='Prepare then remove while continuously holding the isolated base stopped')
    parser.add_argument('--require-observation',action='store_true',help='Wait for fresh post-prepare stable RGB-D metadata; configure executor pick_topic=/namo/can_pose and disable fixture target')
    args,ros=parser.parse_known_args()
    rclpy.init(args=ros)
    node=Probe(args)
    try:ok=node.run()
    except Exception as exc:
        output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
        (output/f'{args.operation}_request_failure.json').write_text(json.dumps(dict(
            physical_success=False,reason=str(exc),executor_status=node.status,
            validation_truth_only=True,last_can_world=node.can),indent=2))
        raise
    finally:node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__=='__main__':main()
