"""Base drive additionally requires a fresh arm-stowed status and no arm lock."""
import json
import time
import rclpy
from std_msgs.msg import Bool,String
from nav_msgs.msg import Path
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from namo_sim.velocity_gate import VelocityGate
from namo_sim.mission import DURABLE
from .drive_safety import drive_permission,slew_command

class ArmVelocityGate(VelocityGate):
    def __init__(self):
        self.arm_stowed=False;self.arm_received=-float('inf')
        self.arm_locked=True;self.lock_received=-float('inf')
        self.plan_received=self.plan_stamp=-float('inf')
        self.applied=(0.,0.);self.slew_time=time.monotonic()
        super().__init__()
        self.create_subscription(String,'/namo/physical_removal_status',self.on_arm,DURABLE)
        self.create_subscription(Bool,'/namo/manipulation_lock',self.on_lock,10)
        self.create_subscription(Path,'/received_global_plan',self.on_controller_plan,10)
        self.status=self.create_publisher(String,'/namo/drive_gate_status',10)
    def on_arm(self,msg):
        try:
            d=json.loads(msg.data)
            self.arm_stowed=d.get('stowed_verified',False) and not d.get('busy',True) and not d.get('holding_contact',True)
        except ValueError:self.arm_stowed=False
        self.arm_received=time.monotonic()
    def on_lock(self,msg):
        self.arm_locked=msg.data;self.lock_received=time.monotonic()
    def on_lease(self,msg):
        super().on_lease(msg)
        if not msg.data:self.applied=(0.,0.)
    def on_controller_plan(self,msg):
        # RPP transforms this path using its current robot pose every control
        # cycle. A newly received Twist alone cannot prove that pose is fresh.
        if msg.poses:
            self.plan_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.plan_received=time.monotonic()
    def tick(self):
        requested=self.enabled;now=time.monotonic()
        sim=self.get_clock().now().nanoseconds/1e9
        self.enabled,reason=drive_permission(requested=requested,lease_age=now-self.lease_time,
            command_age=now-self.command_time,arm_age=now-self.arm_received,lock_age=now-self.lock_received,
            arm_stowed=self.arm_stowed,arm_locked=self.arm_locked,plan_receipt_age=now-self.plan_received,
            plan_stamp_age=sim-self.plan_stamp,lease_timeout=self.get_parameter('lease_timeout').value,
            command_timeout=self.get_parameter('command_timeout').value)
        self.applied=slew_command((self.command.linear.x,self.command.angular.z),self.applied,
                                  now-self.slew_time,self.enabled)
        self.slew_time=now
        output=Twist();output.linear.x,output.angular.z=self.applied
        self.output.publish(output)
        if hasattr(self,'status'):
            self.status.publish(String(data=json.dumps(dict(allowed=self.enabled,requested=requested,reason=reason),ensure_ascii=False)))
        self.enabled=requested

def main(args=None):
    # Keep the context alive until the final zero command is published.
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);n=ArmVelocityGate()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.enabled=False
        if rclpy.ok():n.tick()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
