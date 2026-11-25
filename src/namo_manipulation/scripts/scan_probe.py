#!/usr/bin/env python3
"""Read-only LiDAR fixture validation, never an input to planning or detection."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from namo_manipulation.arm_executor import quaternion_matrix


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='/tmp/namo_scan_validation.json')
    parser.add_argument('--fixture-xy',nargs=2,type=float,default=[.30,0.])
    args=parser.parse_args()
    rclpy.init()
    node=Node('scan_fixture_validation',parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node);latest=[None]
    node.create_subscription(LaserScan,'/scan',lambda msg:latest.__setitem__(0,msg),qos_profile_sensor_data)
    deadline=time.monotonic()+15
    report=None
    while time.monotonic()<deadline and rclpy.ok():
        rclpy.spin_once(node,timeout_sec=.05)
        msg=latest[0]
        if msg is None: continue
        try:
            tr=tf.lookup_transform('odom',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp))
            base=tf.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp))
        except Exception:continue
        r=quaternion_matrix(tr.transform.rotation)
        t=tr.transform.translation;origin=np.array([t.x,t.y,t.z])
        br=quaternion_matrix(base.transform.rotation);bt=base.transform.translation
        bo=np.array([bt.x,bt.y,bt.z])
        samples=[];can_matches=[];self_hits=[]
        for index,distance in enumerate(msg.ranges):
            angle=msg.angle_min+index*msg.angle_increment
            degrees=(math.degrees(angle)+180)%360-180
            if not math.isfinite(distance) or not msg.range_min<=distance<msg.range_max: continue
            local=np.array([distance*math.cos(angle),distance*math.sin(angle),0.])
            point=r@local+origin
            bp=br@local+bo
            reference_distance=math.hypot(point[0]-args.fixture_xy[0],point[1]-args.fixture_xy[1])
            sample=dict(angle_deg=degrees,range_m=distance,odom_xyz=point.tolist(),base_xyz=bp.tolist())
            if reference_distance<.05:
                sample['fixture_validation_class']='near_known_can_fixture';can_matches.append(sample)
            elif -.21<bp[0]<.17 and abs(bp[1])<.17:
                sample['fixture_validation_class']='within_base_footprint';self_hits.append(sample)
            else:sample['fixture_validation_class']='other'
            if abs(degrees)<=35:samples.append(sample)
        report=dict(frame=msg.header.frame_id,stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
            range_min=msg.range_min,range_max=msg.range_max,sensor_origin_odom=origin.tolist(),
            fixture_reference_xy=args.fixture_xy,fixture_reference_is_validation_only=True,
            can_fixture_hit_count=len(can_matches),self_footprint_hit_count=len(self_hits),
            forward_samples=samples,can_fixture_samples=can_matches,self_footprint_samples=self_hits)
        break
    node.destroy_node();rclpy.shutdown()
    if report is None:raise SystemExit('No fresh stamped scan+TF available')
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if not k.endswith('samples')},indent=2))


if __name__=='__main__':main()
