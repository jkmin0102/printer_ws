#!/usr/bin/env python3
"""Finite model/controller/sensor smoke check on an already running simulator."""
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('model_smoke')
    out = Path('.namo_manip/model_check'); out.mkdir(parents=True, exist_ok=True)
    latest = {}
    subscriptions = []
    for name, topic, kind in [
            ('scan', '/scan', LaserScan), ('joints', '/joint_states', JointState),
            ('rgb', '/realsense/d435i/color/image_raw', Image),
            ('depth', '/realsense/d435i/aligned_depth_to_color/image_raw', Image),
            ('info', '/realsense/d435i/color/camera_info', CameraInfo)]:
        subscriptions.append(node.create_subscription(kind, topic,
            lambda message, key=name: latest.__setitem__(key, message), qos_profile_sensor_data))
    tf = Buffer(); listener = TransformListener(tf, node)

    def wait(predicate, seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate(): return True
        return False

    report = {'sensors_ready': wait(lambda: len(latest)==5, 30)}
    service = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if service.wait_for_service(timeout_sec=5):
        future = service.call_async(ListControllers.Request())
        if wait(future.done, 5):
            report['controllers'] = {c.name: c.state for c in future.result().controller}
    arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    grip = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
    report['arm_server'] = arm.wait_for_server(timeout_sec=5)
    report['gripper_server'] = grip.wait_for_server(timeout_sec=5)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['joint1','joint2','joint3','joint4']
    point = JointTrajectoryPoint(positions=[.15,-.95,.75,.2])
    point.time_from_start.sec = 3
    goal.trajectory.points = [point]
    if report['arm_server']:
        future=arm.send_goal_async(goal)
        if wait(future.done,5) and future.result().accepted:
            result=future.result().get_result_async()
            report['arm_completed']=wait(result.done,15)
            if result.done():
                report['arm_status']=result.result().status
                report['arm_error_code']=result.result().result.error_code
                report['arm_error_string']=result.result().result.error_string
    if report['gripper_server']:
        goal=GripperCommand.Goal();goal.command.position=.012;goal.command.max_effort=1.
        future=grip.send_goal_async(goal)
        if wait(future.done,5) and future.result().accepted:
            result=future.result().get_result_async()
            report['gripper_completed']=wait(result.done,10)
            if result.done():
                report['gripper_status']=result.result().status
                report['gripper_position']=result.result().result.position
    wait(lambda:False, .5)
    for target, source in [('base_link','end_effector_link'),('end_effector_link','d435i_link'),
                            ('d435i_color_frame','d435i_color_optical_frame'),('base_footprint','base_scan')]:
        try:
            tr=tf.lookup_transform(target,source,rclpy.time.Time())
            v=tr.transform
            report[target+'<-'+source]={'xyz':[v.translation.x,v.translation.y,v.translation.z],
                'xyzw':[v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w]}
        except Exception as exc:report[target+'<-'+source]=str(exc)
    if 'rgb' in latest:
        m=latest['rgb'];report['rgb']={'encoding':m.encoding,'width':m.width,'height':m.height,'frame':m.header.frame_id}
        data=np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':data=data[:,:,::-1]
        PILImage.fromarray(data).save(out/'wrist_rgb.png')
    if 'depth' in latest:
        m=latest['depth'];data=np.frombuffer(bytes(m.data),dtype='<f4' if not m.is_bigendian else '>f4')
        finite=data[np.isfinite(data)]
        report['depth']={'encoding':m.encoding,'width':m.width,'height':m.height,'frame':m.header.frame_id,
                         'finite_pixels':len(finite),'minimum':float(finite.min()) if len(finite) else None}
    if 'scan' in latest:
        m=latest['scan'];report['scan']={'frame':m.header.frame_id,'beams':len(m.ranges),'range_min':m.range_min,'range_max':m.range_max}
    report['topics']=node.get_topic_names_and_types()
    (out/'smoke.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
