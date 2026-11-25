"""Stage 2: drive S→P→G using real Nav2; no obstacle/decision code."""
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class Baseline(Node):
    def __init__(self):
        super().__init__('namo_baseline')
        self.declare_parameter('output', '/tmp/namo_baseline.json')
        self.pub = self.create_publisher(Bool, '/namo/drive_enabled', 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.state_future = None
        self.active = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scans = 0
        self.distance = 0.0
        self.previous = None
        self.create_subscription(LaserScan, '/scan', self.scan, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.enabled = False
        self.create_timer(0.1, lambda: self.pub.publish(Bool(data=self.enabled)))

    def scan(self, msg):
        self.scans += 1

    def odom(self, msg):
        p = msg.pose.pose.position
        if self.previous:
            self.distance += math.hypot(p.x-self.previous[0], p.y-self.previous[1])
        self.previous = (p.x, p.y)

    def wait(self, predicate, timeout):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def ready(self):
        if self.state_future is not None and self.state_future.done():
            self.active = self.state_future.result().current_state.id == 3
            self.state_future = None
        if not self.active and self.state_future is None and self.lifecycle.service_is_ready():
            self.state_future = self.lifecycle.call_async(GetState.Request())
        return (self.active and self.client.server_is_ready() and self.scans > 3
                and self.tf.can_transform('map', 'base_footprint', rclpy.time.Time()))

    def run(self):
        started = time.monotonic()
        reason = ''
        results = []
        try:
            if not self.wait(self.ready, 90):
                raise RuntimeError('Nav2/scan/map TF readiness timeout')
            for x, y, yaw in [(0., 0., 0.), (6., 0., 0.)]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.z = math.sin(yaw/2)
                pose.pose.orientation.w = math.cos(yaw/2)
                future = self.client.send_goal_async(NavigateToPose.Goal(pose=pose))
                if not self.wait(future.done, 10) or not future.result().accepted:
                    raise RuntimeError('Nav2 goal rejected or response timeout')
                handle = future.result()
                result = handle.get_result_async()
                self.enabled = True
                if not self.wait(result.done, 180):
                    handle.cancel_goal_async()
                    raise RuntimeError('Nav2 goal execution timeout')
                self.enabled = False
                status = result.result().status
                results.append(status)
                if status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f'Nav2 result status={status}')
            tf = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            final = [tf.transform.translation.x, tf.transform.translation.y]
            if math.dist(final, [6., 0.]) > 0.3:
                raise RuntimeError(f'Final pose outside tolerance: {final}')
        except Exception as exc:
            reason = str(exc)
            final = None
        self.enabled = False
        self.pub.publish(Bool(data=False))
        report = dict(success=not reason, reason=reason, statuses=results,
                      final_pose=final, scans=self.scans, odom_distance=self.distance,
                      wall_seconds=time.monotonic()-started)
        target = Path(self.get_parameter('output').value)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, indent=2))
        self.get_logger().info(json.dumps(report))
        return not reason


def main(args=None):
    rclpy.init(args=args)
    node = Baseline()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
