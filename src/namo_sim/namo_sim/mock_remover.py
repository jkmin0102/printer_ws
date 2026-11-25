"""Simulator-only removal executor; success acknowledges teleport, not clearance.

The mission must verify fresh scan rays and the updated costmap before releasing
the velocity gate. This executor never publishes Gazebo ground-truth positions.
"""

import json
import math
import time

import rclpy
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MockRemover(Node):
    """Accept one removal per run and progress without blocking callbacks."""

    ODOM_MAX_AGE = 1.0
    GATE_MAX_AGE = 0.75
    ACTIVE = ('WAITING', 'MOVING')

    def __init__(self):
        super().__init__(
            'mock_remover',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        defaults = {
            'mock_remove_seconds': 22.0,
            'model_name': 'movable_obstacle',
            'stash_x': -2.0,
            'stash_y': -2.0,
            'stash_z': 0.3,
            'service_timeout': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.wait_seconds = float(self.get_parameter('mock_remove_seconds').value)
        self.timeout = float(self.get_parameter('service_timeout').value)
        self.model_name = str(self.get_parameter('model_name').value)
        self.stash = tuple(float(self.get_parameter(f'stash_{axis}').value)
                           for axis in ('x', 'y', 'z'))
        if (not math.isfinite(self.wait_seconds) or self.wait_seconds < 0.0
                or not math.isfinite(self.timeout) or self.timeout <= 0.0
                or not self.model_name.strip()
                or not all(math.isfinite(x) for x in self.stash)):
            raise ValueError('Invalid mock removal duration, timeout, model name, or stash')

        self.state = 'IDLE'
        self.reason = 'Ready; single movable obstacle is an experimental assumption'
        self.remaining = 0.0
        self.odom_received = self.gate_received = float('-inf')
        self.linear_speed = self.angular_speed = float('inf')
        self.drive_enabled = True
        self.start_sim = 0.0
        self.last_sim = self.sim_time()
        self.last_clock_advance_wall = time.monotonic()
        self.wait_deadline = self.service_deadline = float('inf')
        self.future = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/namo/removal_status', qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/namo/drive_enabled', self.on_gate, 10)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_service(Trigger, '/namo/remove', self.on_request)
        self.create_timer(0.1, self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.linear_speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        self.angular_speed = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
        self.odom_received = time.monotonic()

    def on_gate(self, msg):
        self.drive_enabled = bool(msg.data)
        self.gate_received = time.monotonic()

    def unsafe_reason(self, now):
        if now - self.gate_received > self.GATE_MAX_AGE:
            return 'Closed-gate heartbeat missing or stale'
        if self.drive_enabled:
            return 'Drive gate is enabled'
        if now - self.odom_received > self.ODOM_MAX_AGE:
            return 'Odometry missing or stale; actual stop cannot be confirmed'
        if not (math.isfinite(self.linear_speed) and math.isfinite(self.angular_speed)):
            return 'Odometry velocity is not finite'
        if self.linear_speed >= 0.01 or self.angular_speed >= 0.02:
            return ('Actual base is not stopped: '
                    f'linear={self.linear_speed:.4f} m/s, angular={self.angular_speed:.4f} rad/s')
        return ''

    def on_request(self, _request, response):
        if self.state != 'IDLE':
            response.success = False
            response.message = f'Removal rejected: state={self.state}; restart run to reset'
            return response
        now = time.monotonic()
        unsafe = self.unsafe_reason(now)
        if unsafe:
            response.success = False
            response.message = f'Removal rejected: {unsafe}'
            return response
        self.start_sim = self.last_sim = self.sim_time()
        self.last_clock_advance_wall = now
        # Allow slow simulation down to approximately 0.1 real-time factor,
        # while retaining a finite wall deadline even if clock ticks very slowly.
        self.wait_deadline = now + self.wait_seconds * 10.0 + self.timeout
        self.remaining = self.wait_seconds
        self.transition('WAITING', 'MOCK REMOVE: waiting configured simulation seconds')
        response.success = True
        response.message = 'Accepted MOCK REMOVE; completion is on /namo/removal_status'
        return response

    def transition(self, state, reason):
        self.state, self.reason = state, reason
        if state == 'FAILED':
            self.get_logger().error(f'MOCK REMOVE {state}: {reason}')
        else:
            self.get_logger().info(f'MOCK REMOVE {state}: {reason}')
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state,
            'remaining': max(0.0, self.remaining),
            'reason': self.reason,
            'mock_remove_seconds': self.wait_seconds,
        }, ensure_ascii=False, allow_nan=False)))

    def fail(self, reason):
        # A service already sent cannot be recalled. Ignore its late response;
        # the mission remains stopped and must treat FAILED as terminal.
        if self.future is not None and not self.future.done():
            self.future.cancel()
        self.transition('FAILED', reason)

    def move_request(self):
        request = SetEntityState.Request()
        request.state.name = self.model_name
        request.state.reference_frame = 'world'
        pose = request.state.pose
        pose.position.x, pose.position.y, pose.position.z = self.stash
        pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        return request

    def tick(self):
        if self.state not in self.ACTIVE:
            self.publish_status()
            return
        now, sim = time.monotonic(), self.sim_time()
        unsafe = self.unsafe_reason(now)
        if unsafe:
            self.fail(unsafe)
            return
        if sim < self.last_sim - 1e-9:
            self.fail('Simulation clock moved backwards during removal; restart the run')
            return
        if sim > self.last_sim + 1e-9:
            self.last_clock_advance_wall = now
        self.last_sim = sim
        if now - self.last_clock_advance_wall > self.timeout:
            self.fail('Simulation clock stopped advancing: wall timeout')
            return

        if self.state == 'WAITING':
            self.remaining = max(0.0, self.wait_seconds - (sim - self.start_sim))
            if now > self.wait_deadline:
                self.fail('MOCK REMOVE waiting exceeded bounded wall timeout')
                return
            if self.remaining > 1e-9:
                self.publish_status()
                return
            self.remaining = 0.0
            self.service_deadline = now + self.timeout
            self.transition('MOVING', 'Waiting for Gazebo set_entity_state service')

        if self.future is None:
            if now > self.service_deadline:
                self.fail('Gazebo set_entity_state service unavailable: wall timeout')
                return
            if self.client.service_is_ready():
                try:
                    self.future = self.client.call_async(self.move_request())
                except Exception as exc:
                    self.fail(f'Gazebo set_entity_state request error: {exc}')
                    return
                self.service_deadline = now + self.timeout
                self.reason = 'MOCK REMOVE: waiting for Gazebo relocation acknowledgement'
        elif self.future.done():
            try:
                result = self.future.result()
            except Exception as exc:
                self.fail(f'Gazebo set_entity_state response error: {exc}')
                return
            if result is None or not result.success:
                self.fail('Gazebo rejected relocation (success=false)')
                return
            self.transition(
                'SUCCEEDED',
                'Relocation acknowledged; mission must verify fresh LiDAR and costmap clearance',
            )
            return
        elif now > self.service_deadline:
            self.fail('Gazebo relocation response wall timeout; outcome is unconfirmed')
            return
        self.publish_status()

    def stop(self):
        if self.state in self.ACTIVE and rclpy.ok():
            self.fail('Executor shutdown during MOCK REMOVE')


def main(args=None):
    rclpy.init(args=args)
    node = MockRemover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
