#!/usr/bin/env python3
"""Record ROS graph interfaces without reading simulated object ground truth."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import get_action_names_and_types


TOPICS = (
    '/scan', '/scan_clearing', '/odom', '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_nav',
    '/global_costmap/costmap', '/map',
)
REQUIRED_ACTIONS = ('/navigate_to_pose', '/compute_path_to_pose')


def endpoint(info):
    """Serialize public Humble TopicEndpointInfo fields and QoS."""
    qos = info.qos_profile
    policy = lambda value: getattr(value, 'name', str(value))
    namespace = info.node_namespace.rstrip('/')
    return {
        'node_name': info.node_name,
        'node_namespace': info.node_namespace,
        'node_fqn': f'{namespace}/{info.node_name}',
        'topic_type': info.topic_type,
        'endpoint_type': policy(info.endpoint_type),
        'endpoint_gid': bytes(info.endpoint_gid).hex(),
        'qos': {
            'reliability': policy(qos.reliability),
            'durability': policy(qos.durability),
            'history': policy(qos.history),
            'depth': qos.depth,
            'liveliness': policy(qos.liveliness),
            'deadline_ns': qos.deadline.nanoseconds,
            'lifespan_ns': qos.lifespan.nanoseconds,
            'liveliness_lease_duration_ns': qos.liveliness_lease_duration.nanoseconds,
        },
    }


def snapshot(node):
    graph_types = dict(node.get_topic_names_and_types())
    topics = {}
    for name in TOPICS:
        topics[name] = {
            'types': graph_types.get(name, []),
            'publishers': [endpoint(info)
                           for info in node.get_publishers_info_by_topic(name)],
            'subscribers': [endpoint(info)
                            for info in node.get_subscriptions_info_by_topic(name)],
        }
    actions = dict(get_action_names_and_types(node))
    missing = [f'{name} publisher' for name in ('/scan', '/cmd_vel')
               if not topics[name]['publishers']]
    missing.extend(f'{name} action' for name in REQUIRED_ACTIONS if name not in actions)
    return {
        'ready': not missing,
        'missing_readiness': missing,
        'topics': topics,
        'action_names_types': actions,
        'service_names_types': dict(node.get_service_names_and_types()),
        'node_names': sorted([
            {'name': name, 'namespace': ns, 'fqn': f"{ns.rstrip('/')}/{name}"}
            for name, ns in node.get_node_names_and_namespaces()
        ], key=lambda item: item['fqn']),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', nargs='?', help='Destination JSON file')
    parser.add_argument('--output', dest='output_option', help='Alternative JSON output option')
    args, ros_args = parser.parse_known_args()
    target = args.output_option or args.output
    if not target:
        parser.error('provide an output JSON path')
    output = Path(target).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = rclpy.create_node('namo_runtime_inspector')
    started = time.monotonic()
    report = {}
    interrupted = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = time.monotonic() - started
            if elapsed < 5.0:
                continue
            report = snapshot(node)
            if report['ready'] or elapsed >= 30.0:
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if not report:
            report = snapshot(node) if rclpy.ok() else {
                'ready': False,
                'missing_readiness': ['ROS context shut down before graph capture'],
                'topics': {}, 'action_names_types': {},
                'service_names_types': {}, 'node_names': [],
            }
        report.update({
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'elapsed_wall_seconds': time.monotonic() - started,
            'interrupted': interrupted,
            'read_only': True,
            'note': 'Graph metadata only. No Gazebo model/link states are read.',
        })
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps({
            'output': str(output.resolve()), 'ready': report['ready'],
            'missing_readiness': report['missing_readiness'],
        }, ensure_ascii=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report['ready'] and not interrupted else 2


if __name__ == '__main__':
    raise SystemExit(main())
