#!/usr/bin/env python3
"""Read one actual wrist RGB-D frame and save strict-YOLO diagnostic evidence.

No controller commands, Gazebo object queries, or object-state edits are made.
Run after the arm reaches its recognition pose and the can is in the scene.
"""

import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

from namo_manipulation.can_perception import configure_torch_threads, image_array, red_detection


class Capture(Node):
    def __init__(self, support_frame):
        super().__init__('namo_camera_evidence', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.info = self.pair = None
        self.support_frame = support_frame
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, '/realsense/d435i/color/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.rgb = message_filters.Subscriber(self, Image, '/realsense/d435i/color/image_raw',
                                              qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image,
            '/realsense/d435i/aligned_depth_to_color/image_raw', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb, self.depth], 5, .04)
        self.sync.registerCallback(self.on_pair)

    def on_info(self, msg):
        self.info = msg

    def on_pair(self, rgb, depth):
        self.pair = rgb, depth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', default='/home/user2/mobility_simulation/namo_ws/best.pt')
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--support-frame', default='odom')
    parser.add_argument('--cpu-threads', type=int, default=2)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = Capture(args.support_frame)
    started = time.monotonic()
    pair = info = transform = base_transform = None
    try:
        while rclpy.ok() and time.monotonic()-started < args.timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.pair is None or node.info is None or time.monotonic()-started < 1.5:
                continue
            rgb, depth = node.pair
            stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
            age = (node.get_clock().now()-stamp).nanoseconds*1e-9
            if stamp.nanoseconds == 0 or not -.05 <= age <= 1.:
                continue
            if (not node.tf.can_transform(args.support_frame, rgb.header.frame_id, stamp)
                    or not node.tf.can_transform(args.support_frame, 'base_link', stamp)):
                continue
            pair, info = node.pair, node.info
            transform = node.tf.lookup_transform(args.support_frame, rgb.header.frame_id, stamp).transform
            base_transform = node.tf.lookup_transform(args.support_frame, 'base_link', stamp).transform
            break
        if pair is None:
            raise RuntimeError('Fresh RGB/depth/CameraInfo/stamped optical TF capture timed out')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    rgb, depth = pair
    bgr = image_array(rgb)
    depth_m = image_array(depth).astype(float)
    if depth.encoding.lower() == '16uc1':
        depth_m *= .001
    cv2.imwrite(str(output/'rgb_actual.png'), bgr)
    np.savez_compressed(output/'rgbd_actual.npz', rgb=bgr, depth_m=depth_m, camera_k=np.asarray(info.k))
    weights = Path(args.model)
    inference_device = configure_torch_threads(args.cpu_threads)
    model = YOLO(str(weights))
    inference_started = time.monotonic()
    prediction = model.predict(bgr, conf=.01, imgsz=640, device=inference_device, verbose=False)[0]
    inference_wall = time.monotonic()-inference_started
    detections = []
    annotated = bgr.copy()
    for box in prediction.boxes:
        name = model.names[int(box.cls[0].item())]
        confidence = float(box.conf[0].item())
        xyxy = list(map(float, box.xyxy[0].cpu().tolist()))
        detections.append(dict(class_name=name, confidence=confidence, bbox=xyxy))
        x0, y0, x1, y1 = map(int, xyxy)
        color = (0, 220, 0) if confidence >= .5 else (0, 160, 255)
        cv2.rectangle(annotated, (x0, y0), (x1, y1), color, 2)
        cv2.putText(annotated, f'{name} {confidence:.3f}', (x0, max(14, y0-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
    strict_coke = any(item['class_name'] == 'coke' and item['confidence'] >= .5 for item in detections)
    label = f'ACTUAL GAZEBO RGB | YOLO coke >=0.50: {strict_coke} | no fallback'
    cv2.putText(annotated, label, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    cv2.imwrite(str(output/'yolo_annotated.png'), annotated)
    red, mask = red_detection(bgr)
    cv2.imwrite(str(output/'sim_red_diagnostic_mask.png'), mask)
    q, t = transform.rotation, transform.translation
    base_q, base_t = base_transform.rotation, base_transform.translation
    report = {
        'source': 'actual Gazebo wrist camera topics; no simulator object-state lookup',
        'frame_id': rgb.header.frame_id,
        'stamp': {'sec': rgb.header.stamp.sec, 'nanosec': rgb.header.stamp.nanosec},
        'rgb_encoding': rgb.encoding, 'depth_encoding': depth.encoding,
        'shape': list(bgr.shape), 'camera_k': list(info.k),
        'support_frame': args.support_frame,
        'optical_to_support_translation': [t.x, t.y, t.z],
        'optical_to_support_quaternion_xyzw': [q.x, q.y, q.z, q.w],
        'base_link_to_support_translation': [base_t.x, base_t.y, base_t.z],
        'base_link_to_support_quaternion_xyzw': [base_q.x, base_q.y, base_q.z, base_q.w],
        'model_path': str(weights), 'model_sha256': hashlib.sha256(weights.read_bytes()).hexdigest(),
        'model_classes': model.names, 'diagnostic_threshold': .01, 'strict_threshold': .5,
        'strict_coke_detected': strict_coke, 'detections': detections,
        'inference_wall_seconds': inference_wall,
        'cpu_threads': args.cpu_threads, 'cpu_interop_threads': 1,
        'sim_red_diagnostic_only': {'not_actuation_input': True, 'detected': red is not None,
            'mask_pixels': int(np.count_nonzero(mask)),
            'bbox': None if red is None else [red.x1, red.y1, red.x2, red.y2]},
    }
    (output/'yolo_evidence.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
