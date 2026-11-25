"""Stamped RGB-D localization of an upright can, without simulator object state.

YOLO is the default. SIM_RED is an explicit experimental color-segmentation
mode, never a fallback. Cylinder dimensions and a support plane are assumptions.
The reported pose is the center, not the depth-visible surface or grasp pose.
"""

from concurrent.futures import ThreadPoolExecutor
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
import message_filters
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_pkg.vision_utils import Detection, deproject_pixel, select_target_detection


def image_array(message):
    """Decode ROS Image respecting encoding, endianness and padded row stride.

    This avoids the installed Humble cv_bridge/NumPy-2 ABI incompatibility.
    It does not alter image geometry or invent missing pixel measurements.
    """
    encoding = message.encoding.lower()
    encodings = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3),
                 '32fc1': ('f4', 1), '16uc1': ('u2', 1)}
    if encoding not in encodings:
        raise ValueError(f'Unsupported image encoding {message.encoding}')
    kind, channels = encodings[encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<')+kind)
    width, height, stride = int(message.width), int(message.height), int(message.step)
    if min(width, height) <= 0 or stride < width*channels*dtype.itemsize:
        raise ValueError('Invalid image dimensions or row stride')
    if len(message.data) < height*stride:
        raise ValueError('Truncated image data')
    shape = (height, width, channels) if channels == 3 else (height, width)
    strides = (stride, channels*dtype.itemsize, dtype.itemsize) if channels == 3 else (stride, dtype.itemsize)
    array = np.ndarray(shape, dtype=dtype, buffer=message.data, strides=strides).copy()
    return array[:, :, ::-1].copy() if encoding == 'rgb8' else array


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def configure_torch_threads(cpu_threads=2):
    """Bound CPU inference work so ROS sensor/control callbacks retain CPU time."""
    import torch
    cpu_threads = int(cpu_threads)
    if cpu_threads < 1:
        raise ValueError('cpu_threads must be a positive integer')
    torch.set_num_threads(cpu_threads)
    if torch.get_num_interop_threads() != 1:
        # PyTorch requires this before its first inter-op work. Call immediately
        # before constructing the first model, never during running inference.
        torch.set_num_interop_threads(1)
    # Ultralytics select_device('cpu') resets torch to its global NUM_THREADS.
    # Its documented torch.device argument preserves this explicit budget.
    return torch.device('cpu')


def center_from_surface(surface, camera_origin, diameter=.05, height=.135,
                        support_z=0., top_tolerance=.012):
    """Use known upright-cylinder geometry, distinguishing lid and side samples.

    For a side viewed near its silhouette midpoint, the visible surface is one
    radius closer than the cylinder axis in the horizontal viewing direction.
    A lid-center sample needs no radial offset. This is a bounded geometric
    approximation, not a fitted full 3-D shape or hand-eye calibration.
    """
    surface = np.asarray(surface, dtype=float)
    camera = np.asarray(camera_origin, dtype=float)
    if (surface.shape != (3,) or camera.shape != (3,)
            or not np.all(np.isfinite(np.r_[surface, camera, diameter, height, support_z]))
            or diameter <= 0 or height <= 0):
        raise ValueError('Invalid cylinder observation or dimensions')
    top = support_z+height
    if surface[2] < support_z+.01 or surface[2] > top+top_tolerance:
        raise ValueError('Depth sample is outside the assumed can vertical extent')
    center = surface.copy()
    center[2] = support_z+height/2
    if surface[2] >= top-top_tolerance:
        method = 'upright_lid_center'
    else:
        horizontal = surface[:2]-camera[:2]
        norm = np.linalg.norm(horizontal)
        if norm < diameter:
            raise ValueError('Side observation too close to vertical for radial correction')
        center[:2] += diameter/2*horizontal/norm
        method = 'upright_side_plus_radius'
    return center, method


def red_detection(bgr, minimum_pixels=20):
    """Experimental SIM_RED segmentation; returns a mask, never YOLO confidence."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 90, 45), (12, 255, 255))
    mask |= cv2.inRange(hsv, (168, 90, 45), (179, 255, 255))
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None, mask
    index = 1+int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, width, height, area = stats[index]
    if area < minimum_pixels or min(width, height) < 3:
        return None, mask
    chosen = (labels == index).astype(np.uint8)*255
    # A white printed label creates holes in an otherwise red can silhouette.
    # Fill enclosed holes using that observed contour, not a synthetic box or
    # simulator pose, so registered depth on the label remains usable.
    contours, _ = cv2.findContours(chosen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(chosen, contours, -1, 255, thickness=cv2.FILLED)
    return Detection('SIM_RED_color_proxy', 0., float(x), float(y),
                     float(x+width-1), float(y+height-1)), chosen


def check_support_plane(depth_m, camera_k, bbox, rotation, origin, support_z=0.,
                        tolerance=.02, min_depth=.05, max_depth=2.):
    """Check the assumed floor using measured background directly below the can.

    A tilted physical base with a planar odometry TF can otherwise produce a
    plausible-looking but wrong can-center height. Never compensate with an
    object ground-truth offset: reject this inconsistency instead.
    """
    height, width = depth_m.shape
    x0, _y0, x1, y1 = bbox
    first_row = int(math.ceil(y1))+8
    last_row = min(height-1, first_row+65)
    if last_row-first_row < 8:
        raise ValueError('Can base/floor not fully visible; support-plane assumption unverified')
    center_u = (x0+x1)/2
    half_width = max(15., (x1-x0)*.6)
    us = np.linspace(max(0, center_u-half_width), min(width-1, center_u+half_width), 7).astype(int)
    vs = np.linspace(first_row, last_row, 5).astype(int)
    uu, vv = np.meshgrid(us, vs)
    z = depth_m[vv, uu]
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    if np.count_nonzero(valid) < 12:
        raise ValueError('Insufficient depth below the can to verify support plane')
    k = np.asarray(camera_k)
    points = np.column_stack(((uu[valid]-k[2])*z[valid]/k[0],
                              (vv[valid]-k[5])*z[valid]/k[4], z[valid]))
    heights = (points@np.asarray(rotation).T+np.asarray(origin))[:, 2]
    median = float(np.median(heights))
    error = float(np.median(np.abs(heights-support_z)))
    if error > tolerance:
        raise ValueError(f'Support-plane/TF mismatch: observed median z={median:.4f}m, '
                         f'expected={support_z:.4f}m, error={error:.4f}m; base tilt/calibration must be checked')
    return dict(support_plane_median_z=median, support_plane_error_m=error,
                support_samples=int(np.count_nonzero(valid)))


def fit_lid_circle(points_xy, expected_radius=.025):
    """Fit a measured, nearly complete lid boundary; reject arcs and ellipses."""
    points = np.asarray(points_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 30 or not np.isfinite(points).all():
        raise ValueError('Insufficient finite measured lid points')
    hull = cv2.convexHull(points.astype(np.float32)).reshape(-1, 2).astype(float)
    if len(hull) < 8:
        raise ValueError('Measured lid boundary is incomplete')
    design = np.column_stack((2*hull, np.ones(len(hull))))
    fit = np.linalg.lstsq(design, np.sum(hull*hull, axis=1), rcond=None)[0]
    center = fit[:2]
    radius_squared = fit[2]+float(center@center)
    if radius_squared <= 0:
        raise ValueError('Measured lid circle is degenerate')
    radius = math.sqrt(radius_squared)
    residual = float(np.max(np.abs(np.linalg.norm(hull-center, axis=1)-radius)))
    angles = np.sort(np.arctan2(hull[:, 1]-center[1], hull[:, 0]-center[0]))
    largest_gap = float(np.max(np.diff(np.r_[angles, angles[0]+2*math.pi])))
    if abs(radius-expected_radius) > .004 or residual > .003 or largest_gap > math.pi/2:
        raise ValueError('Measured lid boundary does not verify a complete known-size circular can')
    return center, dict(lid_radius_m=radius, lid_max_residual_m=residual,
                        lid_boundary_samples=len(hull), lid_angular_coverage_rad=2*math.pi-largest_gap)


def fit_side_circle(points_xy, expected_radius=.025):
    """Measure a cylinder side arc; a partial colour label must not shift its centre."""
    points = np.asarray(points_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('Invalid measured cylinder side points')
    points = np.unique(np.round(points, 4), axis=0)
    if len(points) < 20:
        raise ValueError('Insufficient measured cylinder side points')
    mean = np.mean(points, axis=0)
    local = points-mean
    solution, _, rank, _ = np.linalg.lstsq(
        np.column_stack((2*local, np.ones(len(local)))), np.sum(local**2, axis=1), rcond=None)
    if rank != 3:
        raise ValueError('Degenerate measured cylinder arc')
    center = mean+solution[:2]
    radius = math.sqrt(max(0., solution[2]+sum(solution[:2]**2)))
    residual = float(np.quantile(np.abs(np.linalg.norm(points-center, axis=1)-radius), .95))
    angles = np.sort(np.arctan2(points[:,1]-center[1], points[:,0]-center[0]))
    coverage = 2*math.pi-float(np.max(np.diff(np.r_[angles, angles[0]+2*math.pi])))
    if abs(radius-expected_radius) > .004 or residual > .002 or coverage < math.pi/3:
        raise ValueError('Measured side does not verify the known can radius and sufficient arc')
    return center, dict(side_radius_m=radius, side_residual_95_m=residual,
                        side_samples=len(points), side_angular_coverage_rad=coverage)


def cylinder_from_depth(depth_m, camera_k, bbox, rotation, origin,
                        diameter=.05, height=.135, support_z=0.,
                        min_depth=.05, max_depth=2., mask=None):
    x0,y0,x1,y1 = bbox
    rows,cols = np.mgrid[max(0,int(y0)):min(depth_m.shape[0],int(y1)+1),
                        max(0,int(x0)):min(depth_m.shape[1],int(x1)+1)]
    z = depth_m[rows,cols]
    k = np.asarray(camera_k)
    optical = np.stack(((cols-k[2])*z/k[0], (rows-k[5])*z/k[4], z), axis=-1)
    points = optical@np.asarray(rotation).T+np.asarray(origin)
    valid = np.isfinite(z)&(z>=min_depth)&(z<=max_depth)
    valid &= (points[:,:,2]>support_z+.015)&(points[:,:,2]<support_z+height-.01)
    if mask is not None:
        valid &= mask[rows,cols]>0
    side = points[valid]
    center_xy, quality = fit_side_circle(side[:,:2], diameter/2)
    center = np.r_[center_xy,support_z+height/2]
    return center, np.median(side,axis=0), quality


def lid_from_red_seed(depth_m, camera_k, bbox, rotation, origin,
                      diameter=.05, height=.135, support_z=0., min_depth=.05, max_depth=2.):
    """Use the actual depth lid when a red crescent has no red center pixel.

    The red seed supplies a bounded image ROI. Points at the assumed top height
    must themselves verify a complete circle of the known diameter. No object
    pose or fabricated depth is used; partial caps and inconsistent shapes fail.
    """
    x0, y0, x1, y1 = bbox
    width, length = max(3., x1-x0), max(3., y1-y0)
    rows, cols = np.mgrid[max(0, int(y0-2*length)):min(depth_m.shape[0], int(y1+length)+1),
                         max(0, int(x0-width)):min(depth_m.shape[1], int(x1+width)+1)]
    depth = depth_m[rows, cols]
    valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    k = np.asarray(camera_k)
    optical = np.stack(((cols-k[2])*depth/k[0], (rows-k[5])*depth/k[4], depth), axis=-1)
    points = optical@np.asarray(rotation).T+np.asarray(origin)
    valid &= np.abs(points[:, :, 2]-(support_z+height)) <= .004
    lid = points[valid]
    center_xy, quality = fit_lid_circle(lid[:, :2], diameter/2)
    surface = np.r_[center_xy, np.median(lid[:, 2])]
    center = np.r_[center_xy, support_z+height/2]
    return center, surface, quality


class CanPerception(Node):
    def __init__(self):
        super().__init__('can_perception')
        defaults = {
            'model_path': '/home/user2/mobility_simulation/namo_ws/best.pt',
            'perception_mode': 'yolo', 'target_class': 'coke', 'confidence_threshold': .5,
            'cpu_threads': 2,
            'rgb_topic': '/realsense/d435i/color/image_raw',
            'depth_topic': '/realsense/d435i/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/realsense/d435i/color/camera_info',
            'camera_frame': 'd435i_color_optical_frame', 'target_frame': 'base_link',
            'support_frame': 'odom', 'support_plane_z': 0.,
            'support_plane_tolerance': .02,
            'can_diameter': .05, 'can_height': .135, 'depth_scale': .001,
            'min_depth': .05, 'max_depth': 2., 'max_frame_age': 1.,
            'inference_hz': 2., 'stable_samples': 3, 'stable_radius': .015,
            'evidence_dir': os.path.join(os.environ.get('NAMO_RUN_DIR', '/tmp/namo_perception'), 'perception'),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        numeric = ('inference_hz', 'max_frame_age', 'stable_radius', 'can_diameter',
                   'can_height', 'depth_scale', 'min_depth', 'max_depth')
        if (any(not math.isfinite(float(self.p(name))) or self.p(name) <= 0 for name in numeric)
                or self.p('min_depth') >= self.p('max_depth')
                or int(self.p('stable_samples')) < 1):
            raise ValueError('Invalid perception rate, stability, image depth, or can dimensions')
        self.mode = str(self.p('perception_mode')).lower()
        if self.mode not in ('yolo', 'sim_red'):
            raise ValueError('perception_mode must be yolo or explicit sim_red')
        self.model = None
        self.class_names = {}
        if self.mode == 'yolo':
            model_path = Path(self.p('model_path')).expanduser()
            if not model_path.is_file():
                raise FileNotFoundError(f'YOLO model does not exist: {model_path}')
            from ultralytics import YOLO
            self.inference_device = configure_torch_threads(self.p('cpu_threads'))
            self.model = YOLO(str(model_path))
            self.class_names = self.model.names
            if self.p('target_class') not in self.class_names.values():
                raise ValueError(f'Target class absent from model: {self.class_names}')
            self.get_logger().info(f'YOLO strict mode; classes={self.class_names}; '
                                   f'CPU threads={self.p("cpu_threads")}; no fallback')
        else:
            self.get_logger().warn('SIM_RED: explicit color segmentation experiment, NOT YOLO recognition')
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.rgb = message_filters.Subscriber(self, Image, self.p('rgb_topic'), qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, self.p('depth_topic'), qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb, self.depth], 5, .04)
        self.sync.registerCallback(self.on_pair)
        self.create_subscription(CameraInfo, self.p('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, '/namo/can_pose', durable)
        self.status_pub = self.create_publisher(String, '/namo/can_observation', durable)
        self.debug_pub = self.create_publisher(Image, '/namo/can_debug', 1)
        self.info = self.latest = None
        self.pairs = deque(maxlen=8)
        self.last_received = self.next_inference = self.last_evidence = 0.
        self.last_stamp = -1
        self.samples = []
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='can_yolo')
        self.future = None
        self.last_reason = ''
        self.last_status = {}
        self.evidence = Path(self.p('evidence_dir')) if self.p('evidence_dir') else None
        if self.evidence:
            self.evidence.mkdir(parents=True, exist_ok=True)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status(False, False, 'Waiting for synchronized wrist RGB-D and CameraInfo')

    def p(self, name):
        return self.get_parameter(name).value

    def on_info(self, msg):
        self.info = msg

    def on_pair(self, rgb, depth):
        self.latest = rgb, depth
        self.pairs.append(self.latest)
        self.last_received = time.monotonic()

    def publish_status(self, valid, stable, reason, **details):
        status = dict(valid=valid, stable=stable, mode=self.mode.upper(), reason=reason,
                      assumption='Single upright can with known dimensions/support plane', **details)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        if reason != self.last_reason:
            self.get_logger().info(f'{self.mode.upper()} perception: {reason}')
            self.last_reason = reason
        self.last_status = status
        if self.evidence:
            audit = dict(status, audit_wall_time=time.time(),
                         audit_sim_time=self.get_clock().now().nanoseconds*1e-9)
            with (self.evidence/'observations.jsonl').open('a') as stream:
                stream.write(json.dumps(audit, allow_nan=False)+'\n')

    def transform(self, destination, source, stamp):
        transform = self.tf.lookup_transform(destination, source, stamp,
                                             timeout=Duration(seconds=.15)).transform
        q = transform.rotation
        t = transform.translation
        quaternion = [q.x, q.y, q.z, q.w]
        return rotation_matrix(quaternion), np.array([t.x, t.y, t.z]), quaternion

    def observe(self, pair, info):
        rgb, depth = pair
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        if stamp.nanoseconds == 0 or age < -.05 or age > self.p('max_frame_age'):
            raise ValueError(f'RGB timestamp is stale/future: age={age:.3f}s')
        if rgb.header.frame_id != self.p('camera_frame') or depth.header.frame_id != rgb.header.frame_id:
            raise ValueError('Aligned RGB/depth frame differs from configured optical frame')
        if info.header.frame_id != rgb.header.frame_id or info.width != rgb.width or info.height != rgb.height:
            raise ValueError('CameraInfo frame or dimensions do not match RGB')
        if any(abs(float(value)) > 1e-9 for value in info.d):
            raise ValueError('Use rectified registered RGB-D; nonzero camera distortion is unsupported')
        bgr, distances = image_array(rgb), image_array(depth).astype(float)
        if distances.shape != bgr.shape[:2]:
            raise ValueError('Depth is not registered to color pixels')
        if depth.encoding.lower() == '16uc1':
            distances *= self.p('depth_scale')
        mask = None
        if self.mode == 'sim_red':
            detection, mask = red_detection(bgr)
        else:
            prediction = self.model.predict(bgr, conf=self.p('confidence_threshold'),
                                            imgsz=640, device=self.inference_device, verbose=False)[0]
            candidates = []
            for box in prediction.boxes:
                class_name = self.class_names[int(box.cls[0].item())]
                candidates.append(Detection(class_name, float(box.conf[0].item()),
                                            *map(float, box.xyxy[0].cpu().tolist())))
            detection = select_target_detection(candidates, self.p('target_class'), self.p('confidence_threshold'))
        if self.evidence and time.monotonic()-self.last_evidence > 2.:
            cv2.imwrite(str(self.evidence/'latest_rgb.png'), bgr)
            np.savez_compressed(self.evidence/'latest_rgbd.npz', rgb=bgr, depth_m=distances,
                                camera_k=np.asarray(info.k), stamp_ns=stamp.nanoseconds)
            self.last_evidence = time.monotonic()
        if detection is None:
            return dict(valid=False, reason='Target not detected; no fallback used', rgb=rgb, bgr=bgr)
        u, v = detection.center
        if not (0 <= u < rgb.width and 0 <= v < rgb.height):
            raise ValueError('Detected pixel center lies outside RGB image')
        half_width = max(1, min(3, int((detection.x2-detection.x1)*.15)))
        half_height = max(1, min(3, int((detection.y2-detection.y1)*.15)))
        x0, x1 = max(0, u-half_width), min(rgb.width, u+half_width+1)
        y0, y1 = max(0, v-half_height), min(rgb.height, v+half_height+1)
        patch = distances[y0:y1, x0:x1]
        valid = np.isfinite(patch) & (patch >= self.p('min_depth')) & (patch <= self.p('max_depth'))
        if mask is not None:
            valid &= mask[y0:y1, x0:x1] > 0
        rotation, origin, _ = self.transform(self.p('support_frame'), rgb.header.frame_id, stamp)
        support_check = check_support_plane(distances, info.k,
            [detection.x1, detection.y1, detection.x2, detection.y2], rotation, origin,
            self.p('support_plane_z'), self.p('support_plane_tolerance'),
            self.p('min_depth'), self.p('max_depth'))
        try:
            center, surface, quality = cylinder_from_depth(
                distances, info.k, [detection.x1,detection.y1,detection.x2,detection.y2],
                rotation, origin, self.p('can_diameter'), self.p('can_height'),
                self.p('support_plane_z'), self.p('min_depth'), self.p('max_depth'),mask)
            support_check.update(quality)
            depth_m = float((rotation.T@(surface-origin))[2])
            correction = 'observed_depth_cylinder_side_fit'
        except ValueError:
            if self.mode != 'sim_red':
                raise
            center, surface, lid_quality = lid_from_red_seed(
                distances, info.k, [detection.x1, detection.y1, detection.x2, detection.y2],
                rotation, origin, self.p('can_diameter'), self.p('can_height'), self.p('support_plane_z'),
                self.p('min_depth'), self.p('max_depth'))
            support_check.update(lid_quality)
            depth_m = float((rotation.T@(surface-origin))[2])
            correction = 'observed_depth_lid_circle_from_SIM_RED_seed'
        output_rotation, output_translation, quaternion = self.transform(
            self.p('target_frame'), self.p('support_frame'), stamp)
        return dict(valid=True, reason='Stamped RGB-D can center observed', rgb=rgb, bgr=bgr,
                    center=output_rotation@center+output_translation,
                    surface=output_rotation@surface+output_translation,
                    quaternion=quaternion, correction=correction, class_name=detection.class_name,
                    support_check=support_check,
                    confidence=detection.confidence if self.mode == 'yolo' else None,
                    bbox=[detection.x1, detection.y1, detection.x2, detection.y2], depth_m=depth_m)

    def consume(self, result):
        if not result['valid']:
            self.samples = []
            self.publish_status(False, False, result['reason'])
            self.publish_debug(result['rgb'], result['bgr'],
                               f'{self.mode.upper()} NO DETECTION | no fallback')
            return
        rgb = result['rgb']
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
        if (self.get_clock().now()-stamp).nanoseconds*1e-9 > self.p('max_frame_age'):
            self.samples = []
            self.publish_status(False, False, 'Inference result expired before publication')
            return
        point = result['center']
        if self.samples and any(np.linalg.norm(point-old) > self.p('stable_radius') for old in self.samples):
            self.samples = []
        self.samples.append(point)
        self.samples = self.samples[-int(self.p('stable_samples')):]
        stable = len(self.samples) >= self.p('stable_samples')
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = rgb.header.stamp, self.p('target_frame')
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
         pose.pose.orientation.w) = result['quaternion']
        if stable:
            self.pose_pub.publish(pose)
        details = dict(frame_id=self.p('target_frame'),
                       stamp={'sec': rgb.header.stamp.sec, 'nanosec': rgb.header.stamp.nanosec},
                       center_xyz=point.tolist(), surface_xyz=result['surface'].tolist(),
                       confidence=result['confidence'], class_name=result['class_name'],
                       correction=result['correction'], bbox=result['bbox'], depth_m=result['depth_m'],
                       stable_samples=len(self.samples), **result['support_check'])
        self.publish_status(True, stable, 'Stable can center' if stable else 'Collecting stable observations', **details)
        bgr = result['bgr'].copy()
        x0, y0, x1, y1 = map(int, result['bbox'])
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 200, 255), 2)
        self.publish_debug(rgb, bgr, f'{self.mode.upper()} {result["class_name"]} stable={stable}')
        if self.evidence:
            cv2.imwrite(str(self.evidence/'latest_detection.png'), bgr)
            (self.evidence/'observation.json').write_text(json.dumps(self.last_status, indent=2))
            cv2.imwrite(str(self.evidence/'latest_observed_rgb.png'), result['bgr'])
            if stable:
                cv2.imwrite(str(self.evidence/'last_stable_rgb.png'), result['bgr'])
                (self.evidence/'last_stable_observation.json').write_text(
                    json.dumps(self.last_status, indent=2))

    def publish_debug(self, rgb, bgr, label):
        cv2.putText(bgr, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 200, 255), 1)
        debug = Image()
        debug.header = rgb.header
        debug.height, debug.width = bgr.shape[:2]
        debug.encoding, debug.step, debug.data = 'bgr8', bgr.shape[1]*3, bgr.tobytes()
        self.debug_pub.publish(debug)

    def tick(self):
        if self.future is not None and self.future.done():
            try:
                self.consume(self.future.result())
            except Exception as exc:
                self.samples = []
                self.publish_status(False, False, str(exc))
            self.future = None
        now = time.monotonic()
        if now-self.last_received > self.p('max_frame_age'):
            if self.last_status.get('valid') or self.last_status.get('reason') != 'Wrist RGB-D stream missing or stale':
                self.samples = []
                self.publish_status(False, False, 'Wrist RGB-D stream missing or stale')
            return
        if self.future is not None or self.latest is None or self.info is None or now < self.next_inference:
            return
        # Sensor messages can precede their /clock tick and arm TF by one
        # publication interval. Select a recent *stamped* pair whose transforms
        # have arrived, rather than treating this ordinary latency as a lost
        # detection and resetting the stable sequence.
        pair = None
        clock_ns = self.get_clock().now().nanoseconds
        for candidate in reversed(self.pairs):
            at = rclpy.time.Time.from_msg(candidate[0].header.stamp)
            if at.nanoseconds <= self.last_stamp or not 0 <= (clock_ns-at.nanoseconds)*1e-9 <= self.p('max_frame_age'):
                continue
            if all(self.tf.can_transform(self.p(frame), candidate[0].header.frame_id, at)
                   for frame in ('support_frame', 'target_frame')):
                pair = candidate
                break
        if pair is None:
            previous = self.last_status.get('stamp', {})
            age = clock_ns*1e-9-previous.get('sec',0)-previous.get('nanosec',0)*1e-9
            if self.last_status.get('valid') and age > self.p('max_frame_age'):
                self.samples = []
                self.publish_status(False, False, 'Stamped camera transforms missing or stale')
            return
        self.last_stamp = rclpy.time.Time.from_msg(pair[0].header.stamp).nanoseconds
        self.next_inference = now+1./self.p('inference_hz')
        self.future = self.worker.submit(self.observe, pair, self.info)

    def close(self):
        self.worker.shutdown(wait=True, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = CanPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
