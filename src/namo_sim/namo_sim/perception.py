"""LiDAR geometry only: no scenario labels, simulator models, or object truth.

The ROS caller validates LaserScan range bounds, transforms at the message stamp,
and supplies map-frame points. Infinite returns may be converted to range_max
endpoints ONLY for clearance_evidence; they are never obstacle points.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree


def _points(value):
    data = np.asarray(value, dtype=float)
    if data.size == 0:
        return np.empty((0, 2), dtype=float)
    data = data.reshape((-1, 2))
    return data[np.isfinite(data).all(axis=1)]


def _cluster(points):
    lo, hi = points.min(axis=0), points.max(axis=0)
    return {'center': tuple(points.mean(axis=0)), 'points': points.copy(),
            'bounds': (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))}


def _path_distances(points, path):
    """Distances to continuous segments and closest-segment unit tangents."""
    starts, deltas = path[:-1], np.diff(path, axis=0)
    lengths2 = np.sum(deltas * deltas, axis=1)
    valid = lengths2 > 1e-12
    starts, deltas, lengths2 = starts[valid], deltas[valid], lengths2[valid]
    if len(starts) == 0:
        return None, None
    offsets = points[:, None, :] - starts[None, :, :]
    t = np.clip(np.sum(offsets * deltas[None, :, :], axis=2) / lengths2, 0, 1)
    nearest = starts[None, :, :] + t[:, :, None] * deltas[None, :, :]
    distances2 = np.sum((points[:, None, :] - nearest) ** 2, axis=2)
    indices = np.argmin(distances2, axis=1)
    distances = np.sqrt(distances2[np.arange(len(points)), indices])
    tangents = deltas[indices] / np.sqrt(lengths2[indices])[:, None]
    return distances, tangents


class ScanDetector:
    """Persistent map-novel cluster intersecting the robot's candidate route.

    update() returns target=None until consecutive observations confirm it.
    Once confirmed, target remains available when occluded, with visible=False;
    disappearance alone therefore cannot be confused with successful removal.
    Call reset() only for a new mission. Clusters do not imply object identity or
    manipulability: those are separate experiment assumptions.
    """

    def __init__(self, grid, robot_radius=0.28, wall_tolerance=0.13,
                 cluster_distance=0.16, min_points=5, min_span=0.25,
                 persistence=3, match_distance=0.30):
        self.grid = grid
        self.robot_radius = float(robot_radius)
        self.wall_tolerance = float(wall_tolerance)
        self.cluster_distance = float(cluster_distance)
        self.min_points = int(min_points)
        self.min_span = float(min_span)
        self.persistence = int(persistence)
        self.match_distance = float(match_distance)
        if min(self.robot_radius, self.cluster_distance, self.min_points,
               self.persistence, self.match_distance) <= 0:
            raise ValueError('Detector sizes and persistence must be positive')
        self.reset()

    def reset(self):
        self.target = None
        self._candidate_center = None
        self._streak = 0

    def _clusters(self, points):
        if len(points) < self.min_points:
            return []
        parents = list(range(len(points)))

        def find(index):
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        for left, right in cKDTree(points).query_pairs(self.cluster_distance):
            parents[find(right)] = find(left)
        groups = {}
        for index in range(len(points)):
            groups.setdefault(find(index), []).append(index)
        return [_cluster(points[indices]) for indices in groups.values()
                if len(indices) >= self.min_points]

    def _blocks(self, cluster, path):
        if len(path) < 2:
            return False
        distances, _ = _path_distances(cluster['points'], path)
        if distances is None or float(distances.min()) > self.robot_radius:
            return False
        _, tangent = _path_distances(np.asarray([cluster['center']]), path)
        normal = np.array([-tangent[0, 1], tangent[0, 0]])
        projected = cluster['points'] @ normal
        return float(np.ptp(projected)) >= self.min_span

    def update(self, points_world, reference_path):
        points = _points(points_world)
        # Includes unknown/out-of-map rejection and conservative cell-area
        # distance rather than distance only to occupied cell centers.
        novel = np.asarray([p for p in points
                            if self.grid.distance_to_static(p) > self.wall_tolerance],
                           dtype=float).reshape((-1, 2))
        clusters = self._clusters(novel)
        path = _points(reference_path)
        selected = None
        if self.target is not None:
            candidates = [(math.dist(c['center'], self.target['center']), i)
                          for i, c in enumerate(clusters)]
            if candidates:
                distance, index = min(candidates)
                if distance <= self.match_distance:
                    selected = index
                    self.target = {**clusters[index], 'visible': True,
                                   'observations': self.target['observations'] + 1}
            if selected is None:
                self.target = {**self.target, 'visible': False}
        else:
            blocking = [i for i, c in enumerate(clusters) if self._blocks(c, path)]
            if blocking:
                index = min(blocking, key=lambda i: math.dist(clusters[i]['center'], path[0]))
                center = clusters[index]['center']
                self._streak = self._streak + 1 if (
                    self._candidate_center is not None and
                    math.dist(center, self._candidate_center) <= self.match_distance) else 1
                self._candidate_center = center
                selected = index
                if self._streak >= self.persistence:
                    self.target = {**clusters[index], 'visible': True,
                                   'observations': self._streak}
            else:
                self._candidate_center = None
                self._streak = 0
        return {'target': self.target,
                'other_clusters': [c for i, c in enumerate(clusters) if i != selected],
                'novel_points': novel}


def clearance_evidence(origin_xy, endpoints_world, target_bounds, margin=0.20,
                       min_rays=5, bounds_padding=0.025):
    """Require live rays through the former target, continuing beyond it.

    origin_xy is the transformed scan origin; endpoints_world are valid finite
    returns plus +inf returns represented by range_max endpoints. NaN/invalid
    ranges must be omitted by caller. Bounds order is xmin,ymin,xmax,ymax.
    A nearer return along a target-directed ray counts as blocked, including an
    occluder before the old obstacle. No data or mere missing points never clear.
    The caller must additionally require fresh stamps and fresh costmap clearance.
    """
    origin = np.asarray(origin_xy, dtype=float)
    bounds = np.asarray(target_bounds, dtype=float)
    if origin.shape != (2,) or bounds.shape != (4,) or not (
            np.isfinite(origin).all() and np.isfinite(bounds).all()):
        return {'clear': False, 'through_rays': 0, 'blocked_rays': 0}
    if min_rays < 1 or margin < 0 or bounds_padding < 0:
        raise ValueError('Invalid clearance thresholds')
    lower = bounds[:2] - bounds_padding
    upper = bounds[2:] + bounds_padding
    if np.any(lower >= upper):
        return {'clear': False, 'through_rays': 0, 'blocked_rays': 0}
    through = blocked = 0
    for endpoint in _points(endpoints_world):
        delta = endpoint - origin
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            continue
        direction = delta / distance
        enter, leave = 0.0, math.inf
        for axis in (0, 1):
            if abs(direction[axis]) < 1e-10:
                if not lower[axis] <= origin[axis] <= upper[axis]:
                    leave = -math.inf
                    break
            else:
                a = (lower[axis] - origin[axis]) / direction[axis]
                b = (upper[axis] - origin[axis]) / direction[axis]
                enter = max(enter, min(a, b))
                leave = min(leave, max(a, b))
        if leave < enter or leave <= 0 or not math.isfinite(leave):
            continue
        if distance >= leave + margin:
            through += 1
        else:
            blocked += 1
    return {'clear': through >= min_rays and blocked == 0,
            'through_rays': through, 'blocked_rays': blocked}
