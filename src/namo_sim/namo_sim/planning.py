"""Conservative static-map A* and p=1 NAMO time comparison (no ROS dependency).

Grid data follows nav_msgs/OccupancyGrid: data[row=y, column=x], bottom row
first; -1 is unknown and blocked. The origin is the bottom-left corner of
cell (0, 0), including its yaw. The input map is copied and never modified.
Both planners use a 0.28 m circular footprint. This grid planner adds half a
cell diagonal when measuring distance from occupied *cell centers*, so an
occupied cell's area is covered, not merely its center. Unknown and outside
map space are occupied. Dynamic overlays affect only the requested copy.
"""
from dataclasses import dataclass
import heapq
import math
from typing import Optional

import numpy as np
from scipy.ndimage import distance_transform_edt


class PlanError(RuntimeError):
    """Planning failure with a machine-readable reason; never a fake path."""

    def __init__(self, code, detail=''):
        self.code = code
        super().__init__(f'{code}: {detail}' if detail else code)


class Grid:
    def __init__(self, data, resolution, origin=(0.0, 0.0, 0.0), occupied_threshold=65):
        self.data = np.array(data, dtype=float, copy=True)
        self.resolution = float(resolution)
        self.origin = tuple(float(v) for v in origin)
        if (self.data.ndim != 2 or not self.data.size or not np.isfinite(self.data).all()
                or not math.isfinite(self.resolution) or self.resolution <= 0
                or len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin)
                or not 0 < occupied_threshold <= 100):
            raise PlanError('INVALID_MAP', 'finite 2D occupancy, positive resolution, and valid origin required')
        self.height, self.width = self.data.shape
        self.occupied_threshold = occupied_threshold
        self.blocked = (self.data < 0) | (self.data >= occupied_threshold)
        self.data.flags.writeable = False
        self.blocked.flags.writeable = False
        self._cos, self._sin = math.cos(self.origin[2]), math.sin(self.origin[2])
        self._inflated_cache = {}

    def world_to_local(self, point):
        dx, dy = float(point[0])-self.origin[0], float(point[1])-self.origin[1]
        return self._cos*dx+self._sin*dy, -self._sin*dx+self._cos*dy

    def world_to_cell(self, point):
        """Return (column, row); out-of-map values are intentionally not clipped."""
        x, y = self.world_to_local(point)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('Non-finite world point')
        return math.floor(x/self.resolution), math.floor(y/self.resolution)

    def cell_to_world(self, cell):
        """Return the center of a (column, row) cell, with origin yaw applied."""
        x, y = (cell[0]+0.5)*self.resolution, (cell[1]+0.5)*self.resolution
        return (self.origin[0]+self._cos*x-self._sin*y,
                self.origin[1]+self._sin*x+self._cos*y)

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def inflated(self, radius=0.28, extra_points=()):
        """Boolean occupied mask; caller-owned result, never an edit to static data."""
        radius = float(radius)
        if not math.isfinite(radius) or radius < 0:
            raise ValueError('Footprint radius must be finite and nonnegative')
        extra_points = tuple(extra_points)
        if not extra_points and radius in self._inflated_cache:
            return self._inflated_cache[radius].copy()
        blocked = self.blocked.copy()
        for point in extra_points:
            cell = self.world_to_cell(point)
            if self.in_bounds(cell):
                blocked[cell[1], cell[0]] = True
        if radius:
            distances = distance_transform_edt(np.pad(~blocked, 1, constant_values=False))[1:-1, 1:-1]
            occupied = distances*self.resolution <= radius+self.resolution/math.sqrt(2)
        else:
            occupied = blocked
        if not extra_points:
            self._inflated_cache[radius] = occupied.copy()
        return occupied

    def is_free(self, point, radius=0.28, extra_points=()):
        try:
            cell = self.world_to_cell(point)
        except (ValueError, TypeError, IndexError):
            return False
        return self.in_bounds(cell) and not self.inflated(radius, extra_points)[cell[1], cell[0]]

    def distance_to_static(self, point):
        """Distance to occupied cell area, conservative; outside map returns zero."""
        cell = self.world_to_cell(point)
        if not self.in_bounds(cell):
            return 0.0
        if not hasattr(self, '_distance'):
            self._distance = distance_transform_edt(np.pad(~self.blocked, 1, constant_values=False))[1:-1, 1:-1]
        return max(0.0, float(self._distance[cell[1], cell[0]])*self.resolution
                   - self.resolution/math.sqrt(2))

    def astar(self, start, goal, radius=0.28, extra_points=()):
        """Eight-connected A*, exact endpoints, no diagonal corner cutting.

        Raises PlanError with INVALID_START/INVALID_GOAL/NO_PATH. An empty path
        is never returned. Additional observed obstacles remain in the copy;
        callers must omit only the explicitly selected removable target.
        """
        occupied = self.inflated(radius, extra_points)
        start = (float(start[0]), float(start[1]))
        goal = (float(goal[0]), float(goal[1]))
        for point, code in ((start, 'INVALID_START'), (goal, 'INVALID_GOAL')):
            try:
                cell = self.world_to_cell(point)
            except ValueError as exc:
                raise PlanError(code, str(exc)) from exc
            if not self.in_bounds(cell) or occupied[cell[1], cell[0]]:
                raise PlanError(code, f'footprint intersects occupancy at {point}')
        source, target = self.world_to_cell(start), self.world_to_cell(goal)
        if source == target:
            return [start, goal] if start != goal else [start]
        costs = {source: 0.0}
        parents = {}
        queue = [(math.dist(source, target), 0.0, source)]
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > costs[current]+1e-12:
                continue
            if current == target:
                cells = [current]
                while current != source:
                    current = parents[current]
                    cells.append(current)
                cells.reverse()
                points = [start] + [self.cell_to_world(c) for c in cells] + [goal]
                # Collinear compression preserves length and the checked cell route.
                output = []
                for point in points:
                    if output and math.dist(output[-1], point) < 1e-12:
                        continue
                    while len(output) >= 2:
                        a, b = output[-2], output[-1]
                        cross = (b[0]-a[0])*(point[1]-b[1])-(b[1]-a[1])*(point[0]-b[0])
                        dot = (b[0]-a[0])*(point[0]-b[0])+(b[1]-a[1])*(point[1]-b[1])
                        if abs(cross) < 1e-12 and dot >= 0:
                            output.pop()
                        else:
                            break
                    output.append(point)
                return output
            cx, cy = current
            for dx, dy in directions:
                neighbor = (cx+dx, cy+dy)
                if not self.in_bounds(neighbor) or occupied[cy+dy, cx+dx]:
                    continue
                if dx and dy and (occupied[cy, cx+dx] or occupied[cy+dy, cx]):
                    continue
                new_cost = cost+math.hypot(dx, dy)
                if new_cost+1e-12 >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = new_cost
                parents[neighbor] = current
                heapq.heappush(queue, (new_cost+math.dist(neighbor, target), new_cost, neighbor))
        raise PlanError('NO_PATH', 'no footprint-safe static-map route')


def path_length(points):
    """Sum consecutive Euclidean distances; empty/invalid paths are errors."""
    points = list(points)
    if not points:
        raise ValueError('Empty path')
    if not all(len(p) >= 2 and math.isfinite(p[0]) and math.isfinite(p[1]) for p in points):
        raise ValueError('Invalid path point')
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


@dataclass(frozen=True)
class Decision:
    action: str
    t_remove: Optional[float]
    t_detour: Optional[float]
    reason: str


def choose_action(clear_length, detour_length, removable=True, v_eff=0.2,
                  mock_remove=22.0, switch=0.0, margin=2.0, data_ready=True):
    """p=1 decision. None length means *confirmed* unavailable route.

    Service errors, timeout, unready planner, empty paths, and missing TF must
    set data_ready=False; they cannot be treated as evidence of no route.
    Effective speed includes ordinary travel/turn losses; switch is only
    additional delay before detour movement, avoiding duplicate turn costs.
    """
    if not data_ready:
        return Decision('STOP', None, None, 'Required planning/observation data unavailable')
    values = (v_eff, mock_remove, switch, margin)
    if not all(math.isfinite(v) for v in values) or v_eff <= 0 or min(values[1:]) < 0:
        return Decision('STOP', None, None, 'Invalid time-model parameters')
    if any(length is not None and (not math.isfinite(length) or length < 0)
           for length in (clear_length, detour_length)):
        return Decision('STOP', None, None, 'Invalid path length')
    tr = None if clear_length is None else mock_remove+clear_length/v_eff
    td = None if detour_length is None else switch+detour_length/v_eff
    if removable and tr is not None and (td is None or tr+margin < td):
        reason = 'Only removal route is feasible' if td is None else 'T_remove + margin < T_detour'
        return Decision('REMOVE', tr, td, reason)
    if td is not None:
        reason = 'Target removal not permitted' if not removable else 'Detour wins or removal advantage is within margin'
        return Decision('DETOUR', tr, td, reason)
    return Decision('STOP', tr, td, 'No permitted feasible route')
