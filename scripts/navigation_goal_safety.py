"""ROS-independent, orientation-aware goal footprint checks.

Grid values follow nav_msgs/OccupancyGrid, not raw Costmap2D bytes. Inflated
costs remain preferences: actual occupied and unknown cells must be separated
from the full robot polygon by ``safety_margin`` metres. This checks a goal
pose only; the navigation planner remains responsible for checking the route.
"""
import ast
import math
from dataclasses import dataclass

DEFAULT_FOOTPRINT = ((0.042, 0.085), (0.042, -0.085),
                     (-0.147, -0.085), (-0.147, 0.085))
_EPS = 1e-10


@dataclass(frozen=True)
class GoalSafetyResult:
    safe: bool
    reason: str
    cell: object = None


def parse_footprint(value):
    """Accept a ROS footprint parameter represented by a list or string."""
    if isinstance(value, str):
        value = ast.literal_eval(value)
    polygon = tuple((float(p[0]), float(p[1])) for p in value)
    if len(polygon) < 3 or not all(math.isfinite(v) for p in polygon for v in p):
        raise ValueError('footprint must have at least three finite vertices')
    area = sum(a[0] * b[1] - b[0] * a[1]
               for a, b in zip(polygon, polygon[1:] + polygon[:1]))
    if abs(area) <= _EPS:
        raise ValueError('footprint has zero area')
    return polygon


def quaternion_yaw(q):
    """Read a geometry_msgs-style quaternion, rejecting invalid poses."""
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('orientation contains non-finite values')
    norm = math.sqrt(sum(v * v for v in values))
    if norm <= _EPS:
        raise ValueError('orientation has zero norm')
    x, y, z, w = (v / norm for v in values)
    if abs(x) > 1e-4 or abs(y) > 1e-4:
        raise ValueError('navigation orientation must be planar')
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 <= _EPS else max(0.0, min(1.0,
        ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segment_distance(a, b, c, d):
    if (_cross(a, b, c) * _cross(a, b, d) < 0 and
            _cross(c, d, a) * _cross(c, d, b) < 0):
        return 0.0
    return min(_point_segment_distance(a, c, d),
               _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b),
               _point_segment_distance(d, a, b))


def _contains(polygon, point):
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if _point_segment_distance(point, a, b) <= _EPS:
            return True
        if (a[1] > point[1]) != (b[1] > point[1]):
            crossing = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if point[0] < crossing:
                inside = not inside
    return inside


def _polygon_distance(first, second):
    if _contains(first, second[0]) or _contains(second, first[0]):
        return 0.0
    return min(_segment_distance(a, b, c, d)
               for a, b in zip(first, first[1:] + first[:1])
               for c, d in zip(second, second[1:] + second[:1]))


class GridFootprintChecker:
    def __init__(self, width, height, resolution, data, origin=(0.0, 0.0, 0.0),
                 footprint=DEFAULT_FOOTPRINT, safety_margin=0.04,
                 lethal_threshold=100):
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin = tuple(float(v) for v in origin)
        self.footprint = parse_footprint(footprint)
        self.safety_margin = float(safety_margin)
        self.lethal_threshold = int(lethal_threshold)
        self.data = data
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0 or
                not math.isfinite(self.resolution) or
                len(data) != self.width * self.height):
            raise ValueError('invalid occupancy grid dimensions')
        if len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError('invalid grid origin')
        if not math.isfinite(self.safety_margin) or self.safety_margin < 0:
            raise ValueError('safety margin must be finite and nonnegative')
        if not 1 <= self.lethal_threshold <= 100:
            raise ValueError('lethal threshold must be an OccupancyGrid value')

    @classmethod
    def from_message(cls, message, footprint=DEFAULT_FOOTPRINT,
                     safety_margin=0.04, lethal_threshold=100):
        info = message.info
        return cls(info.width, info.height, info.resolution, message.data,
                   (info.origin.position.x, info.origin.position.y,
                    quaternion_yaw(info.origin.orientation)), footprint,
                   safety_margin, lethal_threshold)

    def check_pose(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return GoalSafetyResult(False, 'NONFINITE_POSE')
        # Work in grid-local metric coordinates, including rotated origins.
        ox, oy, origin_yaw = self.origin
        c, s = math.cos(origin_yaw), math.sin(origin_yaw)
        gx, gy = c * (x - ox) + s * (y - oy), -s * (x - ox) + c * (y - oy)
        c, s = math.cos(yaw - origin_yaw), math.sin(yaw - origin_yaw)
        polygon = tuple((gx + c * px - s * py, gy + s * px + c * py)
                        for px, py in self.footprint)
        margin, resolution = self.safety_margin, self.resolution
        low_x = min(p[0] for p in polygon) - margin
        low_y = min(p[1] for p in polygon) - margin
        high_x = max(p[0] for p in polygon) + margin
        high_y = max(p[1] for p in polygon) + margin
        if (low_x <= _EPS or low_y <= _EPS or
                high_x >= self.width * resolution - _EPS or
                high_y >= self.height * resolution - _EPS):
            return GoalSafetyResult(False, 'OUTSIDE_MAP_OR_MARGIN')
        # Include cells on either side of an exact boundary: touching counts.
        for row in range(math.floor((low_y - _EPS) / resolution),
                         math.floor((high_y + _EPS) / resolution) + 1):
            for col in range(math.floor((low_x - _EPS) / resolution),
                             math.floor((high_x + _EPS) / resolution) + 1):
                value = self.data[row * self.width + col]
                if 0 <= value < self.lethal_threshold:
                    continue
                left, bottom = col * resolution, row * resolution
                cell = ((left, bottom), (left + resolution, bottom),
                        (left + resolution, bottom + resolution),
                        (left, bottom + resolution))
                if _polygon_distance(polygon, cell) <= margin + _EPS:
                    return GoalSafetyResult(False,
                        'UNKNOWN_OR_MARGIN' if value < 0 else 'OBSTACLE_OR_MARGIN',
                        (col, row))
        return GoalSafetyResult(True, 'CLEAR')
