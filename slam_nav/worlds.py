"""
The two worlds of day 7, shared by every node of this package.

One source of truth: the simulator, the laser and the visualisation all use
the walls defined here. Unlike day 6 there is NO map file of these worlds.
The robot has to draw the map itself with SLAM.

loop   the square corridor of slide 8: 4 m sides on the centre line, 1.1 m wide.
       Drive once around and 2 m further to see the loop closure.

    y
    5.1 +-------------------------------+
        | start (o->                    |
    4.0 |      +-------------------+    |
        |      |                   |    |
        |      |    inner block    |    |
        |      |                   |    |
    1.1 |      +-------------------+    |
        |                               |
      0 +-------------------------------+
        0     1.1                 4.0  5.1  x

room   the room of day 6, slide 11 (package mcl_room), with the inner wall
       and the box. Small enough to map in one minute.

    y
    3.4 +-----------------------------+---------------------+
        |                             |                     |
        |                             |  inner wall         |
    1.8 |                             +                     |
        |                                                   |
        |       robot                          +-------+    |
    1.2 |        (o->                          |  box  |    |
    0.6 |                                      +-------+    |
      0 +---------------------------------------------------+
        0                            3.0      4.2     5.0  5.6   x

All lengths are in metres, in the frame "world". The robot starts at the
start pose of its world. SLAM puts its map frame where the robot starts,
so the simulator publishes the static tf map -> world (see drive_sim.py).
"""
import math
from typing import Dict, List, Optional, Tuple

Point = Tuple[float, float]
Segment = Tuple[Point, Point]
Rect = Tuple[float, float, float, float]   # x_min, y_min, x_max, y_max
Pose = Tuple[float, float, float]          # x, y, yaw

ROBOT_RADIUS = 0.17   # m, the robot of day 6


def rect_walls(r: Rect) -> List[Segment]:
    """Return the four sides of a rectangle as wall segments."""
    x0, y0, x1, y1 = r
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


class World:
    """A rectangular area with solid blocks and thin walls inside."""

    def __init__(self, name: str, width: float, height: float,
                 solids: List[Rect], thin_walls: List[Segment],
                 start: Pose, tour: List[Pose]) -> None:
        self.name = name
        self.width = width
        self.height = height
        self.solids = solids          # blocks the robot cannot enter
        self.walls = rect_walls((0.0, 0.0, width, height)) + thin_walls
        for s in solids:
            self.walls += rect_walls(s)
        self.start = start            # x, y, yaw where the robot starts
        self.tour = tour              # Nav2 goals for the node "tour"

    def raycast(self, x: float, y: float, angle: float,
                max_range: float) -> Optional[float]:
        """
        Return the distance to the first wall along a ray, or None.

        Ray:      p = (x, y) + t * (cos a, sin a),  t >= 0
        Segment:  q = a + u * (b - a),             0 <= u <= 1
        Solving p = q for t and u is a 2 x 2 linear system (Cramer's rule).
        """
        dx, dy = math.cos(angle), math.sin(angle)
        best = None
        for (ax, ay), (bx, by) in self.walls:
            ex, ey = bx - ax, by - ay
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:      # ray parallel to the wall
                continue
            rx, ry = ax - x, ay - y
            t = (rx * ey - ry * ex) / denom
            u = (rx * dy - ry * dx) / denom
            if t > 1e-9 and 0.0 <= u <= 1.0 and (best is None or t < best):
                best = t
        if best is None or best > max_range:
            return None
        return best

    def distance_to_walls(self, px: float, py: float) -> float:
        """Return the shortest distance from a point to any wall."""
        return min(distance_to_segment(px, py, seg) for seg in self.walls)

    def inside(self, px: float, py: float) -> bool:
        """Check that the point lies inside the outer walls and outside every block."""
        if not (0.0 < px < self.width and 0.0 < py < self.height):
            return False
        return not any(x0 <= px <= x1 and y0 <= py <= y1
                       for x0, y0, x1, y1 in self.solids)

    def is_free(self, px: float, py: float, radius: float = ROBOT_RADIUS) -> bool:
        """Check that a robot of the given radius fits at this point."""
        return self.inside(px, py) and self.distance_to_walls(px, py) > radius


def distance_to_segment(px: float, py: float, seg: Segment) -> float:
    """Shortest distance from a point to a line segment."""
    (ax, ay), (bx, by) = seg
    vx, vy = bx - ax, by - ay
    t = ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


# The square corridor of slide 8. Centre line: square from (0.55, 0.55) to
# (4.55, 4.55), 4 m sides. Walls 0.55 m on each side: corridor 1.1 m wide.
LOOP = World(
    name='loop', width=5.1, height=5.1,
    solids=[(1.1, 1.1, 4.0, 4.0)],
    thin_walls=[],
    start=(0.55, 4.55, 0.0),
    # clockwise like slide 8: east, south, west, north, then 2 m east again
    tour=[(4.55, 4.55, -math.pi / 2), (4.55, 0.55, math.pi),
          (0.55, 0.55, math.pi / 2), (0.55, 4.55, 0.0), (2.55, 4.55, 0.0)],
)

# The room of day 6, slide 11: walls, inner wall and box as in mcl_room/room.py.
ROOM = World(
    name='room', width=5.6, height=3.4,
    solids=[(4.2, 0.6, 5.0, 1.2)],                 # the box
    thin_walls=[((3.0, 3.4), (3.0, 1.8))],         # inner wall from the top
    start=(1.3, 1.3, 0.0),
    # round trip through both halves of the room and back to the start
    tour=[(2.4, 2.8, math.pi / 2), (0.6, 2.8, math.pi), (0.6, 0.6, -math.pi / 2),
          (3.6, 0.5, 0.0), (5.2, 2.6, math.pi / 2), (3.6, 2.8, math.pi),
          (3.6, 1.4, -math.pi / 2), (1.3, 1.3, math.pi)],
)

WORLDS: Dict[str, World] = {'loop': LOOP, 'room': ROOM}


def get_world(name: str) -> World:
    """Look up a world by name, with a clear error for a typo."""
    if name not in WORLDS:
        raise ValueError(f'unknown world "{name}", use one of {sorted(WORLDS)}')
    return WORLDS[name]


def world_to_map(world: World, x: float, y: float, yaw: float) -> Pose:
    """
    Convert a pose from frame world to frame map.

    SLAM starts its map frame at the pose where the robot was switched on,
    which is world.start. So map = start pose, and a world pose seen from
    there is: rotate (x - x0, y - y0) by -yaw0, subtract yaw0.
    """
    x0, y0, yaw0 = world.start
    dx, dy = x - x0, y - y0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - yaw0))


def wrap(angle: float) -> float:
    """Wrap an angle to the range -pi .. +pi."""
    return math.atan2(math.sin(angle), math.cos(angle))
