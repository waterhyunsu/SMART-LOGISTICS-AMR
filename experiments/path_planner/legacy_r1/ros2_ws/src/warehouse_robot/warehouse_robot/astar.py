"""A* path planning on a grid map (no ROS dependency, numpy only).

1. Compute the distance to obstacles; cells within the robot radius are impassable (lethal).
2. From the radius out to inflation_radius, add cost the closer to a wall, to keep to the middle of aisles.
3. A* search with 8-connected moves and an octile heuristic.
   algorithm='dijkstra' runs the same search with the heuristic set to 0 (same grid, costs, neighbours and
   tie-breaking), so the two differ only in the heuristic. Every cell cost factor is >= 1, so the octile
   heuristic is admissible and consistent: both return a minimum-cost path (the path itself can differ on ties).
   Search statistics of the last plan are kept in GridPlanner.last_stats (research 1: A* vs Dijkstra).
"""
import heapq
import math
import time

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except Exception:  # environments without scipy or with a mismatched numpy (e.g. Raspberry Pi)
    distance_transform_edt = None

SQRT2 = math.sqrt(2.0)
# (drow, dcol, move distance)
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
ESCAPE_PENALTY = 20.0  # cost of leaving the lethal area when the robot starts against a wall
ALGORITHMS = ('astar', 'dijkstra')


class PlanningError(Exception):
    pass


def path_length(points):
    """Total length (m) of a polyline [(x, y), ...]."""
    return float(sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:])))


def obstacle_distance(obstacle, resolution, max_dist):
    """Distance (m) from each cell to the nearest obstacle."""
    if not obstacle.any():
        return np.full(obstacle.shape, np.inf)
    if distance_transform_edt is not None:
        return distance_transform_edt(~obstacle) * resolution
    # without scipy, approximate by repeated dilation (up to max_dist)
    dist = np.full(obstacle.shape, np.inf)
    dist[obstacle] = 0.0
    reached = obstacle.copy()
    for k in range(1, int(math.ceil(max_dist / resolution)) + 1):
        grown = reached.copy()
        grown[1:, :] |= reached[:-1, :]
        grown[:-1, :] |= reached[1:, :]
        grown[:, 1:] |= reached[:, :-1]
        grown[:, :-1] |= reached[:, 1:]
        new = grown & ~reached
        dist[new] = k * resolution
        reached = grown
    return dist


def pad_grid(data, width, height, resolution, origin_x, origin_y, points, margin=1.0):
    """Grow the grid with unknown (-1) cells until all points (world coords) fit (for goals still outside the map during SLAM).

    Returns: (data 1-D array, width, height, origin_x, origin_y)
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    cells = int(math.ceil(margin / resolution))
    left = max(0, int(math.ceil((origin_x - min(xs)) / resolution)) + cells) if min(xs) < origin_x else 0
    bottom = max(0, int(math.ceil((origin_y - min(ys)) / resolution)) + cells) if min(ys) < origin_y else 0
    right_edge = origin_x + width * resolution
    top_edge = origin_y + height * resolution
    right = int(math.ceil((max(xs) - right_edge) / resolution)) + cells if max(xs) >= right_edge else 0
    top = int(math.ceil((max(ys) - top_edge) / resolution)) + cells if max(ys) >= top_edge else 0
    if not (left or bottom or right or top):
        return grid.ravel(), width, height, origin_x, origin_y
    out = np.full((height + bottom + top, width + left + right), -1, dtype=np.int16)
    out[bottom:bottom + height, left:left + width] = grid
    return (out.ravel(), out.shape[1], out.shape[0],
            origin_x - left * resolution, origin_y - bottom * resolution)


class GridPlanner:
    """A* planner built from OccupancyGrid data.

    data: length width*height, row 0 = map bottom (same order as OccupancyGrid), values -1 or 0..100
    """

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 robot_radius=0.22, inflation_radius=0.55, cost_weight=3.0,
                 occupied_threshold=65, allow_unknown=False, unknown_cost=2.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.obstacle = grid >= occupied_threshold
        if not allow_unknown:
            self.obstacle |= grid < 0

        self.dist = obstacle_distance(self.obstacle, resolution, inflation_radius)
        self.lethal = self.obstacle | (self.dist < robot_radius)

        # cost factor multiplied by the move distance when entering a cell (>= 1)
        span = max(inflation_radius - robot_radius, 1e-6)
        closeness = np.clip((inflation_radius - self.dist) / span, 0.0, 1.0)
        self.cost = 1.0 + cost_weight * closeness ** 2
        if allow_unknown:
            # unknown cells are passable but cost extra so known free cells are preferred (for exploring during SLAM)
            self.cost = self.cost + unknown_cost * (grid < 0)

        # convert to 1-D lists for search loop speed
        self._obstacle = self.obstacle.ravel().tolist()
        self._lethal = self.lethal.ravel().tolist()
        self._dist = self.dist.ravel().tolist()
        self._cost = self.cost.ravel().tolist()

        # statistics of the last plan() call (None until a search ran):
        #   algorithm, found, expanded (cells closed), pushed (heap pushes), search_ms (search loop only),
        #   plan_ms (whole plan() incl. goal snapping and smoothing), cost (path cost in m x cost factor),
        #   raw_length (m, cell path), length (m, returned path)
        self.last_stats = None
        self.last_closed = None   # bytearray of expanded cells of the last search (row-major, for visualisation)

    # -------------------------------------------------------------- coordinate conversion
    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row, col):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def inside(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, x, y):
        row, col = self.world_to_cell(x, y)
        return self.inside(row, col) and not self.lethal[row, col]

    def nearest_free(self, row, col, max_cells):
        """Nearest passable cell within max_cells of (row, col)."""
        r0, r1 = max(0, row - max_cells), min(self.height, row + max_cells + 1)
        c0, c1 = max(0, col - max_cells), min(self.width, col + max_cells + 1)
        free = ~self.lethal[r0:r1, c0:c1]
        if not free.any():
            return None
        rr, cc = np.nonzero(free)
        d2 = (rr + r0 - row) ** 2 + (cc + c0 - col) ** 2
        i = int(np.argmin(d2))
        if d2[i] > max_cells ** 2:
            return None
        return int(rr[i] + r0), int(cc[i] + c0)

    # -------------------------------------------------------------- search
    def plan(self, start, goal, goal_tolerance=0.3, smooth=True, algorithm='astar'):
        """start, goal: (x, y) [m]. Returns: [(x, y), ...] (start -> goal).

        algorithm: 'astar' (octile heuristic) | 'dijkstra' (heuristic 0). Statistics go to self.last_stats.
        """
        if algorithm not in ALGORITHMS:
            raise ValueError(f'unknown algorithm: {algorithm!r} (expected one of {ALGORITHMS})')
        t0 = time.perf_counter()
        self.last_stats = None
        self.last_closed = None
        sr, sc = self.world_to_cell(*start)
        gr, gc = self.world_to_cell(*goal)
        if not self.inside(sr, sc):
            raise PlanningError('현재 위치가 맵 밖에 있습니다.')
        if not self.inside(gr, gc):
            raise PlanningError('목표 위치가 맵 밖에 있습니다.')

        exact_goal = True
        if self.lethal[gr, gc]:
            cell = self.nearest_free(gr, gc, int(goal_tolerance / self.resolution))
            if cell is None:
                raise PlanningError('목표 위치가 장애물에 너무 가깝습니다.')
            gr, gc = cell
            exact_goal = False

        cells = self._search(sr, sc, gr, gc, algorithm)
        if cells is None:
            self.last_stats['plan_ms'] = (time.perf_counter() - t0) * 1000.0
            raise PlanningError('목표까지 갈 수 있는 경로가 없습니다.')

        points = [self.cell_to_world(r, c) for r, c in cells]
        points[0] = tuple(start)
        if exact_goal:
            points[-1] = tuple(goal)
        raw_length = path_length(points)
        if smooth:
            points = smooth_path(points)
        self.last_stats.update(plan_ms=(time.perf_counter() - t0) * 1000.0,
                               raw_length=raw_length, length=path_length(points))
        return points

    def _astar(self, sr, sc, gr, gc):
        return self._search(sr, sc, gr, gc, 'astar')

    def _search(self, sr, sc, gr, gc, algorithm='astar'):
        """Best-first search on the cell graph. 'astar': f = g + octile h, 'dijkstra': f = g (h = 0).

        Everything else (costs, neighbours, corner rule, lethal escape, tie-breaking by (f, g, index)) is shared.
        Returns the cell path [(row, col), ...] or None, and leaves statistics in self.last_stats.
        """
        t0 = time.perf_counter()
        use_h = algorithm == 'astar'
        w = self.width
        n = w * self.height
        start, goal = sr * w + sc, gr * w + gc
        obstacle, lethal, cost, dist = self._obstacle, self._lethal, self._cost, self._dist

        g = [math.inf] * n
        parent = [-1] * n
        closed = bytearray(n)
        g[start] = 0.0
        expanded = 0
        pushed = 1

        def h(idx):
            dr = abs(idx // w - gr)
            dc = abs(idx % w - gc)
            return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)

        heap = [(h(start) if use_h else 0.0, 0.0, start)]
        while heap:
            _, gc_cur, cur = heapq.heappop(heap)
            if closed[cur]:
                continue
            if cur == goal:
                break
            closed[cur] = 1
            expanded += 1
            r, c = divmod(cur, w)
            cur_lethal = lethal[cur]
            for dr, dc, step in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if nr < 0 or nc < 0 or nr >= self.height or nc >= w:
                    continue
                nxt = nr * w + nc
                if closed[nxt] or obstacle[nxt]:
                    continue
                extra = 0.0
                if lethal[nxt]:
                    # The lethal area is only for escaping when the start is already inside it: only move to cells farther
                    # from obstacles (moving freely inside it would let paths run along the lethal band through narrow gaps).
                    if not cur_lethal or dist[nxt] <= dist[cur]:
                        continue
                    extra = ESCAPE_PENALTY
                if dr and dc and (obstacle[r * w + nc] or obstacle[nr * w + c]):
                    continue  # do not cut obstacle corners diagonally
                ng = gc_cur + step * cost[nxt] + extra
                if ng < g[nxt]:
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(heap, ((ng + h(nxt)) if use_h else ng, ng, nxt))
                    pushed += 1
        # The goal is only reached by popping it (break above); if the heap ran out it was never pushed, so g stays inf
        goal_found = g[goal] < math.inf
        self.last_closed = closed
        self.last_stats = {
            'algorithm': algorithm, 'found': goal_found, 'expanded': expanded, 'pushed': pushed,
            'search_ms': (time.perf_counter() - t0) * 1000.0,
            'cost': g[goal] * self.resolution if goal_found else None,
        }
        if not goal_found:
            return None
        path = []
        idx = goal
        while idx != -1:
            path.append(divmod(idx, w))
            idx = parent[idx]
        path.reverse()
        return path


def smooth_path(points, window=2):
    """Smooth staircase shapes with a moving average, keeping both end points fixed."""
    if len(points) <= 2 * window + 1:
        return points
    arr = np.asarray(points)
    out = arr.copy()
    for i in range(window, len(arr) - window):
        out[i] = arr[i - window:i + window + 1].mean(axis=0)
    return [tuple(p) for p in out]


def path_yaws(points, goal_yaw):
    """Heading at each path point. The last point gets goal_yaw."""
    yaws = []
    for i in range(len(points) - 1):
        (x0, y0), (x1, y1) = points[i], points[i + 1]
        yaws.append(math.atan2(y1 - y0, x1 - x0))
    yaws.append(goal_yaw)
    return yaws
