import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

Cell2 = Tuple[int, int]


@dataclass(frozen=True)
class BoxObstacle:
    cx: float; cy: float; cz: float
    sx: float; sy: float; sz: float

    @property
    def top(self) -> float:
        return self.cz + 0.5 * self.sz


@dataclass(frozen=True)
class CylinderObstacle:
    cx: float; cy: float; zmin: float; zmax: float; radius: float

    @property
    def top(self) -> float:
        return self.zmax


class TrajectoryPlanner:
    """2D A* (XY) + altitude profile: climb → cruise → descent by distance to goal."""

    def __init__(
        self,
        start_xyz: Optional[np.ndarray] = None,
        goal_xyz:  Optional[np.ndarray] = None,
        arena_size_xy:    float = 45.0,
        arena_height:     float = 16.0,
        cell_size:        float = 0.8,
        nominal_speed:    float = 0.9,
        fault_speed_scale: float = 0.35,
        max_accel_nominal: float = 0.25,
        max_accel_fault:   float = 0.08,
        clearance:        float = 0.4,
        obstacle_seed:    int   = 123,
        start_hold_time:  float = 1.2,
    ):
        self.arena_size_xy    = arena_size_xy
        self.arena_height     = arena_height
        self.cell_size        = cell_size
        self.nominal_speed    = nominal_speed
        self.fault_speed_scale = fault_speed_scale
        self.max_accel_nominal = max_accel_nominal
        self.max_accel_fault   = max_accel_fault
        self.clearance        = clearance
        self.path_clearance   = clearance
        self.start_hold_time  = start_hold_time

        self.half_xy = 0.5 * arena_size_xy
        self.max_x   = int(round(arena_size_xy / cell_size))
        self.max_y   = int(round(arena_size_xy / cell_size))

        self.start_xyz = np.array(start_xyz if start_xyz is not None else [-18., -18., 2.], dtype=float)
        self.goal_xyz  = np.array(goal_xyz  if goal_xyz  is not None else [ 20.,  10., 0.5], dtype=float)

        self.boxes:     List[BoxObstacle]      = []
        self.cylinders: List[CylinderObstacle] = []
        self.path_xy:   List[np.ndarray]       = []
        self._last_v_d = np.zeros(3)
        self._last_a_d = np.zeros(3)

        for attempt in range(60):
            self._build_obstacles(obstacle_seed + attempt)
            if self._blocked(self.start_xyz[:2]) or self._blocked(self.goal_xyz[:2]):
                continue
            raw = self._astar(self.start_xyz[:2], self.goal_xyz[:2])
            if len(raw) < 2:
                continue
            path = self._smooth(self._resample(self._simplify(raw), step=0.6))
            if self._path_free(path):
                self.path_xy = path
                break
        else:
            self.path_xy = [self.start_xyz[:2].copy(), self.goal_xyz[:2].copy()]

        self._path_s     = self._arc_lengths(self.path_xy)
        self._total_len  = float(self._path_s[-1])
        self._progress_s = 0.0
        self._speed_cmd  = 0.0
        self._last_time  = None
        self.landed      = False

        tops = [o.top for o in self.boxes] + [o.top for o in self.cylinders]
        self.cruise_z = float(np.clip(
            max(tops) + 1.0 if tops else 3.0,
            2.0, min(arena_height - 1.0, 8.0)
        ))
        self.z_cmd  = float(self.start_xyz[2])
        self.vz_cmd = 0.0
        self.z_vmax = 0.8
        self.z_amax = 0.5

    # ── Obstacles ──────────────────────────────────────────────────────────

    def _build_obstacles(self, seed: int):
        rng = np.random.default_rng(seed)
        self.boxes, self.cylinders = [], []

        # Uniform 4×4 grid with small jitter
        xs = np.linspace(-14, 14, 4)
        ys = np.linspace(-14, 14, 4)
        grid = [(float(x), float(y)) for x in xs for y in ys]
        order = rng.permutation(len(grid))

        def ok(cx, cy, r):
            if np.hypot(cx - self.start_xyz[0], cy - self.start_xyz[1]) < r + 1.0:
                return False
            if np.hypot(cx - self.goal_xyz[0],  cy - self.goal_xyz[1])  < r + 1.0:
                return False
            for b in self.boxes:
                if np.hypot(cx - b.cx, cy - b.cy) < r + 0.5*max(b.sx, b.sy) + 1.5:
                    return False
            for c in self.cylinders:
                if np.hypot(cx - c.cx, cy - c.cy) < r + c.radius + 1.5:
                    return False
            return True

        for i, idx in enumerate(order):
            gx, gy = grid[idx]
            cx = gx + float(rng.uniform(-1.5, 1.5))
            cy = gy + float(rng.uniform(-1.5, 1.5))
            h  = float(rng.uniform(9.0, 15.0))

            if i % 2 == 0:          # box
                s = float(rng.uniform(4.0, 7.0))
                if ok(cx, cy, 0.5*s):
                    self.boxes.append(BoxObstacle(cx, cy, 0.5*h, s, s, h))
            else:                   # cylinder
                r = float(rng.uniform(1.8, 3.0))
                if ok(cx, cy, r):
                    self.cylinders.append(CylinderObstacle(cx, cy, 0.0, h, r))

            if len(self.boxes) >= 9 and len(self.cylinders) >= 6:
                break

    # ── Collision check (rectangular for boxes, circular for cylinders) ────

    def _blocked(self, p: np.ndarray) -> bool:
        x, y = float(p[0]), float(p[1])
        if not (-self.half_xy < x < self.half_xy and -self.half_xy < y < self.half_xy):
            return True
        c = self.path_clearance
        for b in self.boxes:
            if abs(x - b.cx) < 0.5*b.sx + c and abs(y - b.cy) < 0.5*b.sy + c:
                return True
        for cyl in self.cylinders:
            if np.hypot(x - cyl.cx, y - cyl.cy) < cyl.radius + c:
                return True
        return False

    # ── A* ────────────────────────────────────────────────────────────────

    def _w2c(self, p: np.ndarray) -> Cell2:
        return (
            int(round((np.clip(p[0], -self.half_xy, self.half_xy) + self.half_xy) / self.cell_size)),
            int(round((np.clip(p[1], -self.half_xy, self.half_xy) + self.half_xy) / self.cell_size)),
        )

    def _c2w(self, c: Cell2) -> np.ndarray:
        return np.array([-self.half_xy + c[0]*self.cell_size,
                         -self.half_xy + c[1]*self.cell_size])

    def _cell_ok(self, c: Cell2) -> bool:
        return (0 <= c[0] <= self.max_x and 0 <= c[1] <= self.max_y
                and not self._blocked(self._c2w(c)))

    def _astar(self, start: np.ndarray, goal: np.ndarray) -> List[np.ndarray]:
        s, g = self._w2c(start), self._w2c(goal)
        if not self._cell_ok(s) or not self._cell_ok(g):
            return []

        DIRS = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        h = lambda a, b: float(np.hypot(a[0]-b[0], a[1]-b[1]))

        heap = [(h(s,g), s)]
        came: Dict[Cell2, Cell2] = {}
        gsc:  Dict[Cell2, float] = {s: 0.0}
        closed: set = set()

        while heap:
            _, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            if cur == g:
                path = []
                while cur in came:
                    path.append(cur); cur = came[cur]
                path.append(s)
                return [self._c2w(c) for c in reversed(path)]
            closed.add(cur)
            for dx, dy in DIRS:
                nb = (cur[0]+dx, cur[1]+dy)
                if not self._cell_ok(nb) or nb in closed:
                    continue
                if abs(dx)+abs(dy) > 1:  # diagonal: check both straights
                    if not self._cell_ok((cur[0]+dx, cur[1])) or \
                       not self._cell_ok((cur[0], cur[1]+dy)):
                        continue
                ng = gsc[cur] + np.hypot(dx, dy)
                if nb not in gsc or ng < gsc[nb]:
                    gsc[nb] = ng; came[nb] = cur
                    heapq.heappush(heap, (ng + h(nb, g), nb))
        return []

    # ── Path processing ───────────────────────────────────────────────────

    def _seg_free(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        n = max(int(np.ceil(np.linalg.norm(p1-p0) / (0.3*self.cell_size))), 4)
        return all(not self._blocked(p0 + i/n*(p1-p0)) for i in range(n+1))

    def _simplify(self, pts: List[np.ndarray]) -> List[np.ndarray]:
        if len(pts) <= 2:
            return pts
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i+1 and not self._seg_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j]); i = j
        return out

    @staticmethod
    def _arc_lengths(pts: List[np.ndarray]) -> np.ndarray:
        s = [0.0]
        for i in range(1, len(pts)):
            s.append(s[-1] + float(np.linalg.norm(pts[i]-pts[i-1])))
        return np.array(s)

    @staticmethod
    def _interp(pts: List[np.ndarray], s: np.ndarray, q: float) -> np.ndarray:
        idx = int(np.clip(np.searchsorted(s, q, side="right")-1, 0, len(pts)-2))
        a = (q - s[idx]) / max(s[idx+1]-s[idx], 1e-9)
        return pts[idx] + a*(pts[idx+1]-pts[idx])

    def _resample(self, pts: List[np.ndarray], step: float) -> List[np.ndarray]:
        s = self._arc_lengths(pts)
        out = [self._interp(pts, s, q) for q in np.arange(0, float(s[-1]), max(step, 0.25))]
        out.append(pts[-1].copy())
        return out

    def _smooth(self, pts: List[np.ndarray], w: int = 9, passes: int = 2) -> List[np.ndarray]:
        arr = np.array(pts, dtype=float)
        if len(arr) < w:
            return pts
        k = np.ones(w) / w
        for _ in range(passes):
            pad = np.pad(arr, ((w//2, w//2), (0, 0)), mode="edge")
            for d in range(2):
                arr[:, d] = np.convolve(pad[:, d], k, mode="valid")
            arr[0] = np.array(pts[0]); arr[-1] = np.array(pts[-1])
        return [p.copy() for p in arr]

    def _path_free(self, pts: List[np.ndarray]) -> bool:
        return all(self._seg_free(pts[i], pts[i+1]) for i in range(len(pts)-1))

    # ── Path sampling ─────────────────────────────────────────────────────

    def _sample(self, q: float) -> Tuple[np.ndarray, np.ndarray]:
        q   = float(np.clip(q, 0.0, self._total_len))
        idx = int(np.clip(np.searchsorted(self._path_s, q, side="right")-1,
                          0, len(self.path_xy)-2))
        p0, p1 = self.path_xy[idx], self.path_xy[idx+1]
        a   = (q - self._path_s[idx]) / max(self._path_s[idx+1]-self._path_s[idx], 1e-9)
        pos = p0 + a*(p1-p0)
        tng = p1-p0; n = np.linalg.norm(tng)
        return pos, (tng/n if n > 1e-9 else np.array([1., 0.]))

    def _tangent(self, q: float) -> np.ndarray:
        p1, _ = self._sample(max(q-0.5, 0.0))
        p2, _ = self._sample(min(q+0.5, self._total_len))
        d = p2-p1; n = np.linalg.norm(d)
        if n < 1e-6:
            _, t = self._sample(q); return t
        return d/n

    # ── Main loop ─────────────────────────────────────────────────────────

    def compute(self, time, fault_flag, p_fault, v_fault, psi_fault, t_fault=0.0):
        if self._total_len <= 1e-6:
            self.landed = True
            return self.goal_xyz.copy(), np.zeros(3), np.zeros(3), 0.0

        if time < self.start_hold_time:
            self._last_time = float(time)
            self._last_v_d  = np.zeros(3)
            return self.start_xyz.copy(), np.zeros(3), np.zeros(3), 0.0

        dt = 0.0 if self._last_time is None else max(float(time-self._last_time), 0.0)
        self._last_time = float(time)

        # XY speed: braking near end + curvature slowdown
        v_max  = self.nominal_speed * (self.fault_speed_scale if fault_flag else 1.0)
        a_lim  = self.max_accel_fault if fault_flag else self.max_accel_nominal
        remain = max(self._total_len - self._progress_s, 0.0)
        v_target = min(v_max, float(np.sqrt(max(a_lim * remain, 0.0))))

        if self._progress_s < self._total_len - 1.0:
            t1 = self._tangent(self._progress_s)
            t2 = self._tangent(self._progress_s + 1.0)
            angle = float(np.arccos(np.clip(np.dot(t1, t2), -1., 1.)))
            v_target *= max(0.4, 1.0 - angle)

        if dt > 0.0:
            self._speed_cmd += float(np.clip(v_target-self._speed_cmd, -a_lim*dt, a_lim*dt))
        self._progress_s = min(self._progress_s + self._speed_cmd*dt, self._total_len)

        pos_xy, _ = self._sample(self._progress_s)
        tangent    = self._tangent(self._progress_s)

        # Z: descent triggered by physical distance to XY goal, not s_ratio
        DESCENT_R    = 5.0
        dist_goal_xy = float(np.linalg.norm(pos_xy - self.goal_xyz[:2]))
        if dist_goal_xy >= DESCENT_R:
            z_target = self.cruise_z
        else:
            alpha    = 1.0 - dist_goal_xy / DESCENT_R
            z_target = self.cruise_z + alpha * (float(self.goal_xyz[2]) - self.cruise_z)

        if dt > 0.0:
            z_rem     = abs(self.z_cmd - z_target)
            vz_brake  = float(np.sqrt(max(2.0*self.z_amax*z_rem, 0.0)))
            vz_target = float(np.clip(1.5*(z_target-self.z_cmd),
                                      -min(self.z_vmax, vz_brake),
                                       min(self.z_vmax, vz_brake)))
            self.vz_cmd += float(np.clip(vz_target-self.vz_cmd,
                                         -self.z_amax*dt, self.z_amax*dt))
            self.z_cmd  += self.vz_cmd * dt

        if self.vz_cmd < 0.0 and self.z_cmd < z_target:   # no overshoot below target
            self.z_cmd = z_target; self.vz_cmd = 0.0
        self.z_cmd = float(np.clip(self.z_cmd, 0.2, self.arena_height-0.3))

        done_xy = self._progress_s >= self._total_len - 1e-3
        done_z  = abs(self.z_cmd - float(self.goal_xyz[2])) < 0.05 and abs(self.vz_cmd) < 0.05
        if done_xy and done_z and self._speed_cmd < 0.04:
            self.landed = True
            self._last_v_d = np.zeros(3)
            return self.goal_xyz.copy(), np.zeros(3), np.zeros(3), 0.0

        self.landed = False
        v_xy = tangent * self._speed_cmd
        p_d  = np.array([pos_xy[0], pos_xy[1], self.z_cmd])
        v_d  = np.array([v_xy[0],   v_xy[1],   self.vz_cmd])

        if dt > 0.0:
            a_raw = np.clip((v_d - self._last_v_d)/dt, -0.3, 0.3)
            a_d   = 0.15*a_raw + 0.85*self._last_a_d   # first-order filter
        else:
            a_d   = self._last_a_d.copy()
        self._last_v_d = v_d.copy()
        self._last_a_d = a_d.copy()

        psi_d = float(np.arctan2(v_xy[1], v_xy[0])) if np.linalg.norm(v_xy) > 1e-6 else 0.0
        return p_d, v_d, a_d, psi_d

    # ── Unity / visualisation helpers ─────────────────────────────────────

    def obstacle_payload(self):
        types, params = [], []
        for b in self.boxes:
            types.append(0)
            params += [b.cx, b.cy, b.cz, b.sx, b.sy, b.sz, 0.0]
        for c in self.cylinders:
            types.append(1)
            h = c.zmax - c.zmin
            params += [c.cx, c.cy, 0.5*(c.zmin+c.zmax), c.radius, h, 0.0, 0.0]
        return types, params

    def obstacles_2d(self):
        out = []
        for b in self.boxes:
            out.append({"type": "box", "cx": b.cx, "cy": b.cy, "sx": b.sx, "sy": b.sy})
        for c in self.cylinders:
            out.append({"type": "cyl", "cx": c.cx, "cy": c.cy, "r": c.radius})
        return out