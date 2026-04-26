import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

Cell2 = Tuple[int, int]


@dataclass(frozen=True)
class BoxObstacle:
    cx: float
    cy: float
    cz: float
    sx: float
    sy: float
    sz: float

    @property
    def top(self) -> float:
        return self.cz + 0.5 * self.sz

    def sdf_xy(self, p_xy: np.ndarray) -> float:
        x, y = float(p_xy[0]), float(p_xy[1])
        xmin = self.cx - 0.5 * self.sx
        xmax = self.cx + 0.5 * self.sx
        ymin = self.cy - 0.5 * self.sy
        ymax = self.cy + 0.5 * self.sy

        dx = max(xmin - x, 0.0, x - xmax)
        dy = max(ymin - y, 0.0, y - ymax)
        outside = np.hypot(dx, dy)

        inside = xmin <= x <= xmax and ymin <= y <= ymax
        if inside:
            return -min(x - xmin, xmax - x, y - ymin, ymax - y)
        return float(outside)


@dataclass(frozen=True)
class CylinderObstacle:
    cx: float
    cy: float
    zmin: float
    zmax: float
    radius: float

    @property
    def top(self) -> float:
        return self.zmax

    def sdf_xy(self, p_xy: np.ndarray) -> float:
        return float(np.hypot(float(p_xy[0]) - self.cx, float(p_xy[1]) - self.cy) - self.radius)


class TrajectoryPlanner:
    """2D A* (XY) + separate altitude profile (climb/cruise/descent)."""

    def __init__(
        self,
        start_xyz: Optional[np.ndarray] = None,
        goal_xyz: Optional[np.ndarray] = None,
        arena_size_xy: float = 40.0,
        arena_height: float = 16.0,
        cell_size: float = 0.8,
        nominal_speed: float = 1.2,
        fault_speed_scale: float = 0.55,
        max_accel_nominal: float = 0.45,
        max_accel_fault: float = 0.18,
        clearance: float = 0.8,
        sdf_weight: float = 1.8,
        obstacle_seed: int = 123,
        allow_diagonal: bool = True,
        start_hold_time: float = 1.2,
    ):
        self.arena_size_xy = arena_size_xy
        self.arena_height = arena_height
        self.cell_size = cell_size
        self.nominal_speed = nominal_speed
        self.fault_speed_scale = fault_speed_scale
        self.max_accel_nominal = max_accel_nominal
        self.max_accel_fault = max_accel_fault
        self.clearance = clearance
        self.path_clearance = clearance + 0.9
        self.sdf_weight = sdf_weight
        self.allow_diagonal = allow_diagonal
        self.start_hold_time = start_hold_time

<<<<<<< HEAD
    # Коэффициент скорости торможения [1/с].
    # Чем больше — тем резче остановка.
    # Рекомендуется 0.8…2.0 для типичных скоростей 1-3 м/с.
    BRAKE_K = 1.2
    FAILURE_CONTROL = False

    def __init__(self, radius=10.0, omega_t=0.4, w_t=1.0):
        self.radius  = radius
        self.omega_t = omega_t
        self.w_t     = w_t
=======
        self.half_xy = 0.5 * arena_size_xy
        self.max_x = int(round(arena_size_xy / cell_size))
        self.max_y = int(round(arena_size_xy / cell_size))

        self.start_xyz = np.array(start_xyz if start_xyz is not None else [-18.0, -18.0, 2.0], dtype=float)
        self.goal_xyz = np.array(goal_xyz if goal_xyz is not None else [16.0, 16.0, 0.5], dtype=float)
>>>>>>> my-branch

        self.boxes: List[BoxObstacle] = []
        self.cylinders: List[CylinderObstacle] = []
        self.path_xy: List[np.ndarray] = []

<<<<<<< HEAD
        fault_flag — True/1 после первого отказа
        v_fault    — инерциальная скорость в момент отказа
        t_fault    — время отказа
        """
        if fault_flag and not self.FAILURE_CONTROL:
            k  = self.BRAKE_K
            dt = max(time - t_fault, 0.0)
            decay = np.exp(-k * dt)
=======
        found = False
        for attempt in range(60):
            self._build_fixed_obstacles(obstacle_seed + attempt)
            if self._is_blocked_xy(self.start_xyz[:2]) or self._is_blocked_xy(self.goal_xyz[:2]):
                continue
>>>>>>> my-branch

            raw = self._plan_astar_xy(self.start_xyz[:2], self.goal_xyz[:2])
            if len(raw) < 2:
                continue

<<<<<<< HEAD
            p_d = p_hover - (v_fault / k) * decay
            v_d = v_fault * decay
            a_d = -k * v_fault * decay
            
            if np.mean(v_d) > 0.05:
                self.FAILURE_CONTROL = True
            return p_d, v_d, a_d, psi_fault
        
        elif self.FAILURE_CONTROL:
            # Если скорость уже почти нулевая, продолжать спираль, чтобы не стоять на месте     
            r  = self.radius
            wt = self.omega_t
            wz = self.w_t
            th = wt * time
            p_d = np.array([r * np.cos(th),
                            r * np.sin(th),
                            wz * time])
            v_d = np.array([-r * np.sin(th) * wt,
                            r * np.cos(th) * wt,
                            wz])
            a_d = np.array([-r * np.cos(th) * wt**2,
                            -r * np.sin(th) * wt**2,
                            0.0])
            psi_d = th
=======
            smooth = self._line_of_sight_simplify_xy(raw)
            sampled = self._resample_polyline_xy(smooth, step=0.6)
            if self._path_collision_free_xy(sampled):
                self.path_xy = sampled
                found = True
                break
>>>>>>> my-branch

        if not found:
            self.path_xy = [self.start_xyz[:2].copy(), self.start_xyz[:2].copy()]

<<<<<<< HEAD
        # CLIMB_TIME = 3.0     # время набора высоты (сек)
        # VZ = 1.0             # вертикальная скорость (м/с)
        # VX = 2.0             # горизонтальная скорость (м/с)

        if not fault_flag:
            # if time < CLIMB_TIME:
            #     # ── Фаза 1: подъём ──
            #     p_d = np.array([0.0,
            #                     0.0,
            #                     VZ * time])
            #     v_d = np.array([0.0,
            #                     0.0,
            #                     VZ])
            #     a_d = np.array([0.0, 0.0, 0.0])

            # else:
            #     # ── Фаза 2: полёт вперёд ──
            #     dt = time - CLIMB_TIME

            #     p_d = np.array([VX * dt,
            #                     0.0,
            #                     VZ * CLIMB_TIME])   # держим высоту

            #     v_d = np.array([VX,
            #                     0.0,
            #                     0.0])

            #     a_d = np.array([0.0, 0.0, 0.0])

            # psi_d = 0.0   # смотрим вперёд
            # return p_d, v_d, a_d, psi_d
            
            
            # ── Нормальный режим: спираль ──────────────────────────
            r  = self.radius
            wt = self.omega_t
            wz = self.w_t
            th = wt * time

            p_d = np.array([r * np.cos(th),
                            r * np.sin(th),
                            wz * time])
            v_d = np.array([-r * np.sin(th) * wt,
                            r * np.cos(th) * wt,
                            wz])
            a_d = np.array([-r * np.cos(th) * wt**2,
                            -r * np.sin(th) * wt**2,
                            0.0])
            psi_d = th
            return p_d, v_d, a_d, psi_d
=======
        self._path_s = self._build_arc_lengths_xy(self.path_xy)
        self._total_len = float(self._path_s[-1]) if len(self._path_s) else 0.0
        self._progress_s = 0.0
        self._speed_cmd = 0.0
        self._last_time = None
        self.landed = False

        tops = [o.top for o in self.boxes] + [o.top for o in self.cylinders]
        mean_top = float(np.mean(tops)) if tops else 2.0
        max_top = float(np.max(tops)) if tops else 2.0
        cruise_h = 0.5 * (mean_top + max_top) + 0.7
        self.cruise_z = float(np.clip(cruise_h, 1.5, min(arena_height - 1.0, 8.0)))

        self.z_cmd = float(self.start_xyz[2])
        self.vz_cmd = 0.0
        self.z_vmax = 0.8
        self.z_amax = 0.7

    def compute(self, time, fault_flag, p_fault, v_fault, psi_fault, t_fault=0.0):
        if self._total_len <= 1e-6:
            self.landed = True
            return self.goal_xyz.copy(), np.zeros(3), np.zeros(3), 0.0

        if time < self.start_hold_time:
            self._last_time = float(time)
            self._progress_s = 0.0
            self._speed_cmd = 0.0
            self.z_cmd = float(self.start_xyz[2])
            self.vz_cmd = 0.0
            self.landed = False
            return self.start_xyz.copy(), np.zeros(3), np.zeros(3), 0.0

        dt = 0.0 if self._last_time is None else max(float(time - self._last_time), 0.0)
        self._last_time = float(time)

        speed_target = self.nominal_speed * (self.fault_speed_scale if fault_flag else 1.0)
        accel_lim = self.max_accel_fault if fault_flag else self.max_accel_nominal
        remain = max(self._total_len - self._progress_s, 0.0)
        speed_target = min(speed_target, np.sqrt(max(2.0 * accel_lim * remain, 0.0)))

        if dt > 0.0:
            dv = speed_target - self._speed_cmd
            self._speed_cmd += float(np.clip(dv, -accel_lim * dt, accel_lim * dt))

        self._progress_s = min(self._progress_s + self._speed_cmd * dt, self._total_len)
        pos_xy, tangent_xy = self._sample_path_xy(self._progress_s)

        s_ratio = (self._progress_s / self._total_len) if self._total_len > 1e-9 else 1.0
        if s_ratio < 0.2:
            z_target = self.cruise_z
        elif s_ratio < 0.8:
            z_target = self.cruise_z
        else:
            z_target = float(self.goal_xyz[2])

        if dt > 0.0:
            vz_target = float(np.clip(1.2 * (z_target - self.z_cmd), -self.z_vmax, self.z_vmax))
            dvz = np.clip(vz_target - self.vz_cmd, -self.z_amax * dt, self.z_amax * dt)
            self.vz_cmd += float(dvz)
            self.z_cmd += self.vz_cmd * dt

        self.z_cmd = float(np.clip(self.z_cmd, 0.2, self.arena_height - 0.3))

        done_xy = self._progress_s >= (self._total_len - 1e-3)
        done_z = abs(self.z_cmd - self.goal_xyz[2]) < 0.05 and abs(self.vz_cmd) < 0.05
        if done_xy and done_z and self._speed_cmd < 0.04:
            self.landed = True
            return self.goal_xyz.copy(), np.zeros(3), np.zeros(3), 0.0

        self.landed = False
        v_xy = tangent_xy * self._speed_cmd
        p_d = np.array([pos_xy[0], pos_xy[1], self.z_cmd], dtype=float)
        v_d = np.array([v_xy[0], v_xy[1], self.vz_cmd], dtype=float)
        a_d = np.zeros(3)
        psi_d = float(np.arctan2(v_xy[1], v_xy[0])) if np.linalg.norm(v_xy) > 1e-6 else 0.0
        return p_d, v_d, a_d, psi_d

    def obstacle_payload(self):
        types: List[int] = []
        params: List[float] = []
        for b in self.boxes:
            types.append(0)
            params += [b.cx, b.cy, b.cz, b.sx, b.sy, b.sz, 0.0]
        for c in self.cylinders:
            types.append(1)
            h = c.zmax - c.zmin
            cz = 0.5 * (c.zmin + c.zmax)
            params += [c.cx, c.cy, cz, c.radius, h, 0.0, 0.0]
        return types, params

    def obstacles_2d(self):
        out = []
        for b in self.boxes:
            out.append({"type": "box", "cx": b.cx, "cy": b.cy, "sx": b.sx, "sy": b.sy})
        for c in self.cylinders:
            out.append({"type": "cyl", "cx": c.cx, "cy": c.cy, "r": c.radius})
        return out

    def _world_sdf_xy(self, p_xy: np.ndarray) -> float:
        d_world = min(
            p_xy[0] + self.half_xy,
            self.half_xy - p_xy[0],
            p_xy[1] + self.half_xy,
            self.half_xy - p_xy[1],
        )
        d_obs = float("inf")
        for b in self.boxes:
            d_obs = min(d_obs, b.sdf_xy(p_xy))
        for c in self.cylinders:
            d_obs = min(d_obs, c.sdf_xy(p_xy))
        return float(min(d_world, d_obs))

    def _is_blocked_xy(self, p_xy: np.ndarray) -> bool:
        return self._world_sdf_xy(p_xy) <= self.path_clearance

    def _build_fixed_obstacles(self, seed: int):
        rng = np.random.default_rng(seed)
        self.boxes = []
        self.cylinders = []

        xs = np.linspace(-13.5, 13.5, 4)
        ys = np.linspace(-11.5, 11.5, 3)
        anchors = [(x, y) for y in ys for x in xs]
        rng.shuffle(anchors)

        def far_key(cx, cy, r):
            s = np.hypot(cx - self.start_xyz[0], cy - self.start_xyz[1])
            g = np.hypot(cx - self.goal_xyz[0], cy - self.goal_xyz[1])
            return s > (4.5 + r) and g > (4.5 + r)

        def overlaps(cx, cy, r):
            for b in self.boxes:
                if np.hypot(cx - b.cx, cy - b.cy) < (r + 0.55 * max(b.sx, b.sy) + 1.2):
                    return True
            for c in self.cylinders:
                if np.hypot(cx - c.cx, cy - c.cy) < (r + c.radius + 1.2):
                    return True
            return False

        # 4 cubes
        for i in range(4):
            cx, cy = anchors[i]
            s = float(rng.uniform(4.8, 6.4))
            h = float(rng.uniform(8.0, 12.5))
            if far_key(cx, cy, 0.55 * s) and not overlaps(cx, cy, 0.55 * s):
                self.boxes.append(BoxObstacle(cx, cy, 0.5 * h, s, s, h))

        # 2 rectangular boxes
        for i in range(4, 6):
            cx, cy = anchors[i]
            sx = float(rng.uniform(6.8, 9.2))
            sy = float(rng.uniform(3.8, 5.3))
            if rng.random() < 0.5:
                sx, sy = sy, sx
            h = float(rng.uniform(9.0, 13.8))
            r = 0.55 * max(sx, sy)
            if far_key(cx, cy, r) and not overlaps(cx, cy, r):
                self.boxes.append(BoxObstacle(cx, cy, 0.5 * h, sx, sy, h))

        # 6 cylinders
        for i in range(6, 12):
            cx, cy = anchors[i]
            r = float(rng.uniform(2.0, 3.3))
            h = float(rng.uniform(8.5, 14.5))
            if far_key(cx, cy, r) and not overlaps(cx, cy, r):
                self.cylinders.append(CylinderObstacle(cx, cy, 0.0, h, r))

        self.boxes = self.boxes[:6]
        self.cylinders = self.cylinders[:6]

    def _world_to_cell_xy(self, p_xy: np.ndarray) -> Cell2:
        return (
            int(round((np.clip(p_xy[0], -self.half_xy, self.half_xy) + self.half_xy) / self.cell_size)),
            int(round((np.clip(p_xy[1], -self.half_xy, self.half_xy) + self.half_xy) / self.cell_size)),
        )

    def _cell_to_world_xy(self, c: Cell2) -> np.ndarray:
        return np.array([
            -self.half_xy + c[0] * self.cell_size,
            -self.half_xy + c[1] * self.cell_size,
        ])

    def _inside_cell_xy(self, c: Cell2) -> bool:
        return 0 <= c[0] <= self.max_x and 0 <= c[1] <= self.max_y

    def _cell_blocked_xy(self, c: Cell2) -> bool:
        return (not self._inside_cell_xy(c)) or self._is_blocked_xy(self._cell_to_world_xy(c))

    def _neighbors_xy(self):
        n = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            n += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        return n

    @staticmethod
    def _heuristic_xy(a: Cell2, b: Cell2) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _step_cost_xy(self, cur: Cell2, nb: Cell2) -> float:
        base = float(np.hypot(nb[0] - cur[0], nb[1] - cur[1]))
        sdf = max(self._world_sdf_xy(self._cell_to_world_xy(nb)) - self.path_clearance, 0.05)
        return base * (1.0 + self.sdf_weight / sdf)

    def _plan_astar_xy(self, start_xy: np.ndarray, goal_xy: np.ndarray) -> List[np.ndarray]:
        s = self._world_to_cell_xy(start_xy)
        g = self._world_to_cell_xy(goal_xy)
        if self._cell_blocked_xy(s) or self._cell_blocked_xy(g):
            return []

        open_heap = [(self._heuristic_xy(s, g), s)]
        came: Dict[Cell2, Cell2] = {}
        g_score: Dict[Cell2, float] = {s: 0.0}
        closed = set()

        for_pop = self._neighbors_xy()
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == g:
                cells = [cur]
                while cur in came:
                    cur = came[cur]
                    cells.append(cur)
                cells.reverse()
                return [self._cell_to_world_xy(c) for c in cells]
            closed.add(cur)

            for dx, dy in for_pop:
                nb = (cur[0] + dx, cur[1] + dy)
                if (not self._inside_cell_xy(nb)) or (nb in closed) or self._cell_blocked_xy(nb):
                    continue

                if abs(dx) + abs(dy) > 1:
                    if self._cell_blocked_xy((cur[0] + dx, cur[1])):
                        continue
                    if self._cell_blocked_xy((cur[0], cur[1] + dy)):
                        continue

                ng = g_score[cur] + self._step_cost_xy(cur, nb)
                if nb not in g_score or ng < g_score[nb]:
                    g_score[nb] = ng
                    came[nb] = cur
                    heapq.heappush(open_heap, (ng + self._heuristic_xy(nb, g), nb))
        return []

    def _segment_collision_free_xy(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        L = float(np.linalg.norm(p1 - p0))
        n = max(int(np.ceil(L / (0.28 * self.cell_size))), 4)
        for i in range(n + 1):
            p = p0 + (i / n) * (p1 - p0)
            if self._is_blocked_xy(p):
                return False
        return True

    def _line_of_sight_simplify_xy(self, pts: List[np.ndarray]) -> List[np.ndarray]:
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if self._segment_collision_free_xy(pts[i], pts[j]):
                    break
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _path_collision_free_xy(self, pts: List[np.ndarray]) -> bool:
        for i in range(len(pts) - 1):
            if not self._segment_collision_free_xy(pts[i], pts[i + 1]):
                return False
        return True

    @staticmethod
    def _build_arc_lengths_xy(pts: List[np.ndarray]) -> np.ndarray:
        if not pts:
            return np.array([0.0])
        s = [0.0]
        for i in range(1, len(pts)):
            s.append(s[-1] + float(np.linalg.norm(pts[i] - pts[i - 1])))
        return np.array(s)

    def _resample_polyline_xy(self, pts: List[np.ndarray], step: float) -> List[np.ndarray]:
        if len(pts) < 2:
            return pts
        s = self._build_arc_lengths_xy(pts)
        total = float(s[-1])
        out = []
        q = 0.0
        ds = max(step, 0.25)
        while q < total:
            out.append(self._sample_polyline_xy(pts, s, q))
            q += ds
        out.append(pts[-1].copy())
        return out

    @staticmethod
    def _sample_polyline_xy(pts, s, q):
        idx = int(np.searchsorted(s, q, side="right") - 1)
        idx = int(np.clip(idx, 0, len(pts) - 2))
        p0, p1 = pts[idx], pts[idx + 1]
        s0, s1 = s[idx], s[idx + 1]
        a = (q - s0) / max(s1 - s0, 1e-9)
        return p0 + a * (p1 - p0)

    def _sample_path_xy(self, q):
        q = float(np.clip(q, 0.0, self._total_len))
        idx = int(np.searchsorted(self._path_s, q, side="right") - 1)
        idx = int(np.clip(idx, 0, len(self.path_xy) - 2))
        p0, p1 = self.path_xy[idx], self.path_xy[idx + 1]
        s0, s1 = self._path_s[idx], self._path_s[idx + 1]
        a = (q - s0) / max(s1 - s0, 1e-9)
        pos = p0 + a * (p1 - p0)
        tng = p1 - p0
        n = np.linalg.norm(tng)
        tng = tng / n if n > 1e-9 else np.array([1.0, 0.0])
        return pos, tng
>>>>>>> my-branch

