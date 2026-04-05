import numpy as np
from uav_params import UAVParams

def skew(v):
    return np.array([[ 0,    -v[2],  v[1]],
                     [ v[2],  0,    -v[0]],
                     [-v[1],  v[0],  0   ]])

def quat_mult(p, q):
    """Перемножение кватернионов [w, x, y, z]"""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])

def quat_to_rotmat(q):
    """Кватернион [w,x,y,z] → матрица вращения 3×3"""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [w*w+x*x-y*y-z*z,  2*(x*y-w*z),      2*(x*z+w*y)     ],
        [2*(x*y+w*z),       w*w+y*y-x*x-z*z,  2*(y*z-w*x)     ],
        [2*(x*z-w*y),       2*(y*z+w*x),       w*w+z*z-x*x-y*y],
    ])

def quat_to_euler(q):
    """Кватернион → [psi, theta, phi] (ZYX) в радианах"""
    C = quat_to_rotmat(q)
    psi   = np.arctan2(C[1,0], C[1,1])
    theta = np.arctan2(-C[2,0], C[2,2])
    phi   = np.arctan2(C[2,1], np.sqrt(C[2,0]**2 + C[2,2]**2))
    return np.array([psi, theta, phi])


class HexacopterDynamics:
    """
    Состояние: p (3), q (4), v_b (3), omega_b (3), omega_r (6)
    Интегрирование: RK4
    """

    def __init__(self):
        p     = UAVParams
        self.F     = p.build_input_map()          # 4×6
        self.Minv  = np.linalg.inv(p.total_mass_matrix())  # 6×6
        self.D     = p.D
        self.k_f   = p.k_f
        self.k_m   = p.k_m
        self.g_vec = np.array([0, 0, -p.g])

        # Начальное состояние
        self.p       = np.zeros(3)
        self.q       = np.array([1., 0., 0., 0.])  # [w,x,y,z]
        self.v_b     = np.zeros(3)
        self.omega_b = np.zeros(3)
        self.omega_r = np.zeros(6)

    def step(self, omega_cmd: np.ndarray, dt: float, lambda_r: np.ndarray):
        """Один шаг RK4"""
        state = self._pack()
        k1 = self._deriv(state, omega_cmd, lambda_r)
        k2 = self._deriv(state + 0.5*dt*k1, omega_cmd, lambda_r)
        k3 = self._deriv(state + 0.5*dt*k2, omega_cmd, lambda_r)
        k4 = self._deriv(state + dt*k3,      omega_cmd, lambda_r)
        state_new = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        self._unpack(state_new)
        # Нормализация кватерниона
        self.q /= np.linalg.norm(self.q)

    def _deriv(self, state, omega_cmd, lambda_r):
        p, q, v_b, omega_b, omega_r = self._unpack_arr(state)
        R = quat_to_rotmat(q)

        # Динамика моторов (первый порядок)
        d_omega_r = UAVParams.k_m * (omega_cmd - omega_r)

        # Тяги моторов
        u = lambda_r * UAVParams.k_f * omega_r**2  # (6,)

        # Обобщённые силы через матрицу входов F (4×6)
        # wrench = [Tc, tau_x, tau_y, tau_z]
        wrench = self.F @ u          # (4,)
        Tc   = wrench[0]
        tau  = wrench[1:]            # (3,)

        # Гравитация в body frame
        g_b = R.T @ np.array([0, 0, -UAVParams.g * UAVParams.m])

        # Линейное ускорение (body)
        f_body = np.array([0, 0, Tc]) + g_b
        # Угловое ускорение
        gyro_term = -np.cross(omega_b, UAVParams.J @ omega_b)
        alpha_b = UAVParams.Jinv @ (tau + gyro_term
                                    - self.D[3:, 3:] @ omega_b)

        # Кватернионная кинематика
        eta = q[0]; eps = q[1:]
        W = np.vstack([-eps, skew(eps) + eta*np.eye(3)])  # 4×3
        q_dot = 0.5 * W @ omega_b

        # Позиция (инерциальная)
        p_dot = R @ v_b

        # Скорость body
        v_b_dot = (f_body / UAVParams.m
                   - np.cross(omega_b, v_b)
                   - self.D[:3, :3] @ v_b / UAVParams.m)

        return np.concatenate([p_dot, q_dot, v_b_dot, alpha_b, d_omega_r])

    def _pack(self):
        return np.concatenate([self.p, self.q, self.v_b,
                                self.omega_b, self.omega_r])

    def _unpack(self, s):
        self.p, self.q, self.v_b, self.omega_b, self.omega_r = \
            self._unpack_arr(s)
        return self

    @staticmethod
    def _unpack_arr(s):
        return s[0:3], s[3:7], s[7:10], s[10:13], s[13:19]

    @property
    def euler(self):
        return quat_to_euler(self.q)

    @property
    def R(self):
        return quat_to_rotmat(self.q)
