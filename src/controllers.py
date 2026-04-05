import numpy as np
from scipy.linalg import solve_continuous_are
from dynamics import skew, quat_to_rotmat
from uav_params import UAVParams


class PositionController:
    """
    LQR позиционный контроль.
    Выход: f_c_inertial (3,)

    Anti-windup: интеграл ограничен по модулю MAX_INT.
    В fault-режиме интегральный канал можно заморозить снаружи
    через freeze_integral=True.
    """

    MAX_INT       = 3.0   # м·с  — порог насыщения интегратора
    MAX_FC        = 3.0 * UAVParams.m * UAVParams.g  # Н — лимит |f_c|

    def __init__(self):
        A = np.array([[0, 1, 0],
                      [0, 0, 0],
                      [1, 0, 0]])
        B = np.array([[0], [1], [0]])
        Q = np.diag([5, 0.5, 1])
        R = np.array([[1.0]])

        P  = solve_continuous_are(A, B, Q, R)
        K  = np.linalg.inv(R) @ B.T @ P
        self.k_p = K[0, 0]
        self.k_v = K[0, 1]
        self.k_i = K[0, 2]

        self.integral       = np.zeros(3)
        self.freeze_integral = False

    def compute(self, p, v_inertial, p_d, v_d, a_d, R_body, dt):
        e_pos = p - p_d
        e_vel = v_inertial - v_d

        if not self.freeze_integral:
            self.integral += e_pos * dt
            # Anti-windup: обрезаем каждую ось
            self.integral = np.clip(self.integral,
                                    -self.MAX_INT, self.MAX_INT)

        f_c = (-self.k_p * e_pos
               - self.k_v * e_vel
               - self.k_i * self.integral
               + UAVParams.m * a_d
               + UAVParams.m * np.array([0, 0, UAVParams.g]))

        # Лимит выходного усилия (защита от windup-броска)
        fc_norm = np.linalg.norm(f_c)
        if fc_norm > self.MAX_FC:
            f_c = f_c * (self.MAX_FC / fc_norm)

        return f_c

    def reset_integral(self):
        self.integral = np.zeros(3)


class AttitudePlanner:
    """attitude_planner из MATLAB, защита от сингулярности."""

    # Минимальный размер |f_c|, ниже которого берём fallback
    FC_MIN = 0.5   # Н

    @staticmethod
    def compute(psi_d: float, f_c: np.ndarray) -> np.ndarray:
        fc_norm = np.linalg.norm(f_c)

        # Если тяга почти нулевая — сохраняем вертикаль
        if fc_norm < AttitudePlanner.FC_MIN:
            f_c = np.array([0.0, 0.0, AttitudePlanner.FC_MIN])
            fc_norm = AttitudePlanner.FC_MIN

        k_p = f_c / fc_norm
        h_p = np.array([np.cos(psi_d), np.sin(psi_d), 0.0])

        cross = np.cross(k_p, h_p)
        cross_norm = np.linalg.norm(cross)

        # Сингулярность: k_p почти параллелен h_p
        # (происходит когда thrust почти горизонтален по курсу)
        if cross_norm < 1e-4:
            # Fallback: строим j_p через мировой Z или X
            alt = np.array([0., 0., 1.]) if abs(k_p[2]) < 0.9 \
                  else np.array([1., 0., 0.])
            cross = np.cross(k_p, alt)
            cross_norm = np.linalg.norm(cross)

        j_p = cross / cross_norm
        i_p = np.cross(j_p, k_p)

        return np.column_stack([i_p, j_p, k_p])   # 3×3, ортонормальна по построению


class AttitudeController:
    """
    SO(3) attitude controller.
    Входы: R_current (3×3), R_d (3×3), omega_b (3,)
    Выход: tau_b (3,)
    """

    def __init__(self):
        p   = UAVParams
        w_n = 12.0
        xi  = 0.5

        Kw = 2*xi*w_n * np.diag([p.J[0,0], p.J[1,1], p.J[2,2]])
        Kr_diag = w_n**2 * np.array([p.J[0,0], p.J[1,1], p.J[2,2]])

        A = np.array([[0,1,1],[1,0,1],[1,1,0]])
        k_vect = np.linalg.solve(A, Kr_diag)

        self.k_R     = k_vect
        self.k_omega = Kw

    def compute(self, R: np.ndarray, R_d: np.ndarray,
                omega_b: np.ndarray) -> np.ndarray:
        e_R_mat = 0.5 * (R_d.T @ R - R.T @ R_d)
        e_R = np.array([e_R_mat[2,1], e_R_mat[0,2], e_R_mat[1,0]])

        tau = (-self.k_R * e_R
               - self.k_omega @ omega_b
               + np.cross(omega_b, UAVParams.J @ omega_b))
        return tau