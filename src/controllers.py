import numpy as np
from scipy.linalg import solve_continuous_are
from dynamics import skew, quat_to_rotmat
from uav_params import UAVParams
from quat_mat import rotmat_to_quat, quat_mult, quat_conj

class PositionController:
    """
    LQR позиционный контроль.
    Выход: f_c_inertial (3,)

    Anti-windup: интеграл ограничен по модулю MAX_INT.
    В fault-режиме интегральный канал можно заморозить снаружи
    через freeze_integral=True.
    """

    MAX_INT       = 2.0   # м·с  — порог насыщения интегратора
    MAX_FC        = 2.0 * UAVParams.m * UAVParams.g  # Н — лимит |f_c|

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
    FC_MIN       = 0.5
    MAX_TILT_DEG = 25.0

    @staticmethod
    def compute(psi_d: float, f_c: np.ndarray) -> np.ndarray:
        """
        Возвращает q_d: np.ndarray [w, x, y, z] — желаемый кватернион.
        """
        fc_norm = np.linalg.norm(f_c)
        if fc_norm < AttitudePlanner.FC_MIN:
            f_c     = np.array([0.0, 0.0, AttitudePlanner.FC_MIN])
            fc_norm = AttitudePlanner.FC_MIN

        k_p = f_c / fc_norm

        if k_p[2] > 1e-6:
            tilt_max  = np.deg2rad(AttitudePlanner.MAX_TILT_DEG)
            tan_tilt  = np.tan(tilt_max)
            k_xy      = np.linalg.norm(k_p[:2])
            k_xy_max  = tan_tilt * k_p[2]
            if k_xy > k_xy_max and k_xy > 1e-9:
                k_p[:2] *= k_xy_max / k_xy
                k_p      = k_p / np.linalg.norm(k_p)

        h_p        = np.array([np.cos(psi_d), np.sin(psi_d), 0.0])
        cross      = np.cross(k_p, h_p)
        cross_norm = np.linalg.norm(cross)

        if cross_norm < 1e-4:
            alt        = np.array([0.,0.,1.]) if abs(k_p[2]) < 0.9 \
                         else np.array([1.,0.,0.])
            cross      = np.cross(k_p, alt)
            cross_norm = np.linalg.norm(cross)

        j_p = cross / cross_norm
        i_p = np.cross(j_p, k_p)

        R_d = np.column_stack([i_p, j_p, k_p])
        return rotmat_to_quat(R_d)   # ← единственное новое
    
class AttitudeController:
    """
    Кватернионный attitude controller.
    Входы: q (4,), q_d (4,), omega_b (3,)
    Выход: tau_b (3,)

    Закон управления:
        e_q  = vect(q_d* ⊗ q)  · sign(scal(q_d* ⊗ q))
        tau  = −k_q ⊙ e_q − K_ω @ ω + ω × Jω
    """

    def __init__(self):
        p   = UAVParams
        w_n = 12.0
        xi  = 0.5

        # ── Пересчёт коэффициентов ──────────────────────────────────────────
        # В SO(3) ошибка: e_R ≈ θ·n̂  (для малых углов)
        # В кватернионах: ε_e ≈ (θ/2)·n̂
        # Чтобы сохранить ту же полосу пропускания (w_n), нужно удвоить k_p:
        #   J·θ̈ = −k_q·(θ/2) − k_ω·θ̇  →  ω_n² = k_q/(2J)  →  k_q = 2·J·ω_n²
        J_diag   = np.array([p.J[0,0], p.J[1,1], p.J[2,2]])
        self.k_q     = 2.0 * w_n**2 * J_diag        # пропорциональный
        self.k_omega = 2.0 * xi * w_n * np.diag(J_diag)  # демпфирующий

        # Интеграл только по yaw
        self.k_i_yaw       = 0.15 * self.k_q[2]
        self.yaw_integral  = 0.0
        self.freeze_integral = False
        self.max_yaw_int   = np.deg2rad(25.0)

    def compute(self, q: np.ndarray, q_d: np.ndarray,
                omega_b: np.ndarray, dt: float) -> np.ndarray:

        # ── Кватернион ошибки (аналог R_d.T @ R в SO(3)) ───────────────────
        q_e = quat_mult(quat_conj(q_d), q)

        # Двойное покрытие: q и −q — одна ориентация.
        # Берём представление с w > 0 → кратчайший путь вращения.
        if q_e[0] < 0:
            q_e = -q_e

        e_q = q_e[1:]   # векторная часть: [ex, ey, ez]

        # ── Интеграл по yaw с anti-windup ───────────────────────────────────
        if not self.freeze_integral:
            self.yaw_integral += e_q[2] * dt
            self.yaw_integral  = np.clip(self.yaw_integral,
                                         -self.max_yaw_int,
                                         self.max_yaw_int)

        # Медленное стравливание у нуля
        if abs(e_q[2]) < np.deg2rad(2.0) and abs(omega_b[2]) < np.deg2rad(5.0):
            self.yaw_integral *= 0.98

        # ── Момент управления ────────────────────────────────────────────────
        tau = (-self.k_q * e_q
               - self.k_omega @ omega_b
               - np.array([0.0, 0.0, self.k_i_yaw * self.yaw_integral])
               + np.cross(omega_b,  UAVParams.J @ omega_b))   # гироскопическая компенсация
        return tau

    def reset_integral(self):
        self.yaw_integral = 0.0
