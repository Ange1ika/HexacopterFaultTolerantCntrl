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


import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.optimize import minimize
from uav_params import UAVParams

class MPCPositionController:
    """
    MPC позиционный контроль (drop-in замена PositionController).

    Модель ошибки (ZOH, dt_mpc):
        e_p[k+1] = e_p[k] + dt·e_v[k]
        e_v[k+1] = e_v[k] + dt·u[k]/m
        x = [e_p(3), e_v(3)]  — 6D,  u = deviation от feedforward (3D)

    QP (condensed form, предвычислен):
        min_U  ½ U^T H U + (F^T x0)^T U
        s.t.   −u_max ≤ U_k ≤ u_max   ∀k

    Feedforward и интеграл добавляются поверх оптимального u0:
        f_c = u0 + m·(a_d + g_e) − k_i·integral

    Warm-start: U предыдущего шага сдвигается на 1 (receding horizon).
    Solver: L-BFGS-B (scipy). Для продакшна — заменить на osqp/quadprog.
    """

    # ── Гиперпараметры ────────────────────────────────────────────────────
    N       = 10                              # горизонт предсказания
    DT_MPC  = 0.2                            # шаг предсказания, с
    MAX_INT = 2.0                             # anti-windup, м·с
    MAX_FC  = 2.0 * UAVParams.m * UAVParams.g # лимит |f_c|, Н
    K_I     = 1.0                             # усиление интегратора

    # Веса: [e_px, e_py, e_pz,  e_vx, e_vy, e_vz]
    Q_DIAG  = np.array([5.0, 5.0, 5.0,  0.5, 0.5, 0.5])
    R_DIAG  = np.array([1.0, 1.0, 1.0])

    def __init__(self):
        m   = UAVParams.m
        dt  = self.DT_MPC
        I3  = np.eye(3)
        Z3  = np.zeros((3, 3))

        # ZOH дискретизация двойного интегратора
        self.A = np.block([[I3,      dt * I3],   # (6,6)
                           [Z3,           I3]])
        self.B = np.block([[Z3            ],      # (6,3)
                           [dt / m * I3   ]])

        Q  = np.diag(self.Q_DIAG)
        R  = np.diag(self.R_DIAG)

        # Терминальная стоимость из DARE (гарантирует стабильность)
        try:
            Qf = solve_discrete_are(self.A, self.B, Q, R)
        except Exception:
            Qf = Q * 20          # fallback

        # Предвычисляем H и F один раз
        self.H_qp, self.F_qp = self._build_condensed_qp(Q, R, Qf)

        # Состояние интегратора
        self.integral        = np.zeros(3)
        self.freeze_integral = False

        # Warm-start буфер: U ∈ R^{N*3}
        self._U_prev = np.zeros(self.N * 3)
        self.u0_last  = np.zeros(3) 

    # ── Предвычисление QP-матриц ─────────────────────────────────────────
    def _build_condensed_qp(self, Q, R, Qf):
        N   = self.N
        A, B = self.A, self.B
        n, nu = A.shape[0], B.shape[1]   # 6, 3

        # Phi (N·n, n) — свободное движение
        Phi = np.zeros((N * n, n))
        Apow = A.copy()
        for i in range(N):
            Phi[i*n:(i+1)*n] = Apow
            Apow = A @ Apow

        # Gamma (N·n, N·nu) — отклик на управление
        Gamma = np.zeros((N * n, N * nu))
        for j in range(N):
            AB = B.copy()
            for i in range(j, N):
                Gamma[i*n:(i+1)*n, j*nu:(j+1)*nu] = AB
                AB = A @ AB

        # Блочно-диагональные веса с терминальным Qf
        Q_bar = np.kron(np.eye(N), Q)
        Q_bar[(N-1)*n:, (N-1)*n:] = Qf
        R_bar = np.kron(np.eye(N), R)

        H = Gamma.T @ Q_bar @ Gamma + R_bar
        H = (H + H.T) / 2          # численная симметрия

        # F: x0^T F Gamma — градиент по x0
        # grad_U = H·U + F^T·x0
        F = Phi.T @ Q_bar @ Gamma  # (n, N·nu)

        # Сохраняем Phi для warm-start сдвига
        self._Phi = Phi
        return H, F

    # ── Основной метод (совместим с PositionController.compute) ──────────
    def compute(self, p, v_inertial, p_d, v_d, a_d, R_body, dt):
        m = UAVParams.m
        g = UAVParams.g

        e_p = p          - p_d
        e_v = v_inertial - v_d

        # Интегратор с anti-windup
        if not self.freeze_integral:
            self.integral += e_p * dt
            self.integral  = np.clip(self.integral, -self.MAX_INT, self.MAX_INT)

        x0 = np.concatenate([e_p, e_v])   # (6,)

        # Feedforward + интегральная добавка
        ff    = m * (a_d + np.array([0.0, 0.0, g]))  # (3,)
        u_int = -self.K_I * self.integral              # (3,)

        # ── QP: min ½U^T H U + g_vec^T U ──────────────────────────────
        g_vec  = self.F_qp.T @ x0          # (N*3,)
        u_max  = self.MAX_FC               # консервативные симметричные bounds

        # Warm-start: сдвигаем на 1 шаг (рецедирующий горизонт)
        U_warm = np.roll(self._U_prev, -3)
        U_warm[-3:] = self._U_prev[-3:]    # дублируем последний шаг

        result = minimize(
            fun=lambda U: (0.5 * U @ self.H_qp @ U + g_vec @ U,
                           self.H_qp @ U + g_vec),
            x0=U_warm,
            method='L-BFGS-B',
            jac=True,
            bounds=[(-u_max, u_max)] * (self.N * 3),
            options={'maxiter': 50, 'ftol': 1e-8, 'gtol': 1e-6},
        )

        self._U_prev = result.x
        u0 = result.x[:3]                  # первый шаг (receding horizon)
        self.u0_last = u0.copy() 

        # Финальный выход
        f_c = u0 + ff + u_int

        fc_norm = np.linalg.norm(f_c)
        if fc_norm > self.MAX_FC:
            f_c *= self.MAX_FC / fc_norm

        return f_c

    def reset_integral(self):
        self.integral  = np.zeros(3)
        self._U_prev   = np.zeros(self.N * 3)
        
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
