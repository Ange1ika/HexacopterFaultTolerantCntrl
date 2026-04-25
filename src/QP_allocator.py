from scipy.optimize import minimize
import numpy as np
from uav_params import UAVParams

class QPAllocator:
    """
    QP control allocation:
      min  ½·uᵀWu + ρ·||Bu - d||²
      s.t. 0 ≤ u_i ≤ u_max_i

    W  — веса равномерности нагрузки (обычно I)
    rho — штраф за невыполнение wrench-команды
    """

    def __init__(self, W=None, rho=1e4):
        self.F   = UAVParams.build_input_map()
        self.k_f = UAVParams.k_f
        self.Nr  = UAVParams.Nr
        self.rho = rho
        self.W   = W if W is not None else np.eye(self.Nr)

    def allocate(self, desired: np.ndarray,
                 active: np.ndarray,
                 u_max: np.ndarray) -> np.ndarray:
        """
        desired : (4,)  [Tc, tau_x, tau_y, tau_z]
        active  : (6,)  bool маска
        u_max   : (6,)  лимиты тяг (Н)
        """
        idx = np.where(active)[0]
        B   = self.F[:, idx]                  # 4 × n_active
        W_a = self.W[np.ix_(idx, idx)]        # n × n
        n   = len(idx)
        rho = self.rho

        # Матрица квадратичной формы: Q = W + ρ·BᵀB
        Q = W_a + rho * (B.T @ B)
        # Линейный член: c = -ρ·Bᵀd
        c = -rho * (B.T @ desired)

        bounds = [(0.0, float(u_max[i])) for i in idx]

        # Тёплый старт — решение pinv (без ограничений)
        u0 = np.clip(np.linalg.pinv(B) @ desired, 0, [u_max[i] for i in idx])

        res = minimize(
            fun=lambda u: 0.5 * u @ Q @ u + c @ u,
            jac=lambda u: Q @ u + c,
            x0=u0,
            bounds=bounds,
            method='L-BFGS-B',
            options={'maxiter': 50, 'ftol': 1e-9}
        )
        u_full = np.zeros(self.Nr)
        u_full[idx] = res.x
        return u_full