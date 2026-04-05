import numpy as np
from uav_params import UAVParams


class FaultTolerantAllocator:
    """
    Control allocation с fault-tolerant логикой.
    Два режима:
      mode 0 — нормальный полёт
      mode 1 — hover (автоматически при отказе)
    """

    # Времена отказов (сек) — менять здесь
    #T_FAIL = {3: 10.0}    # ключ: индекс мотора (0-based), значение: время
    
    T_FAIL = {3: 25.0}  # два отказа
    #T_FAIL = 0

    ALPHA_MARGIN = 0.85

    def __init__(self):
        self.F   = UAVParams.build_input_map()   # 4×6
        self.k_f = UAVParams.k_f
        self.Nr  = UAVParams.Nr

    def compute(self, tau: np.ndarray, Tc: float,
                lambda_r: np.ndarray, t: float):
        """
        Возвращает:
          omega_cmd (6,)  — команды скоростей роторов
          fault_flag (int)
          T_max_avail (float)
        """
        # --- 1. Определение отказов ---
        lam = lambda_r.copy().astype(float)
        for motor_idx, t_fail in self.T_FAIL.items():
            if t > t_fail:
                lam[motor_idx] = 0.0

        failed   = lam < 1e-6
        active   = ~failed
        n_active = int(np.sum(active))

        fault_flag  = int(np.any(failed))
        T_max_avail = self.Nr * self.k_f * UAVParams.Omega_max**2

        omega_cmd = np.zeros(self.Nr)

        if n_active < 4:
            return omega_cmd, fault_flag, 0.0

        # --- 2. T_max_avail ---
        T_max_avail = n_active * self.k_f * UAVParams.Omega_max**2 * self.ALPHA_MARGIN

        # --- 3. Команды в зависимости от режима ---
        mode = fault_flag   # 0 = норма, 1 = hover

        tau_cmd = tau.copy()
        Tc_cmd  = Tc

        if mode == 1:
            # Hover: рыскание отключаем, тягу ограничиваем
            two_opposite = (n_active == 4 and failed[2] and failed[5])
            tau_cmd[2] = 0.5 * tau[2] if two_opposite else 0.0
            Tc_cmd = min(Tc, T_max_avail)

        # --- 4. Аллокация (pseudo-inverse) ---
        # F shape: 4×6, строки: [Tc, tau_x, tau_y, tau_z]
        B       = self.F[:, active]                      # 4 × n_active
        desired = np.array([Tc_cmd, tau_cmd[0],
                            tau_cmd[1], tau_cmd[2]])

        u_active     = np.linalg.pinv(B) @ desired
        u_max_active = lam[active] * self.k_f * UAVParams.Omega_max**2
        u_active     = np.clip(u_active, 0, u_max_active)

        # --- 5. Тяга → скорость вращения ---
        u_full = np.zeros(self.Nr)
        u_full[active] = u_active

        for i in range(self.Nr):
            if failed[i] or u_full[i] <= 0:
                omega_cmd[i] = 0.0
            else:
                omega_cmd[i] = np.sqrt(u_full[i] / (lam[i] * self.k_f))

        return omega_cmd, fault_flag, T_max_avail


class FaultLatch:
    """
    Запоминает состояние в момент первого отказа.
    Сохраняет также скорость для плавного торможения.
    """

    def __init__(self):
        self.latched    = False
        self.p_fault    = np.zeros(3)
        self.v_fault    = np.zeros(3)   # скорость в момент отказа
        self.psi_fault  = 0.0
        self.t_fault    = 0.0

    def update(self, fault_flag: int,
               p_current: np.ndarray,
               v_current: np.ndarray,   # инерциальная скорость
               psi_current: float,
               t: float):
        if fault_flag and not self.latched:
            self.p_fault   = p_current.copy()
            self.v_fault   = v_current.copy()
            self.psi_fault = psi_current
            self.t_fault   = t
            self.latched   = True
        return self.p_fault, self.v_fault, self.psi_fault, self.t_fault