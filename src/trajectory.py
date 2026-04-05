import numpy as np


class TrajectoryPlanner:
    """
    Два режима:
      fault_flag = 0 → спираль
      fault_flag = 1 → плавное экспоненциальное торможение до hover

    При отказе желаемая траектория:
      p_d(t) = p_hover - (v_fault / k) * exp(-k * dt)
      v_d(t) = v_fault * exp(-k * dt)
      a_d(t) = -k * v_fault * exp(-k * dt)
    где p_hover = p_fault + v_fault / k — конечная точка зависания,
    k — коэффициент затухания (1/с).
    """

    # Коэффициент скорости торможения [1/с].
    # Чем больше — тем резче остановка.
    # Рекомендуется 0.8…2.0 для типичных скоростей 1-3 м/с.
    BRAKE_K = 1.2

    def __init__(self, radius=10.0, omega_t=0.1, w_t=1.0):
        self.radius  = radius
        self.omega_t = omega_t
        self.w_t     = w_t

    def compute(self, time: float,
                fault_flag: int,
                p_fault: np.ndarray,
                v_fault: np.ndarray,
                psi_fault: float,
                t_fault: float = 0.0):
        """
        Возвращает p_d (3,), v_d (3,), a_d (3,), psi_d (float).

        fault_flag — True/1 после первого отказа
        v_fault    — инерциальная скорость в момент отказа
        t_fault    — время отказа
        """
        if fault_flag:
            k  = self.BRAKE_K
            dt = max(time - t_fault, 0.0)
            decay = np.exp(-k * dt)

            # Асимптотическая точка зависания
            p_hover = p_fault + v_fault / k

            p_d = p_hover - (v_fault / k) * decay
            v_d = v_fault * decay
            a_d = -k * v_fault * decay

            return p_d, v_d, a_d, psi_fault

        # ── Нормальный режим: спираль ──────────────────────────
        # r  = self.radius
        # wt = self.omega_t
        # wz = self.w_t
        # th = wt * time

        # p_d = np.array([r * np.cos(th),
        #                 r * np.sin(th),
        #                 wz * time])
        # v_d = np.array([-r * np.sin(th) * wt,
        #                  r * np.cos(th) * wt,
        #                  wz])
        # a_d = np.array([-r * np.cos(th) * wt**2,
        #                 -r * np.sin(th) * wt**2,
        #                  0.0])
        # psi_d = th
        # параметры
        CLIMB_TIME = 3.0     # время набора высоты (сек)
        VZ = 1.0             # вертикальная скорость (м/с)
        VX = 2.0             # горизонтальная скорость (м/с)

        if not fault_flag:
            if time < CLIMB_TIME:
                # ── Фаза 1: подъём ──
                p_d = np.array([0.0,
                                0.0,
                                VZ * time])
                v_d = np.array([0.0,
                                0.0,
                                VZ])
                a_d = np.array([0.0, 0.0, 0.0])

            else:
                # ── Фаза 2: полёт вперёд ──
                dt = time - CLIMB_TIME

                p_d = np.array([VX * dt,
                                0.0,
                                VZ * CLIMB_TIME])   # держим высоту

                v_d = np.array([VX,
                                0.0,
                                0.0])

                a_d = np.array([0.0, 0.0, 0.0])

            psi_d = 0.0   # смотрим вперёд
            return p_d, v_d, a_d, psi_d

    
    