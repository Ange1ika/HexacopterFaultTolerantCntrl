import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from uav_params import UAVParams


class SimulationLogger:
    """Сбор данных во время симуляции"""

    def __init__(self):
        self.t         = []
        self.p         = []
        self.p_d       = []
        self.v         = []
        self.v_d       = []
        self.euler     = []
        self.omega     = []
        self.omega_cmd = []
        # ── диагностика ──
        self.fc        = []   # вектор силы позиционного контроллера (3,)
        self.Tc        = []   # скалярная команда тяги
        self.integral  = []   # интегральный член (3,)
        self.fault     = []   # 0/1
        self.u0 = [] 

    def log(self, t, dyn, p_d, v_d, omega_cmd,
            fc=None, Tc=None, integral=None, fault=0, u0=None):
        self.t.append(t)

        self.p.append(dyn.p.copy())
        self.p_d.append(p_d.copy())

        v_inertial = dyn.R @ dyn.v_b
        self.v.append(v_inertial.copy())
        self.v_d.append(v_d.copy())

        self.euler.append(dyn.euler.copy())
        self.omega.append(dyn.omega_r.copy())
        self.omega_cmd.append(omega_cmd.copy())

        self.fc.append(fc.copy()       if fc       is not None else np.zeros(3))
        self.Tc.append(float(Tc)       if Tc       is not None else 0.0)
        self.integral.append(integral.copy() if integral is not None else np.zeros(3))
        self.fault.append(int(fault))
        self.u0.append(u0.copy() if u0 is not None else np.zeros(3))

    def to_numpy(self):
        self.t         = np.array(self.t)
        self.p         = np.array(self.p)
        self.p_d       = np.array(self.p_d)
        self.v         = np.array(self.v)
        self.v_d       = np.array(self.v_d)
        self.euler     = np.array(self.euler)
        self.omega     = np.array(self.omega)
        self.omega_cmd = np.array(self.omega_cmd)
        self.fc        = np.array(self.fc)
        self.Tc        = np.array(self.Tc)
        self.integral  = np.array(self.integral)
        self.fault     = np.array(self.fault)
        self.u0 = np.array(self.u0)

    def print_diagnostics(self):
        """Печатает в консоль ключевые моменты нестабильности"""
        self.to_numpy() if isinstance(self.t, list) else None

        vz      = self.v[:, 2]
        pitch   = np.rad2deg(self.euler[:, 1])
        Tc      = self.Tc
        int_z   = self.integral[:, 2]

        # Пороги
        VZ_WARN    =  5.0   # м/с
        PITCH_WARN = 30.0   # °
        TC_WARN    = 20.0   # Н

        print("\n═══ ДИАГНОСТИКА ═══")

        # Первый момент когда vz > порог
        idx_vz = np.where(np.abs(vz) > VZ_WARN)[0]
        if len(idx_vz):
            i = idx_vz[0]
            print(f"[!] vz > {VZ_WARN} м/с впервые в t={self.t[i]:.2f}s: "
                  f"vz={vz[i]:.2f}, pitch={pitch[i]:.1f}°, "
                  f"Tc={Tc[i]:.2f}N, int_z={int_z[i]:.3f}")

        # Первый момент когда |pitch| > порог
        idx_p = np.where(np.abs(pitch) > PITCH_WARN)[0]
        if len(idx_p):
            i = idx_p[0]
            print(f"[!] |pitch| > {PITCH_WARN}° впервые в t={self.t[i]:.2f}s: "
                  f"pitch={pitch[i]:.1f}°, vz={vz[i]:.2f}, "
                  f"Tc={Tc[i]:.2f}N, int_z={int_z[i]:.3f}")

        # Первый момент когда Tc > порог
        idx_tc = np.where(Tc > TC_WARN)[0]
        if len(idx_tc):
            i = idx_tc[0]
            print(f"[!] Tc > {TC_WARN}N впервые в t={self.t[i]:.2f}s: "
                  f"Tc={Tc[i]:.2f}N, fc_z={self.fc[i,2]:.2f}, "
                  f"int_z={int_z[i]:.3f}, pitch={pitch[i]:.1f}°")

        # Максимальные значения
        print(f"\n  max |vz|    = {np.max(np.abs(vz)):.2f} м/с  @ t={self.t[np.argmax(np.abs(vz))]:.2f}s")
        print(f"  max |pitch| = {np.max(np.abs(pitch)):.1f}°  @ t={self.t[np.argmax(np.abs(pitch))]:.2f}s")
        print(f"  max Tc      = {np.max(Tc):.2f} N  @ t={self.t[np.argmax(Tc)]:.2f}s")
        print(f"  max |int_z| = {np.max(np.abs(int_z)):.3f} m·s  @ t={self.t[np.argmax(np.abs(int_z))]:.2f}s")
        print("═══════════════════\n")


class Plotter:
    """Построение графиков"""

    def __init__(self, log: SimulationLogger):
        self.log = log
        self.output_dir = Path("results")

    def plot_all(self):
        if isinstance(self.log.t, list):
            self.log.to_numpy()

        self.log.print_diagnostics()

        self.plot_trajectory()
        self.plot_position()
        self.plot_velocity()
        self.plot_euler()
        self.plot_rotors()
        self.plot_rotor_commands()
        self.plot_errors()
        self.plot_thrust()
        self.plot_diagnostics()   
        self.plot_mpc_control()
        self.save_all_figures()
        self.plot_mpc_u()
        
        plt.show()

    def save_all_figures(self):
        """Сохраняет все построенные графики в папку results."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        for i, fig_num in enumerate(plt.get_fignums(), start=1):
            fig = plt.figure(fig_num)
            fig.savefig(self.output_dir / f"Figure_{i}.png",
                        dpi=150, bbox_inches='tight')

    # ─────────────────────────────
    def plot_trajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot(self.log.p_d[:,0], self.log.p_d[:,1], self.log.p_d[:,2], label="Заданная")
        ax.plot(self.log.p[:,0],   self.log.p[:,1],   self.log.p[:,2],   label="БПЛА")
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.set_title("Траектория"); ax.legend(); ax.grid()

    def plot_position(self):
        plt.figure()
        for i, ax in enumerate(['x','y','z']):
            plt.plot(self.log.t, self.log.p[:,i],   label=f"{ax} БПЛА")
            plt.plot(self.log.t, self.log.p_d[:,i], '--', label=f"{ax} заданное")
        plt.xlabel("Время [с]"); plt.ylabel("Положение [м]")
        plt.title("Отслеживание положения"); plt.legend(); plt.grid()

    def plot_velocity(self):
        plt.figure()
        for i, ax in enumerate(['vx','vy','vz']):
            plt.plot(self.log.t, self.log.v[:,i],   label=f"{ax} БПЛА")
            plt.plot(self.log.t, self.log.v_d[:,i], '--', label=f"{ax} заданная")
        plt.xlabel("Время [с]"); plt.ylabel("Скорость [м/с]")
        plt.title("Отслеживание скорости"); plt.legend(); plt.grid()

    def plot_euler(self):
        plt.figure()
        plt.plot(self.log.t, np.rad2deg(self.log.euler[:,0]), label="рыскание")
        plt.plot(self.log.t, np.rad2deg(self.log.euler[:,1]), label="тангаж")
        plt.plot(self.log.t, np.rad2deg(self.log.euler[:,2]), label="крен")
        plt.xlabel("Время [с]"); plt.ylabel("Угол [град]")
        plt.title("Углы Эйлера"); plt.legend(); plt.grid()

    def plot_rotors(self):
        plt.figure()
        for i in range(self.log.omega.shape[1]):
            plt.plot(self.log.t, self.log.omega[:, i], label=f"ω{i+1}")
        plt.xlabel("Время [с]"); plt.ylabel("Скорость ротора [рад/с]")
        plt.title("Угловые скорости роторов"); plt.legend(); plt.grid()

    def plot_rotor_commands(self):
        plt.figure()
        for i in range(self.log.omega_cmd.shape[1]):
            plt.plot(self.log.t, self.log.omega_cmd[:, i], '--', label=f"команда{i+1}")
        plt.xlabel("Время [с]"); plt.ylabel("Командная ω [рад/с]")
        plt.title("Команды роторам"); plt.legend(); plt.grid()

    def plot_errors(self):
        plt.figure()
        err = self.log.p - self.log.p_d
        for i, ax in enumerate(['x','y','z']):
            plt.plot(self.log.t, err[:,i], label=f"ошибка {ax}")
        plt.xlabel("Время [с]"); plt.ylabel("Ошибка [м]")
        plt.title("Ошибка положения"); plt.legend(); plt.grid()

    def plot_thrust(self):
        from uav_params import UAVParams
        k_f = UAVParams.k_f
        thrust = k_f * self.log.omega**2
        thrust_total = np.sum(thrust, axis=1)
        plt.figure()
        plt.plot(self.log.t, thrust_total, label="Суммарная", linewidth=2)
        for i in range(thrust.shape[1]):
            plt.plot(self.log.t, thrust[:, i], '--', label=f"T{i+1}")
        plt.xlabel("Время [с]"); plt.ylabel("Тяга [Н]")
        plt.title("Тяги роторов"); plt.legend(); plt.grid()

    def plot_diagnostics(self):
        t       = self.log.t
        vz      = self.log.v[:, 2]
        pitch   = np.rad2deg(self.log.euler[:, 1])
        Tc      = self.log.Tc
        fc_z    = self.log.fc[:, 2]
        int_z   = self.log.integral[:, 2]
        fault   = self.log.fault

        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        fig.suptitle("Диагностика", fontsize=12)

        def shade_fault(ax):
            if np.any(fault):
                t_f = t[np.argmax(fault > 0)]
                ax.axvline(t_f, color='red', linestyle='--',
                           linewidth=1.2, label=f"fault t={t_f:.1f}s")

        # 1. vz
        ax = axes[0]
        ax.plot(t, vz, color='tab:blue', label="vz [м/с]")
        ax.axhline(0, color='k', linewidth=0.5)
        ax.axhline( 5, color='red',   linestyle=':', linewidth=1, label="±5 м/с")
        ax.axhline(-5, color='red',   linestyle=':', linewidth=1)
        shade_fault(ax)
        ax.set_ylabel("vz [м/с]"); ax.legend(fontsize=8); ax.grid()

        # 2. Тангаж
        ax = axes[1]
        ax.plot(t, pitch, color='tab:orange', label="pitch [°]")
        ax.axhline( 30, color='red', linestyle=':', linewidth=1, label="±30°")
        ax.axhline(-30, color='red', linestyle=':', linewidth=1)
        shade_fault(ax)
        ax.set_ylabel("Тангаж [°]"); ax.legend(fontsize=8); ax.grid()

        # 3. Tc и fc_z
        ax = axes[2]
        ax.plot(t, Tc,   color='tab:green',  label="Tc [Н]")
        ax.plot(t, fc_z, color='tab:purple', label="fc_z [Н]", linestyle='--')
        shade_fault(ax)
        ax.set_ylabel("Тяга [Н]"); ax.legend(fontsize=8); ax.grid()

        # 4. Интеграл по Z
        ax = axes[3]
        ax.plot(t, int_z, color='tab:red', label="integral_z [м·с]")
        ax.axhline( 3, color='gray', linestyle=':', linewidth=1, label="±MAX_INT")
        ax.axhline(-3, color='gray', linestyle=':', linewidth=1)
        shade_fault(ax)
        ax.set_ylabel("Интеграл z"); ax.set_xlabel("Время [с]")
        ax.legend(fontsize=8); ax.grid()

        plt.tight_layout()
            
    def plot_mpc_control(self):
        t        = self.log.t
        fc       = self.log.fc          # (T, 3) — вектор силы
        integral = self.log.integral    # (T, 3) — интеграл
        fault    = self.log.fault

        # Δfc — насколько изменилось управление за один шаг (мера пересчёта)
        delta_fc      = np.diff(fc, axis=0, prepend=fc[:1])
        delta_fc_norm = np.linalg.norm(delta_fc, axis=1)
        fc_norm       = np.linalg.norm(fc, axis=1)

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        fig.suptitle("MPC", fontsize=12)

        def shade(ax):
            idx = np.where(fault > 0)[0]
            if len(idx):
                ax.axvspan(t[idx[0]], t[-1], color='red', alpha=0.06,
                        label=f"fault t={t[idx[0]]:.1f}s")

        # ── 1. Компоненты fc ─────────────────────────────────────────────────
        ax = axes[0]
        ax.plot(t, fc[:, 0], label="fc_x", linewidth=1.2)
        ax.plot(t, fc[:, 1], label="fc_y", linewidth=1.2)
        ax.plot(t, fc[:, 2], label="fc_z", linewidth=1.8, color='tab:green')
        ax.plot(t, fc_norm,  label="|fc|",  linewidth=1.0,
                color='k', linestyle='--', alpha=0.5)
        ax.axhline(UAVParams.m * UAVParams.g, color='gray',
                linestyle=':', linewidth=1, label="m·g")
        shade(ax)
        ax.set_ylabel("Сила [Н]")
        ax.legend(fontsize=8, ncol=3)
        ax.grid()

        # ── 2. |Δfc| — скачок управления на каждом шаге ──────────────────────
        ax = axes[1]
        ax.plot(t, delta_fc_norm, color='tab:orange', linewidth=1.2,
                label="|Δfc| — изменение за шаг")
        ax.axhline(np.percentile(delta_fc_norm, 95), color='red',
                linestyle=':', linewidth=1, label="95-й перцентиль")
        shade(ax)
        ax.set_ylabel("|Δfc| [Н/шаг]")
        ax.legend(fontsize=8)
        ax.grid()

        # ── 3. Интегральный канал ─────────────────────────────────────────────
        ax = axes[2]
        ax.plot(t, integral[:, 0], label="int_x", linewidth=1.0, alpha=0.7)
        ax.plot(t, integral[:, 1], label="int_y", linewidth=1.0, alpha=0.7)
        ax.plot(t, integral[:, 2], label="int_z", linewidth=1.8, color='tab:red')
        ax.axhline( 2.0, color='gray', linestyle=':', linewidth=1, label="±MAX_INT")
        ax.axhline(-2.0, color='gray', linestyle=':', linewidth=1)
        shade(ax)
        ax.set_ylabel("Интеграл [м·с]")
        ax.set_xlabel("Время [с]")
        ax.legend(fontsize=8, ncol=2)
        ax.grid()

        plt.tight_layout()
            
    def plot_mpc_u(self):
        t   = self.log.t
        u0  = self.log.u0          # (T,3) — сырой выход QP
        fc  = self.log.fc          # (T,3) — итоговая сила
        ff_z = UAVParams.m * UAVParams.g   # приближённый feedforward по z

        delta_u = np.diff(u0, axis=0, prepend=u0[:1])

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle("MPC: изменения управления", fontsize=12)

        # ── 1. Компоненты u0 ─────────────────────────────────────────────
        ax = axes[0]
        for i, (lbl, col) in enumerate(zip(['u_x','u_y','u_z'],
                                            ['tab:blue','tab:orange','tab:green'])):
            ax.plot(t, u0[:, i], label=lbl, color=col, linewidth=1.4)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.set_ylabel("u₀ [Н]")
        ax.legend(fontsize=8, ncol=3); ax.grid()

        # ── 2. |Δu₀| — насколько сдвинулся оптимум при пересчёте ────────
        ax = axes[1]
        delta_norm = np.linalg.norm(delta_u, axis=1)
        ax.plot(t, delta_norm, color='tab:purple', linewidth=1.2,
                label="|Δu₀| между шагами")
        ax.axhline(np.percentile(delta_norm, 95), color='red',
                linestyle=':', linewidth=1, label="95-й перцентиль")
        ax.set_ylabel("|Δu₀| [Н/шаг]")
        ax.legend(fontsize=8); ax.grid()

        # ── 3. u0_z vs fc_z — что добавил feedforward ────────────────────
        ax = axes[2]
        ax.plot(t, u0[:, 2],  label="u₀_z (QP выход)",  linewidth=1.4,
                color='tab:green')
        ax.plot(t, fc[:, 2],  label="fc_z (итого)",      linewidth=1.4,
                color='k', linestyle='--', alpha=0.6)
        ax.axhline(ff_z, color='gray', linestyle=':', linewidth=1,
                label=f"ff_z ≈ m·g = {ff_z:.1f}Н")
        ax.set_ylabel("Сила по z [Н]")
        ax.set_xlabel("Время [с]")
        ax.legend(fontsize=8, ncol=2); ax.grid()

        plt.tight_layout()
