import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

@staticmethod
def load_csv(filename):
    
    df = pd.read_csv(filename)

    log = SimulationLogger()

    log.t   = df["t"].values

    log.p   = df[["px","py","pz"]].values
    log.p_d = df[["px_d","py_d","pz_d"]].values

    log.v   = df[["vx","vy","vz"]].values
    log.v_d = df[["vx_d","vy_d","vz_d"]].values

    log.euler = df[["psi","theta","phi"]].values

    # ω динамически
    omega_cols = [c for c in df.columns if "omega_" in c and "cmd" not in c]
    omega_cmd_cols = [c for c in df.columns if "omega_cmd" in c]

    log.omega     = df[omega_cols].values
    log.omega_cmd = df[omega_cmd_cols].values

    log.fc        = df[["fc_x","fc_y","fc_z"]].values
    log.Tc        = df["Tc"].values
    log.integral  = df[["int_x","int_y","int_z"]].values
    log.fault     = df["fault"].values

    print(f"[LOG] Данные загружены из {filename}")

    return log
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
        # Метаданные от планировщика (опционально)
        self.start_xyz  = None
        self.goal_xyz   = None
        self.obstacles_2d = None
        self.obstacle_types = None
        self.obstacle_params = None

    def log(self, t, dyn, p_d, v_d, omega_cmd,
            fc=None, Tc=None, integral=None, fault=0):
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
    
    @staticmethod
    def load_csv(filename):


        df = pd.read_csv(filename)

        log = SimulationLogger()

        log.t   = df["t"].values

        log.p   = df[["px","py","pz"]].values
        log.p_d = df[["px_d","py_d","pz_d"]].values

        log.v   = df[["vx","vy","vz"]].values
        log.v_d = df[["vx_d","vy_d","vz_d"]].values

        log.euler = df[["psi","theta","phi"]].values

        # ω динамически
        omega_cols = [c for c in df.columns if "omega_" in c and "cmd" not in c]
        omega_cmd_cols = [c for c in df.columns if "omega_cmd" in c]

        log.omega     = df[omega_cols].values
        log.omega_cmd = df[omega_cmd_cols].values

        log.fc        = df[["fc_x","fc_y","fc_z"]].values
        log.Tc        = df["Tc"].values
        log.integral  = df[["int_x","int_y","int_z"]].values
        log.fault     = df["fault"].values

        print(f"[LOG] Данные загружены из {filename}")

        return log

    def save_csv(self, filename="simulation_log.csv"):
        if isinstance(self.t, list):
            self.to_numpy()

        n_rotors = self.omega.shape[1]

        header = [
            "t",
            "px","py","pz",
            "px_d","py_d","pz_d",
            "vx","vy","vz",
            "vx_d","vy_d","vz_d",
            "psi","theta","phi",
        ]

        # ω
        for i in range(n_rotors):
            header.append(f"omega_{i+1}")

        for i in range(n_rotors):
            header.append(f"omega_cmd_{i+1}")

        # диагностика
        header += [
            "fc_x","fc_y","fc_z",
            "Tc",
            "int_x","int_y","int_z",
            "fault"
        ]

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)

            for i in range(len(self.t)):
                row = [
                    self.t[i],
                    *self.p[i],
                    *self.p_d[i],
                    *self.v[i],
                    *self.v_d[i],
                    *self.euler[i],
                ]

                row += list(self.omega[i])
                row += list(self.omega_cmd[i])

                row += [
                    *self.fc[i],
                    self.Tc[i],
                    *self.integral[i],
                    self.fault[i]
                ]

                writer.writerow(row)

        print(f"[LOG] Данные сохранены в {filename}")
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


import numpy as np
import matplotlib.pyplot as plt


class Plotter:
    def __init__(self, log):
        self.log = log

    def _get_fault_time(self):
        fault = self.log.fault
        if np.any(fault):
            return self.log.t[np.argmax(fault > 0)]
        return None

    def _add_fault_line(self, ax):
        t_f = self._get_fault_time()
        if t_f is not None:
            ax.axvline(t_f, color='red', linestyle='--',
                       linewidth=1.5, label=r"$t_f$ — момент отказа")

    # ─────────────────────────────
    def plot_all(self):
        if isinstance(self.log.t, list):
            self.log.to_numpy()

        self.plot_obstacles_2d()
        self.plot_trajectory()
        self.plot_position()
        self.plot_velocity()
        self.plot_euler()
        self.plot_rotors()
        self.plot_rotor_commands()
        self.plot_errors()
        self.plot_thrust()
        self.plot_diagnostics()

        plt.show()

    # ─────────────────────────────
    def plot_obstacles_2d(self):
        obs = getattr(self.log, "obstacles_2d", None)
        if not obs:
            return

        fig, ax = plt.subplots()
        for o in obs:
            if o.get("type") == "box":
                cx, cy = o["cx"], o["cy"]
                sx, sy = o["sx"], o["sy"]
                rect = plt.Rectangle((cx - 0.5 * sx, cy - 0.5 * sy), sx, sy,
                                     fill=True, alpha=0.35, edgecolor='k')
                ax.add_patch(rect)
            elif o.get("type") == "cyl":
                cx, cy, r = o["cx"], o["cy"], o["r"]
                circle = plt.Circle((cx, cy), r, fill=True, alpha=0.35, edgecolor='k')
                ax.add_patch(circle)

        p0 = getattr(self.log, "start_xyz", None)
        pg = getattr(self.log, "goal_xyz", None)
        if p0 is not None:
            ax.scatter([p0[0]], [p0[1]], c='green', s=70, label='start')
        if pg is not None:
            ax.scatter([pg[0]], [pg[1]], c='red', s=70, label='goal')

        if hasattr(self.log, "p_d") and len(self.log.p_d) > 0:
            ax.plot(self.log.p_d[:, 0], self.log.p_d[:, 1], 'b--', linewidth=1.5, label='A* ref (XY)')
        if hasattr(self.log, "p") and len(self.log.p) > 0:
            ax.plot(self.log.p[:, 0], self.log.p[:, 1], 'k', linewidth=1.5, label='flight (XY)')

        ax.set_aspect('equal', 'box')
        ax.set_xlabel('X, м')
        ax.set_ylabel('Y, м')
        ax.set_title('2D карта препятствий и траектория')
        ax.grid(True)
        ax.legend()

    # ─────────────────────────────
    def plot_trajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        self._draw_obstacles_3d(ax)

        ax.plot(self.log.p_d[:,0], self.log.p_d[:,1], self.log.p_d[:,2],
                label=r"$\mathbf{p}_d$")
        ax.plot(self.log.p[:,0], self.log.p[:,1], self.log.p[:,2],
                label=r"$\mathbf{p}$")

        ax.set_xlabel("X, м")
        ax.set_ylabel("Y, м")
        ax.set_zlabel("Z, м")
        ax.set_title("Пространственная траектория ЛА")
        ax.view_init(elev=28, azim=-52)
        ax.legend()
        ax.grid()

    def _draw_obstacles_3d(self, ax):
        types = getattr(self.log, "obstacle_types", None)
        prm = getattr(self.log, "obstacle_params", None)
        if types is None or prm is None:
            return

        n = min(len(types), len(prm) // 7)
        for i in range(n):
            k = i * 7
            t = int(types[i])
            cx, cy, cz = float(prm[k+0]), float(prm[k+1]), float(prm[k+2])
            a, b, c = float(prm[k+3]), float(prm[k+4]), float(prm[k+5])

            if t == 1:
                self._plot_cylinder(ax, cx, cy, cz, radius=a, height=b,
                                    color=(0.55, 0.55, 0.72, 0.26))
            else:
                self._plot_box(ax, cx, cy, cz, sx=a, sy=b, sz=c,
                               color=(0.50, 0.50, 0.68, 0.24))

    @staticmethod
    def _plot_box(ax, cx, cy, cz, sx, sy, sz, color):
        x0, x1 = cx - 0.5*sx, cx + 0.5*sx
        y0, y1 = cy - 0.5*sy, cy + 0.5*sy
        z0, z1 = cz - 0.5*sz, cz + 0.5*sz

        v = np.array([
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        ])
        faces = [
            [v[0], v[1], v[2], v[3]],
            [v[4], v[5], v[6], v[7]],
            [v[0], v[1], v[5], v[4]],
            [v[1], v[2], v[6], v[5]],
            [v[2], v[3], v[7], v[6]],
            [v[3], v[0], v[4], v[7]],
        ]
        poly = Poly3DCollection(faces, facecolors=[color], edgecolors=(0.2,0.2,0.3,0.35), linewidths=0.6)
        ax.add_collection3d(poly)

    @staticmethod
    def _plot_cylinder(ax, cx, cy, cz, radius, height, color):
        z0, z1 = cz - 0.5*height, cz + 0.5*height
        th = np.linspace(0, 2*np.pi, 24)
        z = np.array([z0, z1])
        thg, zg = np.meshgrid(th, z)
        xg = cx + radius * np.cos(thg)
        yg = cy + radius * np.sin(thg)
        ax.plot_surface(xg, yg, zg, color=color, linewidth=0, antialiased=True, shade=True)

    # ─────────────────────────────
    def plot_position(self):
        plt.figure()

        labels = ['x','y','z']
        for i, ax_name in enumerate(labels):
            plt.plot(self.log.t, self.log.p[:,i],
                     label=rf"$p_{ax_name}$")
            plt.plot(self.log.t, self.log.p_d[:,i], '--',
                     label=rf"$p^d_{ax_name}$")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel("Положение, м")
        plt.title(r"Отслеживание положения: $\mathbf{p}$")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_velocity(self):
        plt.figure()

        labels = ['x','y','z']
        for i, ax_name in enumerate(labels):
            plt.plot(self.log.t, self.log.v[:,i],
                     label=rf"$v_{ax_name}$")
            plt.plot(self.log.t, self.log.v_d[:,i], '--',
                     label=rf"$v^d_{ax_name}$")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel("Скорость, м/с")
        plt.title(r"Отслеживание скорости: $\mathbf{v}$")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_euler(self):
        plt.figure()

        plt.plot(self.log.t, np.rad2deg(self.log.euler[:,0]),
                 label=r"$\psi$ — курс")
        plt.plot(self.log.t, np.rad2deg(self.log.euler[:,1]),
                 label=r"$\theta$ — тангаж")
        plt.plot(self.log.t, np.rad2deg(self.log.euler[:,2]),
                 label=r"$\phi$ — крен")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel("Углы, град")
        plt.title("Ориентация: η = [ψ, θ, φ]")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_rotors(self):
        plt.figure()

        for i in range(self.log.omega.shape[1]):
            plt.plot(self.log.t, self.log.omega[:, i],
                     label=rf"$\omega_{i+1}$")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel(r"$\omega_i$, рад/с")
        plt.title("Фактические угловые скорости роторов")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_rotor_commands(self):
        plt.figure()

        for i in range(self.log.omega_cmd.shape[1]):
            plt.plot(self.log.t, self.log.omega_cmd[:, i], '--',
                     label=rf"$\omega_{{{i+1}}}^{{cmd}}$")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel(r"$\omega_i^{cmd}$, рад/с")
        plt.title("Командные скорости роторов")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_errors(self):
        plt.figure()

        e = self.log.p - self.log.p_d

        labels = ['x','y','z']
        for i, ax_name in enumerate(labels):
            plt.plot(self.log.t, e[:,i],
                     label=rf"$e_{ax_name}$")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel("Ошибка, м")
        plt.title(r"Ошибка позиционирования: $\mathbf{e}_p$")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_thrust(self):
        from uav_params import UAVParams

        k_f = UAVParams.k_f
        thrust = k_f * self.log.omega**2
        thrust_total = np.sum(thrust, axis=1)

        plt.figure()

        plt.plot(self.log.t, thrust_total,
                 linewidth=2,
                 label=r"$T$")

        for i in range(thrust.shape[1]):
            plt.plot(self.log.t, thrust[:, i], '--',
                     label=rf"$T_{i+1}$")

        self._add_fault_line(plt.gca())

        plt.xlabel("t, с")
        plt.ylabel("Тяга, Н")
        plt.title("Тяги роторов")
        plt.legend()
        plt.grid()

    # ─────────────────────────────
    def plot_diagnostics(self):
        t       = self.log.t
        vz      = self.log.v[:, 2]
        pitch   = np.rad2deg(self.log.euler[:, 1])
        Tc      = self.log.Tc
        fc_z    = self.log.fc[:, 2]
        int_z   = self.log.integral[:, 2]

        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        fig.suptitle("Диагностика системы управления")

        # vz
        ax = axes[0]
        ax.plot(t, vz, label=r"$v_z$")
        ax.axhline(5, color='red', linestyle=':')
        ax.axhline(-5, color='red', linestyle=':')
        self._add_fault_line(ax)
        ax.set_ylabel("v_z, м/с")
        ax.legend()
        ax.grid()

        # pitch
        ax = axes[1]
        ax.plot(t, pitch, label=r"$\theta$")
        ax.axhline(30, color='red', linestyle=':')
        ax.axhline(-30, color='red', linestyle=':')
        self._add_fault_line(ax)
        ax.set_ylabel("θ, град")
        ax.legend()
        ax.grid()

        # тяга
        ax = axes[2]
        ax.plot(t, Tc, label=r"$T_c$ — команда тяги")
        ax.plot(t, fc_z, '--', label=r"$f_{c,z}$")
        self._add_fault_line(ax)
        ax.set_ylabel("Сила, Н")
        ax.legend()
        ax.grid()

        # интегратор
        ax = axes[3]
        ax.plot(t, int_z, label=r"$I_z$")
        ax.axhline(3, color='gray', linestyle=':')
        ax.axhline(-3, color='gray', linestyle=':')
        self._add_fault_line(ax)
        ax.set_ylabel("I_z")
        ax.set_xlabel("t, с")
        ax.legend()
        ax.grid()

        plt.tight_layout()
