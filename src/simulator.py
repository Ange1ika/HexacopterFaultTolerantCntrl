#!/usr/bin/env python3
"""
Главный симулятор гексакоптера.
Отправляет состояние в Unity по TCP в реальном времени.

Запуск:
    python3 simulator.py

Unity подключается к 127.0.0.1:9999 и получает JSON каждые dt секунд.
"""

import socket, json, threading, time, struct
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from uav_params         import UAVParams
from dynamics           import HexacopterDynamics, quat_to_rotmat
from controllers        import PositionController, AttitudePlanner, AttitudeController
from control_allocation import FaultTolerantAllocator, FaultLatch
from trajectory         import TrajectoryPlanner
from plotter            import SimulationLogger, Plotter

# ───────────────────────────────────────────────
#  Конфигурация
# ───────────────────────────────────────────────
SIM_DT       = 0.01
SEND_DT      = 0.02
TCP_HOST     = "127.0.0.1"
TCP_PORT     = 9999
SIM_DURATION = 60.0

# Минимальный модуль f_c — AttitudePlanner не должен получать нулевой вектор
TC_MIN = UAVParams.m * UAVParams.g * 0.3   # Н


class SimState:
    """Разделяемое состояние между симуляцией и сетевым потоком"""
    def __init__(self):
        self.lock     = threading.Lock()
        self.running  = True
        self.data     = {}
        self.sim_time = 0.0
        self.logger   = None


def build_packet(dyn, fault_flag, p_d, t, omega_cmd) -> dict:
    euler = dyn.euler
    q     = dyn.q
    return {
        "t":     round(t, 3),
        "fault": int(fault_flag),
        "px": round(float(dyn.p[0]), 4),
        "py": round(float(dyn.p[1]), 4),
        "pz": round(float(dyn.p[2]), 4),
        "qw": round(float(q[0]), 6),
        "qx": round(float(q[1]), 6),
        "qy": round(float(q[2]), 6),
        "qz": round(float(q[3]), 6),
        "psi":   round(float(euler[0]), 4),
        "theta": round(float(euler[1]), 4),
        "phi":   round(float(euler[2]), 4),
        "pdx": round(float(p_d[0]), 4),
        "pdy": round(float(p_d[1]), 4),
        "pdz": round(float(p_d[2]), 4),
        "omega_r":   [round(float(w), 1) for w in dyn.omega_r],
        "omega_cmd": [round(float(w), 1) for w in omega_cmd],
    }


def simulation_loop(state: SimState):
    dyn    = HexacopterDynamics()
    pos_c  = PositionController()
    att_c  = AttitudeController()
    alloc  = FaultTolerantAllocator()
    latch  = FaultLatch()
    traj   = TrajectoryPlanner(radius=4.0, omega_t=0.1, w_t=1.0)
    logger = SimulationLogger()

    lambda_r  = np.ones(UAVParams.Nr)
    t         = 0.0
    omega_cmd = np.zeros(UAVParams.Nr)
    fault_flag = 0
    T_max      = 0.0   # 0 = нет ограничения (нормальный режим)

    p_fault   = np.zeros(3)
    v_fault   = np.zeros(3)
    psi_fault = 0.0
    t_fault   = 0.0

    print(f"[SIM] Запуск. dt={SIM_DT}s")
    print(f"[SIM] Отказ мотора в t="
          f"{list(FaultTolerantAllocator.T_FAIL.values())[0]}s")

    t_last_send = 0.0

    while state.running:
        loop_start = time.perf_counter()

        v_inertial = dyn.R @ dyn.v_b

        # ── 1. Запоминаем состояние latch ДО его обновления ───
        was_latched = latch.latched

        # ── 2. Пробный вызов аллокатора для получения fault_flag
        #       (нулевые tau/Tc — только для детекции отказа) ──
        _, fault_flag, T_max_new = alloc.compute(
            np.zeros(3), UAVParams.m * UAVParams.g, lambda_r, t)

        # ── 3. Обновляем latch (с реальной скоростью) ─────────
        p_fault, v_fault, psi_fault, t_fault = latch.update(
            fault_flag, dyn.p, v_inertial, dyn.euler[0], t)

        # ── 4. Реакция на ПЕРВЫЙ момент отказа ────────────────
        if fault_flag and not was_latched:
            T_max = T_max_new
            pos_c.reset_integral()
            pos_c.freeze_integral = True    # блокируем интегратор
            att_c.reset_integral()
            att_c.freeze_integral = True
            print(f"[SIM] t={t:.2f}s — ОТКАЗ! "
                  f"p={dyn.p.round(2)}, v={v_inertial.round(2)}")

        # ── 5. Траектория ──────────────────────────────────────
        p_d, v_d, a_d, psi_d = traj.compute(
            t, latch.latched, p_fault, v_fault, psi_fault, t_fault)

        # ── 6. Позиционный контроль ───────────────────────────
        f_c = pos_c.compute(dyn.p, v_inertial, p_d, v_d, a_d, dyn.R, SIM_DT)

        # Физическое ограничение: роторы тянут только вверх.
        # fc_z < 0 → AttitudePlanner командует перевёрнутую ориентацию → флип.
        # Зажимаем: пусть дрон просто сбрасывает тягу до минимума, не переворачиваясь.
        f_c[2] = max(f_c[2], TC_MIN)

        # Гарантируем минимальный модуль (защита от NaN в AttitudePlanner)
        fc_norm = np.linalg.norm(f_c)
        if fc_norm < TC_MIN:
            f_c = f_c * (TC_MIN / max(fc_norm, 1e-9))

        # ── 7. Attitude planner ───────────────────────────────
        q_d = AttitudePlanner.compute(psi_d, f_c) 
        Tc  = np.linalg.norm(f_c)
        if latch.latched and T_max > 0:
            Tc = min(Tc, T_max)

        # ── 8. Attitude controller ────────────────────────────
        q_d = AttitudePlanner.compute(psi_d, f_c)   # было: R_d = ...

        tau = att_c.compute(dyn.q, q_d, dyn.omega_b, SIM_DT)  # было: dyn.R, R_d

        # ── 9. Control allocation ─────────────────────────────
        omega_cmd, _, _ = alloc.compute(tau, Tc, lambda_r, t)

        # ── 10. Разморозка интегратора ────────────────────────
        # Только когда ОДНОВРЕМЕННО скорость мала И ошибка по высоте мала.
        # Иначе при большой z-ошибке интеграл сразу набегает до MAX_INT.
        if latch.latched and pos_c.freeze_integral:
            v_mag = np.linalg.norm(v_inertial)
            z_err = abs(dyn.p[2] - p_d[2])
            if v_mag < 0.3 and z_err < 0.5:
                pos_c.freeze_integral = False
                att_c.freeze_integral = False

        # ── 11. Шаг динамики ──────────────────────────────────
        dyn.step(omega_cmd, SIM_DT, lambda_r)
        logger.log(t, dyn, p_d, v_d, omega_cmd,
                   fc=f_c, Tc=Tc,
                   integral=pos_c.integral.copy(),
                   fault=fault_flag)

        # ── Inline-предупреждения ──────────────────────────────
        v_inertial_post = dyn.R @ dyn.v_b
        pitch_deg = np.rad2deg(dyn.euler[1])
        if abs(v_inertial_post[2]) > 5.0:
            print(f"[WARN t={t:.2f}] vz={v_inertial_post[2]:.2f} м/с | "
                  f"pitch={pitch_deg:.1f}° | Tc={Tc:.2f}N | "
                  f"int_z={pos_c.integral[2]:.3f} | "
                  f"fc_z={f_c[2]:.2f}N | fault={fault_flag}")
        if abs(pitch_deg) > 40.0:
            print(f"[WARN t={t:.2f}] pitch={pitch_deg:.1f}° | "
                  f"vz={v_inertial_post[2]:.2f} | Tc={Tc:.2f}N | "
                  f"int_z={pos_c.integral[2]:.3f}")

        # ── 12. Отправка в Unity ──────────────────────────────
        if t - t_last_send >= SEND_DT:
            packet = build_packet(dyn, fault_flag, p_d, t, omega_cmd)
            with state.lock:
                state.data     = packet
                state.sim_time = t
            t_last_send = t

        t += SIM_DT
        if SIM_DURATION > 0 and t > SIM_DURATION:
            print("[SIM] Симуляция завершена")
            state.running = False
            break

        elapsed = time.perf_counter() - loop_start
        sleep_t = SIM_DT - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    state.running = False
    state.logger  = logger


def tcp_server(state: SimState):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)
    server.settimeout(1.0)
    print(f"[NET] TCP сервер слушает {TCP_HOST}:{TCP_PORT}")

    while state.running:
        try:
            conn, addr = server.accept()
            print(f"[NET] Unity подключился: {addr}")
            conn.settimeout(0.1)
            handle_client(conn, state)
            print(f"[NET] Unity отключился")
        except socket.timeout:
            continue
        except Exception as e:
            if state.running:
                print(f"[NET] Ошибка: {e}")

    server.close()


def handle_client(conn: socket.socket, state: SimState):
    last_t = -1.0
    while state.running:
        with state.lock:
            if state.sim_time != last_t and state.data:
                packet = state.data.copy()
                last_t = state.sim_time
            else:
                packet = None

        if packet:
            try:
                msg  = json.dumps(packet) + "\n"
                data = msg.encode("utf-8")
                conn.sendall(struct.pack(">I", len(data)) + data)
            except (BrokenPipeError, ConnectionResetError):
                break
            except Exception as e:
                print(f"[NET] send error: {e}")
                break

        time.sleep(0.005)

    conn.close()


if __name__ == "__main__":
    state = SimState()

    sim_thread = threading.Thread(target=simulation_loop,
                                   args=(state,), daemon=True)
    sim_thread.start()

    try:
        tcp_server(state)
    except KeyboardInterrupt:
        print("\n[MAIN] Остановка...")
        state.running = False

    sim_thread.join(timeout=2.0)

    if state.logger is not None:
        plotter = Plotter(state.logger)
        plotter.plot_all()
    print("[MAIN] Готово")
