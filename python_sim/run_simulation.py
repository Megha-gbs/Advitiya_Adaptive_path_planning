"""
SIH Problem Statement 26037: Vehicle Dynamics & Control (Meghana)
Tuned Benchmark Harness: Pure Pursuit vs Stanley vs Optimal MPC.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

class VehicleParams:
    def __init__(self):
        self.m = 1250.0        # Mass (kg)
        self.Iz = 1850.0       # Yaw moment of inertia (kg*m^2)
        self.L = 2.60          # Wheelbase (m)
        self.Lf = 1.15         # CG to front axle (m)
        self.Lr = 1.45         # CG to rear axle (m)
        self.Cf = 65000.0      # Front cornering stiffness (N/rad)
        self.Cr = 72000.0      # Rear cornering stiffness (N/rad)
        self.mu = 0.85         # Nominal friction
        self.delta_max = np.deg2rad(32)
        self.delta_rate_max = np.deg2rad(38)
        self.a_max = 3.5
        self.a_min = -6.5
        self.a_emergency = -8.5
        self.g = 9.81
        self.Fz_f_static = (self.m * self.g * self.Lr) / self.L
        self.Fz_r_static = (self.m * self.g * self.Lf) / self.L
        self.air_density = 1.225
        self.Cd = 0.34
        self.frontal_area = 2.2
        self.rolling_res = 0.015

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def dynamic_bicycle_step(state, u, vp, dt, mu=0.85):
    X, Y, psi, vx, vy, r = state
    vx = max(0.5, vx)
    delta, ax_cmd = u

    Fz_f = max(500.0, vp.Fz_f_static - (vp.m * ax_cmd * 0.5) / vp.L)
    Fz_r = max(500.0, vp.Fz_r_static + (vp.m * ax_cmd * 0.5) / vp.L)

    alpha_f = delta - np.arctan2(vy + vp.Lf * r, vx)
    alpha_r = -np.arctan2(vy - vp.Lr * r, vx)

    Fyf_max = mu * Fz_f
    Fyr_max = mu * Fz_r

    Fyf = Fyf_max * np.tanh((vp.Cf * alpha_f) / (Fyf_max + 1e-3))
    Fyr = Fyr_max * np.tanh((vp.Cr * alpha_r) / (Fyr_max + 1e-3))

    F_drag = 0.5 * vp.air_density * vp.Cd * vp.frontal_area * vx**2
    F_roll = vp.rolling_res * vp.m * vp.g
    Fx_net = vp.m * ax_cmd - F_drag - F_roll

    dvx = (Fx_net - Fyf * np.sin(delta)) / vp.m + vy * r
    dvy = (Fyf * np.cos(delta) + Fyr) / vp.m - vx * r
    dr  = (vp.Lf * Fyf * np.cos(delta) - vp.Lr * Fyr) / vp.Iz

    dX   = vx * np.cos(psi) - vy * np.sin(psi)
    dY   = vx * np.sin(psi) + vy * np.cos(psi)
    dpsi = r

    next_state = np.array([
        X + dX * dt,
        Y + dY * dt,
        wrap_to_pi(psi + dpsi * dt),
        max(0.0, vx + dvx * dt),
        vy + dvy * dt,
        r + dr * dt
    ])
    ay = (Fyf * np.cos(delta) + Fyr) / vp.m
    return next_state, alpha_f, alpha_r, ay

def find_closest_forward(x_ref, y_ref, x_query, y_query, last_idx, search_window=50):
    n = len(x_ref)
    start_idx = max(0, last_idx - 5)
    end_idx = min(n, last_idx + search_window)
    sub_dists = (x_ref[start_idx:end_idx] - x_query)**2 + (y_ref[start_idx:end_idx] - y_query)**2
    return start_idx + np.argmin(sub_dists)

def pure_pursuit(state, traj, last_idx, vp):
    X, Y, psi, vx, _, _ = state
    X_rear = X - vp.Lr * np.cos(psi)
    Y_rear = Y - vp.Lr * np.sin(psi)

    closest_idx = find_closest_forward(traj['x'], traj['y'], X_rear, Y_rear, last_idx)
    path_yaw = traj['yaw'][closest_idx]
    dx_c = X_rear - traj['x'][closest_idx]
    dy_c = Y_rear - traj['y'][closest_idx]
    e_y = -np.sin(path_yaw) * dx_c + np.cos(path_yaw) * dy_c

    Ld = np.clip(0.50 * vx + 2.0, 2.0, 20.0)
    target_idx = closest_idx
    for i in range(closest_idx, len(traj['x'])):
        d = np.sqrt((traj['x'][i] - X_rear)**2 + (traj['y'][i] - Y_rear)**2)
        if d >= Ld:
            target_idx = i
            break

    dx_tgt = traj['x'][target_idx] - X_rear
    dy_tgt = traj['y'][target_idx] - Y_rear
    alpha = wrap_to_pi(np.arctan2(dy_tgt, dx_tgt) - psi)
    Ld_actual = max(0.5, np.sqrt(dx_tgt**2 + dy_tgt**2))

    delta_cmd = np.clip(np.arctan2(2 * vp.L * np.sin(alpha), Ld_actual), -vp.delta_max, vp.delta_max)
    return delta_cmd, e_y, closest_idx

def stanley(state, traj, last_idx, vp):
    X, Y, psi, vx, _, _ = state
    X_front = X + vp.Lf * np.cos(psi)
    Y_front = Y + vp.Lf * np.sin(psi)

    closest_idx = find_closest_forward(traj['x'], traj['y'], X_front, Y_front, last_idx)
    yaw_ref = traj['yaw'][closest_idx]
    
    theta_e = wrap_to_pi(yaw_ref - psi)
    dx = X_front - traj['x'][closest_idx]
    dy = Y_front - traj['y'][closest_idx]
    e_fa = -np.sin(yaw_ref) * dx + np.cos(yaw_ref) * dy

    k_e = 1.4
    k_soft = 1.0
    delta_e = -np.arctan2(k_e * e_fa, max(0.2, vx) + k_soft)
    
    kappa = traj['curvature'][closest_idx]
    delta_ff = np.arctan(vp.L * kappa)

    delta_cmd = np.clip(theta_e + delta_e + 0.85 * delta_ff, -vp.delta_max, vp.delta_max)
    return delta_cmd, e_fa, theta_e, closest_idx

def mpc_lqr_step(state, u_prev, traj, last_idx, vp, Ts=0.05):
    X, Y, psi, vx, _, _ = state
    c_idx = find_closest_forward(traj['x'], traj['y'], X, Y, last_idx)

    psi_ref = traj['yaw'][c_idx]
    v_ref = traj['velocity'][c_idx]
    kappa_ref = traj['curvature'][c_idx]
    a_ref = traj['acceleration'][c_idx]

    dx = X - traj['x'][c_idx]
    dy = Y - traj['y'][c_idx]
    e_y = -np.sin(psi_ref) * dx + np.cos(psi_ref) * dy
    e_psi = wrap_to_pi(psi - psi_ref)
    e_v = vx - v_ref

    x0 = np.array([e_y, e_psi, e_v])
    
    # Linear discrete-time model
    vk = max(1.5, vx)
    A = np.array([[1.0, vk * Ts, 0.0],
                  [0.0, 1.0,     0.0],
                  [0.0, 0.0,     1.0]])
    B = np.array([[0.0,             0.0],
                  [(vk * Ts) / vp.L, 0.0],
                  [0.0,             Ts]])

    # Tuned weights for smooth control without tire saturation
    Q = np.diag([4.0, 8.0, 2.0])
    R = np.diag([120.0, 5.0])

    # Discrete Algebraic Riccati Equation
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

    # Feedback control u_fb = -K * x0
    u_fb = -K @ x0

    # Feedforward control
    delta_ff = np.arctan(vp.L * kappa_ref)
    a_ff = a_ref

    delta_total = np.clip(delta_ff + u_fb[0], -vp.delta_max, vp.delta_max)
    a_total = np.clip(a_ff + u_fb[1], vp.a_min, vp.a_max)

    # Slew rate limits
    d_delta = np.clip(delta_total - u_prev[0], -vp.delta_rate_max * Ts, vp.delta_rate_max * Ts)
    delta_cmd = u_prev[0] + d_delta
    d_a = np.clip(a_total - u_prev[1], -4.0 * Ts, 4.0 * Ts)
    a_cmd = u_prev[1] + d_a

    return np.array([delta_cmd, a_cmd]), e_y, e_psi, e_v, c_idx

def generate_test_scenarios():
    dt = 0.05
    scenarios = {}

    # 1. Pothole Dodge
    t1 = np.arange(0, 12.0 + dt, dt)
    v1 = 10.0
    x1 = v1 * t1
    y1 = np.zeros_like(x1)
    for i, xi in enumerate(x1):
        if 25 <= xi <= 55:
            y1[i] = 1.5 * 0.5 * (1 - np.cos(np.pi * (xi - 25) / 30))
        elif 55 < xi <= 85:
            y1[i] = 1.5 * 0.5 * (1 + np.cos(np.pi * (xi - 55) / 30))

    dx = np.gradient(x1, dt)
    dy = np.gradient(y1, dt)
    yaw1 = np.arctan2(dy, dx)
    ddx = np.gradient(dx, dt)
    ddy = np.gradient(dy, dt)
    kappa1 = (dx * ddy - dy * ddx) / np.maximum(1e-4, (dx**2 + dy**2)**1.5)
    
    scenarios['pothole_dodge'] = {
        'name': 'Pothole Obstacle Avoidance',
        'time': t1, 'x': x1, 'y': y1, 'yaw': yaw1,
        'velocity': np.full_like(t1, v1),
        'acceleration': np.zeros_like(t1),
        'curvature': kappa1
    }

    # 2. Ghat Road Hairpin Bend
    t2 = np.arange(0, 18.0 + dt, dt)
    x2 = np.zeros_like(t2)
    y2 = np.zeros_like(t2)
    v2 = np.zeros_like(t2)
    R_hairpin = 18.0
    for i, ti in enumerate(t2):
        if ti <= 3.0:
            v2[i] = 11.0
            x2[i] = 11.0 * ti
            y2[i] = 0.0
        elif ti <= 11.0:
            tau = (ti - 3.0) / 8.0
            v2[i] = 11.0 - 5.5 * tau
            ang = tau * np.pi
            x2[i] = 33.0 + R_hairpin * np.sin(ang)
            y2[i] = R_hairpin * (1 - np.cos(ang))
        else:
            t_exit = ti - 11.0
            v2[i] = 5.5 + min(4.5, 0.8 * t_exit)
            x2[i] = 33.0 - (5.5 * t_exit + 0.5 * 0.8 * t_exit**2)
            y2[i] = 2 * R_hairpin

    dx2 = np.gradient(x2, dt)
    dy2 = np.gradient(y2, dt)
    yaw2 = np.unwrap(np.arctan2(dy2, dx2))
    ddx2 = np.gradient(dx2, dt)
    ddy2 = np.gradient(dy2, dt)
    kappa2 = (dx2 * ddy2 - dy2 * ddx2) / np.maximum(1e-4, (dx2**2 + dy2**2)**1.5)
    acc2 = np.gradient(v2, dt)

    scenarios['ghat_curve'] = {
        'name': 'Ghat Road Hairpin Curve',
        'time': t2, 'x': x2, 'y': y2, 'yaw': wrap_to_pi(yaw2),
        'velocity': v2, 'acceleration': acc2, 'curvature': kappa2
    }

    return scenarios

def run_simulation():
    vp = VehicleParams()
    scenarios = generate_test_scenarios()
    output_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    print("========================================================================", flush=True)
    print("  SIH 26037: MEGHANA (VEHICLE DYNAMICS & CONTROL) VALIDATION RUNNER", flush=True)
    print("========================================================================", flush=True)

    for sc_key, traj in scenarios.items():
        print(f"\n>>> Running Scenario: {traj['name']} ({sc_key})", flush=True)
        dt = 0.05
        N = len(traj['time'])
        
        results = {}
        for mode in ['pure_pursuit', 'stanley', 'mpc']:
            state = np.array([traj['x'][0], traj['y'][0] + 0.15, traj['yaw'][0], traj['velocity'][0], 0.0, 0.0])
            u_prev = np.array([0.0, 0.0])
            int_v_err = 0.0
            last_c_idx = 0

            log_x = []
            log_y = []
            log_ey = []
            log_delta = []
            log_v = []

            for k in range(N):
                t = traj['time'][k]
                vx = state[3]
                
                # Lateral control
                if mode == 'pure_pursuit':
                    delta_cmd, ey, last_c_idx = pure_pursuit(state, traj, last_c_idx, vp)
                    v_ref = traj['velocity'][last_c_idx]
                    a_ref = traj['acceleration'][last_c_idx]
                    ev = v_ref - vx
                    int_v_err += ev * dt
                    acc_cmd = np.clip(a_ref + 1.2 * ev + 0.15 * int_v_err, vp.a_min, vp.a_max)
                    u_cmd = np.array([delta_cmd, acc_cmd])
                elif mode == 'stanley':
                    delta_cmd, ey, theta_e, last_c_idx = stanley(state, traj, last_c_idx, vp)
                    v_ref = traj['velocity'][last_c_idx]
                    a_ref = traj['acceleration'][last_c_idx]
                    ev = v_ref - vx
                    int_v_err += ev * dt
                    acc_cmd = np.clip(a_ref + 1.2 * ev + 0.15 * int_v_err, vp.a_min, vp.a_max)
                    u_cmd = np.array([delta_cmd, acc_cmd])
                else: # mpc
                    u_cmd, ey, epsi, ev, last_c_idx = mpc_lqr_step(state, u_prev, traj, last_c_idx, vp)
                
                u_prev = u_cmd
                state, alpha_f, alpha_r, ay = dynamic_bicycle_step(state, u_cmd, vp, dt, mu=0.85)

                log_x.append(state[0])
                log_y.append(state[1])
                log_ey.append(ey)
                log_delta.append(u_cmd[0])
                log_v.append(state[3])

            results[mode] = {
                'x': np.array(log_x), 'y': np.array(log_y),
                'ey': np.array(log_ey), 'delta': np.array(log_delta),
                'v': np.array(log_v)
            }
            max_ey = np.max(np.abs(log_ey))
            rmse_ey = np.sqrt(np.mean(np.array(log_ey)**2))
            print(f"    Controller {mode.upper():14s} | Max |ey|: {max_ey:.3f} m | RMSE |ey|: {rmse_ey:.3f} m", flush=True)

        # Plot comparison
        plt.figure(figsize=(14, 9))
        plt.suptitle(f"SIH 26037: Controller Benchmark - {traj['name']}", fontsize=14, fontweight='bold')

        # 1. Global Trajectory
        plt.subplot(2, 2, 1)
        plt.plot(traj['x'], traj['y'], 'k--', linewidth=2, label='Reference Trajectory')
        colors = {'pure_pursuit': 'tab:orange', 'stanley': 'tab:blue', 'mpc': 'tab:green'}
        for mode in ['pure_pursuit', 'stanley', 'mpc']:
            plt.plot(results[mode]['x'], results[mode]['y'], color=colors[mode], label=mode.upper(), linewidth=1.8)
        plt.xlabel('X (m)', fontweight='bold')
        plt.ylabel('Y (m)', fontweight='bold')
        plt.title('Vehicle Trajectory vs Reference (Closed-Loop)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # 2. Cross-track Error
        plt.subplot(2, 2, 2)
        for mode in ['pure_pursuit', 'stanley', 'mpc']:
            plt.plot(traj['time'], results[mode]['ey'], color=colors[mode], label=f"{mode.upper()}", linewidth=1.6)
        plt.xlabel('Time (s)', fontweight='bold')
        plt.ylabel('Cross-track Error ey (m)', fontweight='bold')
        plt.title('Lateral Tracking Error Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # 3. Velocity Profile
        plt.subplot(2, 2, 3)
        plt.plot(traj['time'], traj['velocity'], 'k--', linewidth=2, label='Ref Speed')
        for mode in ['pure_pursuit', 'stanley', 'mpc']:
            plt.plot(traj['time'], results[mode]['v'], color=colors[mode], label=mode.upper(), linewidth=1.5)
        plt.xlabel('Time (s)', fontweight='bold')
        plt.ylabel('Speed (m/s)', fontweight='bold')
        plt.title('Velocity Profile Tracking')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # 4. Steering Command
        plt.subplot(2, 2, 4)
        for mode in ['pure_pursuit', 'stanley', 'mpc']:
            plt.plot(traj['time'], np.rad2deg(results[mode]['delta']), color=colors[mode], label=mode.upper(), linewidth=1.5)
        plt.xlabel('Time (s)', fontweight='bold')
        plt.ylabel('Steering Angle (deg)', fontweight='bold')
        plt.title('Front Steering Actuator Command')
        plt.legend()
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_path = os.path.join(output_dir, f"benchmark_{sc_key}.png")
        plt.savefig(plot_path, dpi=200)
        plt.close()
        print(f"    Saved comparison plot: {plot_path}", flush=True)

    # Emergency braking test
    print("\n>>> Testing Emergency Collision Avoidance Override...", flush=True)
    dt = 0.05
    t_em = np.arange(0, 8.0 + dt, dt)
    v_init = 12.0
    x_em = v_init * t_em
    traj_em = {
        'name': 'Emergency Stop', 'time': t_em, 'x': x_em, 'y': np.zeros_like(x_em),
        'yaw': np.zeros_like(x_em), 'velocity': np.full_like(t_em, v_init),
        'acceleration': np.zeros_like(t_em), 'curvature': np.zeros_like(t_em)
    }

    state = np.array([0.0, 0.0, 0.0, v_init, 0.0, 0.0])
    v_log = []
    brake_log = []
    throttle_log = []
    
    for ti in t_em:
        if ti >= 3.5: # Emergency trigger (cow/pedestrian crossing)
            u_cmd = np.array([0.0, vp.a_emergency])
            brake = 1.0
            throttle = 0.0
        else:
            u_cmd = np.array([0.0, 0.0])
            brake = 0.0
            throttle = 0.35
            
        state, _, _, _ = dynamic_bicycle_step(state, u_cmd, vp, dt)
        v_log.append(state[3])
        brake_log.append(brake)
        throttle_log.append(throttle)

    plt.figure(figsize=(10, 4.5))
    plt.subplot(1, 2, 1)
    plt.plot(t_em, v_log, 'r-', linewidth=2.0)
    plt.axvline(3.5, color='k', linestyle='--', label='Emergency Trigger (t=3.5s)')
    plt.xlabel('Time (s)', fontweight='bold')
    plt.ylabel('Velocity (m/s)', fontweight='bold')
    plt.title('Emergency Braking Velocity Response')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 2, 2)
    plt.plot(t_em, brake_log, 'r-', linewidth=2.0, label='Brake Command')
    plt.plot(t_em, throttle_log, 'g-', linewidth=2.0, label='Throttle Command')
    plt.axvline(3.5, color='k', linestyle='--')
    plt.xlabel('Time (s)', fontweight='bold')
    plt.ylabel('Normalized Command [0-1]', fontweight='bold')
    plt.title('Actuator Commands During Emergency Stop')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    emerg_path = os.path.join(output_dir, "benchmark_emergency_stop.png")
    plt.savefig(emerg_path, dpi=200)
    plt.close()
    print(f"    Saved emergency braking plot: {emerg_path}", flush=True)
    print("\nAll benchmark simulations completed successfully!", flush=True)

if __name__ == '__main__':
    run_simulation()
