% DYNAMIC_BICYCLE - 3-DOF Dynamic Bicycle Model with Nonlinear Tire Saturation
% States: x = [X (m), Y (m), psi (rad), vx (m/s), vy (m/s), r (rad/s)]
% Inputs: u = [delta (front wheel angle, rad), ax (longitudinal acceleration cmd, m/s^2)]
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [x_next, diag_info] = dynamic_bicycle(x, u, params, dt, mu_override)
    if nargin < 5 || isempty(mu_override)
        mu = params.mu_current;
    else
        mu = mu_override;
    end

    % Unpack state
    X   = x(1);
    Y   = x(2);
    psi = x(3);
    vx  = max(0.5, x(4)); % Numerical regularization for low speeds
    vy  = x(5);
    r   = x(6);

    delta = u(1);
    ax_cmd = u(2);

    % Wheel normal loads with longitudinal load transfer
    Fz_f = params.Fz_f_static - (params.m * ax_cmd * 0.5) / params.L;
    Fz_r = params.Fz_r_static + (params.m * ax_cmd * 0.5) / params.L;
    Fz_f = max(500, Fz_f);
    Fz_r = max(500, Fz_r);

    % Slip angles (rad)
    alpha_f = delta - atan2(vy + params.Lf * r, vx);
    alpha_r = -atan2(vy - params.Lr * r, vx);

    % Lateral tire forces using smooth saturation (Brush/tanh tire model)
    % Linear slope at zero slip is Cf, Cr; saturated at mu * Fz
    Fyf_max = mu * Fz_f;
    Fyr_max = mu * Fz_r;
    
    Fyf = Fyf_max * tanh((params.Cf * alpha_f) / (Fyf_max + 1e-3));
    Fyr = Fyr_max * tanh((params.Cr * alpha_r) / (Fyr_max + 1e-3));

    % Longitudinal resistance forces
    F_drag = 0.5 * params.air_density * params.Cd * params.frontal_area * vx^2;
    F_roll = params.rolling_res * params.m * params.g;
    Fx_net = params.m * ax_cmd - F_drag - F_roll;

    % Dynamic equations of motion in body frame
    % m * (dvx - vy * r) = Fx - Fyf * sin(delta)
    dvx = (Fx_net - Fyf * sin(delta)) / params.m + vy * r;
    % m * (dvy + vx * r) = Fyf * cos(delta) + Fyr
    dvy = (Fyf * cos(delta) + Fyr) / params.m - vx * r;
    % Iz * dr = Lf * Fyf * cos(delta) - Lr * Fyr
    dr  = (params.Lf * Fyf * cos(delta) - params.Lr * Fyr) / params.Iz;

    % Inertial frame kinematic derivatives
    dX   = vx * cos(psi) - vy * sin(psi);
    dY   = vx * sin(psi) + vy * cos(psi);
    dpsi = r;

    dx = [dX; dY; dpsi; dvx; dvy; dr];

    % Forward Euler or RK2 step
    x_next = x + dx * dt;
    x_next(3) = wrapToPi(x_next(3));
    x_next(4) = max(0.0, x_next(4)); % Prevent negative forward speed

    % Pack diagnostics for telemetry & feasibility monitoring
    diag_info = struct();
    diag_info.alpha_f = alpha_f;
    diag_info.alpha_r = alpha_r;
    diag_info.Fyf = Fyf;
    diag_info.Fyr = Fyr;
    diag_info.ay_total = (Fyf * cos(delta) + Fyr) / params.m;
    diag_info.mu = mu;
end
