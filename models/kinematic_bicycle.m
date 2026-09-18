% KINEMATIC_BICYCLE - Nonlinear kinematic bicycle model for autonomous vehicle simulation
% States: x = [X (m), Y (m), psi (rad), v (m/s)]
% Inputs: u = [delta (front wheel angle, rad), a (acceleration, m/s^2)]
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [x_next, dx] = kinematic_bicycle(x, u, params, dt)
    % Unpack state
    X   = x(1);
    Y   = x(2);
    psi = x(3);
    v   = x(4);

    % Unpack inputs
    delta = u(1);
    a     = u(2);

    % Sideslip angle at CG
    beta = atan((params.Lr / (params.Lf + params.Lr)) * tan(delta));

    % Continuous-time state derivatives
    dX   = v * cos(psi + beta);
    dY   = v * sin(psi + beta);
    dpsi = (v / params.Lr) * sin(beta);
    dv   = a;

    % Prevent backward motion if speed goes below zero under braking
    if v + dv*dt < 0
        dv = -v / dt;
    end

    dx = [dX; dY; dpsi; dv];

    % Numerical integration (Runge-Kutta 4th Order for high accuracy)
    k1 = dx;
    
    % Midpoint 1
    beta_k2 = atan((params.Lr / (params.Lf + params.Lr)) * tan(delta));
    v_k2 = v + 0.5 * dt * k1(4);
    psi_k2 = psi + 0.5 * dt * k1(3);
    k2 = [v_k2 * cos(psi_k2 + beta_k2);
          v_k2 * sin(psi_k2 + beta_k2);
          (v_k2 / params.Lr) * sin(beta_k2);
          a];
      
    % Midpoint 2
    beta_k3 = atan((params.Lr / (params.Lf + params.Lr)) * tan(delta));
    v_k3 = v + 0.5 * dt * k2(4);
    psi_k3 = psi + 0.5 * dt * k2(3);
    k3 = [v_k3 * cos(psi_k3 + beta_k3);
          v_k3 * sin(psi_k3 + beta_k3);
          (v_k3 / params.Lr) * sin(beta_k3);
          a];

    % End point
    beta_k4 = atan((params.Lr / (params.Lf + params.Lr)) * tan(delta));
    v_k4 = v + dt * k3(4);
    psi_k4 = psi + dt * k3(3);
    k4 = [v_k4 * cos(psi_k4 + beta_k4);
          v_k4 * sin(psi_k4 + beta_k4);
          (v_k4 / params.Lr) * sin(beta_k4);
          a];

    x_next = x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
    x_next(3) = wrapToPi(x_next(3));
    x_next(4) = max(0.0, x_next(4)); % speed non-negative
end
