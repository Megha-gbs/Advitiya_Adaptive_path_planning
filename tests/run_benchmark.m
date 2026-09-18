% RUN_BENCHMARK - Automated testing and comparative benchmarking of Pure Pursuit vs Stanley vs MPC
% Problem Statement 26037: SIH - Meghana (Vehicle Dynamics & Control)
% Evaluates tracking performance, stability, and actuator limits on Indian road scenarios.

function benchmark_table = run_benchmark()
    fprintf('========================================================================\n');
    fprintf('  SIH 26037: VEHICLE DYNAMICS & CONTROL BENCHMARK (MEGHANA)\n');
    fprintf('  Adaptive Path Planning & Collision Avoidance on Unstructured Indian Roads\n');
    fprintf('========================================================================\n\n');

    % Add paths
    cur_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(cur_dir);
    addpath(fullfile(project_root, 'config'));
    addpath(fullfile(project_root, 'models'));
    addpath(fullfile(project_root, 'controllers'));
    addpath(fullfile(project_root, 'interfaces'));
    addpath(fullfile(project_root, 'scenarios'));
    addpath(fullfile(project_root, 'simulation'));

    % Generate test scenarios
    scenarios = generate_scenarios();
    scenario_list = {'pothole_dodge', 'ghat_curve', 'double_lane_change'};
    controller_modes = {'pure_pursuit', 'stanley', 'mpc'};

    results_matrix = {};
    row_idx = 1;

    for s = 1:length(scenario_list)
        s_key = scenario_list{s};
        traj = scenarios.(s_key);
        fprintf('>>> Running Benchmark Scenario: %s (%s)\n', traj.name, s_key);

        figure('Name', sprintf('Scenario: %s', traj.name), 'Position', [100, 100, 1200, 800], 'Visible', 'off');
        
        % Subplot 1: XY Trajectory
        subplot(2, 2, 1); hold on; grid on;
        plot(traj.x, traj.y, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Reference Path');

        % Subplot 2: Cross-track Error vs Time
        subplot(2, 2, 2); hold on; grid on;

        % Subplot 3: Velocity Tracking vs Time
        subplot(2, 2, 3); hold on; grid on;
        plot(traj.time, traj.velocity, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Ref Speed');

        % Subplot 4: Steering Command vs Time
        subplot(2, 2, 4); hold on; grid on;

        colors = struct('pure_pursuit', [0.85, 0.33, 0.1], ...
                        'stanley',      [0.0, 0.45, 0.74], ...
                        'mpc',          [0.47, 0.67, 0.19]);
        line_styles = struct('pure_pursuit', '-.', 'stanley', '--', 'mpc', '-');

        for c = 1:length(controller_modes)
            c_mode = controller_modes{c};
            fprintf('    Simulating controller: %-14s ... ', upper(c_mode));
            
            % Run dynamic bicycle simulation with Indian dry asphalt (mu = 0.85)
            tic;
            res = simulate_closed_loop(traj, c_mode, 'dynamic', 0.85, [0.0, 0.2, 0.0, 0.0]); % 0.2m initial lateral offset
            elapsed = toc;
            fprintf('Completed in %.2f sec\n', elapsed);

            m = res.metrics;
            results_matrix(row_idx, :) = {traj.name, upper(c_mode), ...
                m.max_abs_ey, m.rmse_ey, m.max_abs_epsi, m.max_abs_ev, m.max_lat_accel, m.control_effort};
            row_idx = row_idx + 1;

            col = colors.(c_mode);
            ls = line_styles.(c_mode);

            % Plot XY
            subplot(2, 2, 1);
            plot(res.x, res.y, 'Color', col, 'LineStyle', ls, 'LineWidth', 1.8, 'DisplayName', upper(c_mode));

            % Plot ey
            subplot(2, 2, 2);
            plot(res.time, res.ey, 'Color', col, 'LineStyle', ls, 'LineWidth', 1.5, 'DisplayName', upper(c_mode));

            % Plot velocity
            subplot(2, 2, 3);
            plot(res.time, res.v, 'Color', col, 'LineStyle', ls, 'LineWidth', 1.5, 'DisplayName', upper(c_mode));

            % Plot steering
            subplot(2, 2, 4);
            plot(res.time, rad2deg(res.delta), 'Color', col, 'LineStyle', ls, 'LineWidth', 1.5, 'DisplayName', upper(c_mode));
        end

        % Format plots
        subplot(2, 2, 1);
        xlabel('X [m]', 'FontWeight', 'bold'); ylabel('Y [m]', 'FontWeight', 'bold');
        title(sprintf('Global Path Tracking: %s', traj.name));
        legend('Location', 'best');

        subplot(2, 2, 2);
        xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Lateral Error e_y [m]', 'FontWeight', 'bold');
        title('Cross-Track Tracking Error');
        legend('Location', 'best');

        subplot(2, 2, 3);
        xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Speed [m/s]', 'FontWeight', 'bold');
        title('Longitudinal Velocity Profile');
        legend('Location', 'best');

        subplot(2, 2, 4);
        xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Steering Angle \delta [deg]', 'FontWeight', 'bold');
        title('Control Command: Front Steering Angle');
        legend('Location', 'best');

        % Save plot
        fig_path = fullfile(project_root, sprintf('benchmark_%s.png', s_key));
        saveas(gcf, fig_path);
        close(gcf);
        fprintf('    [Saved benchmark figure: %s]\n\n', fig_path);
    end

    %% Scenario 4: Test Emergency Collision Avoidance (Safety Override Verification)
    fprintf('>>> Testing Safety Supervisor Emergency Collision Avoidance Override...\n');
    res_emergency = simulate_closed_loop(scenarios.emergency_stop, 'mpc', 'dynamic', 0.85);
    
    fig_emerg = figure('Name', 'Emergency Stop Verification', 'Position', [150, 150, 1000, 500], 'Visible', 'off');
    subplot(1, 2, 1); hold on; grid on;
    plot(res_emergency.time, res_emergency.v, 'r-', 'LineWidth', 2.0);
    xline(3.5, 'k--', 'Emergency Triggered', 'LineWidth', 1.5, 'LabelOrientation', 'horizontal');
    xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Velocity [m/s]', 'FontWeight', 'bold');
    title('Vehicle Speed Under Emergency Stop');

    subplot(1, 2, 2); hold on; grid on;
    plot(res_emergency.time, res_emergency.brake, 'Color', [0.8, 0.1, 0.1], 'LineWidth', 2.0, 'DisplayName', 'Brake');
    plot(res_emergency.time, res_emergency.throttle, 'Color', [0.1, 0.7, 0.2], 'LineWidth', 2.0, 'DisplayName', 'Throttle');
    xline(3.5, 'k--', 'Emergency Triggered', 'LineWidth', 1.5);
    xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Normalized Actuator Command [0-1]', 'FontWeight', 'bold');
    title('Actuator Commands: Emergency Braking');
    legend('Location', 'best');
    
    saveas(fig_emerg, fullfile(project_root, 'benchmark_emergency_stop.png'));
    close(fig_emerg);
    fprintf('    [Saved emergency override figure: benchmark_emergency_stop.png]\n\n');

    %% Create Formatted Comparison Table
    col_names = {'Scenario', 'Controller', 'Max_ey_m', 'RMSE_ey_m', 'Max_epsi_deg', 'Max_ev_mps', 'Max_ay_mps2', 'Control_Effort'};
    benchmark_table = cell2table(results_matrix, 'VariableNames', col_names);
    
    disp('========================================================================');
    disp('                   QUANTITATIVE PERFORMANCE COMPARISON                   ');
    disp('========================================================================');
    disp(benchmark_table);

    % Save Table to CSV
    csv_path = fullfile(project_root, 'benchmark_results.csv');
    writetable(benchmark_table, csv_path);
    fprintf('Saved quantitative benchmark report to: %s\n', csv_path);
end
