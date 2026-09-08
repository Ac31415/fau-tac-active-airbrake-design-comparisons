%% run_airbrake_comparison.m
% Comprehensive comparison runner for Active Airbrake Flight Control in 3D 6-DOF.
% Compares:
%   1. Baseline Uncontrolled (Clean rocket, no airbrakes)
%   2. Design 1: Downward Pivoting Flaps (0 - 90 deg) with Savox SC-1258TG Digital Servo
%   3. Design 2: Radially Outward Sliding Flaps at 90 deg with Actuonix L16 Linear Actuator
%   4. Design 2 at Variable Sliding Speeds: Fast (45 mm/s), Nominal (32 mm/s), Slow (16 mm/s)
%
% Generates engineering comparison plots and exports summary metrics.
%
% Copyright (c) 2026 Aerospace Systems Laboratory

clear; clc; close all;

% Ensure search paths
proj_root = fileparts(fileparts(mfilename('fullpath')));
if isempty(proj_root), proj_root = pwd; end
addpath(fullfile(proj_root, 'scripts'));
addpath(fullfile(proj_root, 'functions'));

results_dir = fullfile(proj_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

fprintf('=========================================================================\n');
fprintf('   6-DOF ACTIVE AIRBRAKE COMPARATIVE FLIGHT SIMULATION RUNNER           \n');
fprintf('=========================================================================\n\n');

%% 1. Run Simulations for All Design Configurations
fprintf('>> Running Scenario 1: Baseline Uncontrolled (No Airbrakes)...\n');
sim_base = rocket_6dof_simulator('baseline_none', false);

fprintf('\n>> Running Scenario 2: Design 1 (Downward Hinged Flaps, 0-90 deg)...\n');
sim_d1 = rocket_6dof_simulator('design1_hinged', true);

fprintf('\n>> Running Scenario 3: Design 2 (Radial Sliding Flaps, Nominal 32 mm/s)...\n');
sim_d2_nom = rocket_6dof_simulator('design2_sliding', true, 0.032);

fprintf('\n>> Running Scenario 4: Design 2 (Radial Sliding Flaps, Fast 45 mm/s)...\n');
sim_d2_fast = rocket_6dof_simulator('design2_sliding', true, 0.045);

fprintf('\n>> Running Scenario 5: Design 2 (Radial Sliding Flaps, Slow 16 mm/s)...\n');
sim_d2_slow = rocket_6dof_simulator('design2_sliding', true, 0.016);

%% 2. Calculate Detailed Performance Metrics
scenarios = {sim_base, sim_d1, sim_d2_nom, sim_d2_fast, sim_d2_slow};
labels    = {'Baseline (Clean)', 'Design 1 (Hinged 0-90°)', ...
             'Design 2 (Sliding 32 mm/s)', 'Design 2 (Sliding 45 mm/s)', ...
             'Design 2 (Sliding 16 mm/s)'};
N_scen = length(scenarios);

metrics = struct();
for i = 1:N_scen
    s = scenarios{i};
    metrics(i).label         = labels{i};
    metrics(i).apogee_m      = s.h_apogee;
    metrics(i).apogee_ft     = s.h_apogee * 3.28084;
    metrics(i).error_m       = s.apogee_error;
    metrics(i).pct_error     = (s.apogee_error / s.target_apogee) * 100;
    metrics(i).t_apogee      = s.t_apogee;
    metrics(i).max_mach      = max(s.mach);
    metrics(i).max_q_kpa     = max(s.q_inf) / 1000.0;
    metrics(i).max_CD        = max(s.CD_total);
    metrics(i).max_act_pos   = max(s.act_pos);
    metrics(i).max_load      = max(s.act_load);
    metrics(i).max_current_A = max(s.current_A);
    metrics(i).min_vbus_V    = min(s.vbus_V);
    
    % Energy consumed in Joules: integral of power dt
    energy_J = trapz(s.t, s.power_W);
    energy_mah = (trapz(s.t, s.current_A) / 3600.0) * 1000.0;
    metrics(i).energy_J      = energy_J;
    metrics(i).energy_mah    = energy_mah;
end

% Print Formatted Comparison Table
fprintf('\n');
fprintf('========================================================================================================\n');
fprintf('                                     PERFORMANCE & DESIGN METRICS TABLE                                \n');
fprintf('========================================================================================================\n');
fprintf('%-28s | %-10s | %-9s | %-8s | %-10s | %-10s | %-8s\n', ...
    'Configuration', 'Apogee (m)', 'Error (m)', 'Mach Peak', 'Peak Load', 'Peak I (A)', 'Batt (mAh)');
fprintf('--------------------------------------------------------------------------------------------------------\n');
for i = 1:N_scen
    m = metrics(i);
    if i == 2
        load_str = sprintf('%.2f N*m', m.max_load);
    elseif i >= 3
        load_str = sprintf('%.1f N', m.max_load);
    else
        load_str = 'N/A';
    end
    fprintf('%-28s | %10.1f | %+9.1f | %8.2f | %10s | %10.2f | %8.2f\n', ...
        m.label, m.apogee_m, m.error_m, m.max_mach, load_str, m.max_current_A, m.energy_mah);
end
fprintf('========================================================================================================\n\n');

%% 3. Generate Publication-Quality Plots

% Color Palette
c_base = [0.2, 0.2, 0.2];     % Dark Gray
c_d1   = [0.85, 0.33, 0.10];   % Coral / Orange
c_d2   = [0.00, 0.45, 0.74];   % Ocean Blue
c_fast = [0.47, 0.67, 0.19];   % Green
c_slow = [0.49, 0.18, 0.56];   % Purple

set(groot, 'defaultAxesFontSize', 10);
set(groot, 'defaultLineLineWidth', 1.6);

%% FIGURE 1: 3D Trajectory in 6-DOF
fig1 = figure('Name', '3D Trajectory', 'Position', [50, 50, 950, 700], 'Color', 'w');
hold on; grid on; box on;
plot3(sim_base.pos_ned(:, 2), sim_base.pos_ned(:, 1), -sim_base.pos_ned(:, 3), ...
    'Color', c_base, 'LineStyle', '--', 'DisplayName', 'Baseline Uncontrolled');
plot3(sim_d1.pos_ned(:, 2), sim_d1.pos_ned(:, 1), -sim_d1.pos_ned(:, 3), ...
    'Color', c_d1, 'DisplayName', 'Design 1 (Hinged Flaps)');
plot3(sim_d2_nom.pos_ned(:, 2), sim_d2_nom.pos_ned(:, 1), -sim_d2_nom.pos_ned(:, 3), ...
    'Color', c_d2, 'DisplayName', 'Design 2 (Sliding Flaps, 32 mm/s)');

% Mark apogees
plot3(sim_base.pos_ned(end, 2), sim_base.pos_ned(end, 1), sim_base.h_apogee, ...
    'ko', 'MarkerSize', 9, 'MarkerFaceColor', c_base, 'DisplayName', sprintf('Baseline Apogee (%.0f m)', sim_base.h_apogee));
plot3(sim_d1.pos_ned(end, 2), sim_d1.pos_ned(end, 1), sim_d1.h_apogee, ...
    's', 'MarkerSize', 10, 'MarkerFaceColor', c_d1, 'DisplayName', sprintf('Design 1 Apogee (%.0f m)', sim_d1.h_apogee));
plot3(sim_d2_nom.pos_ned(end, 2), sim_d2_nom.pos_ned(end, 1), sim_d2_nom.h_apogee, ...
    '^', 'MarkerSize', 10, 'MarkerFaceColor', c_d2, 'DisplayName', sprintf('Design 2 Apogee (%.0f m)', sim_d2_nom.h_apogee));

% Target apogee horizontal plane grid
[X_grid, Y_grid] = meshgrid(linspace(-50, 200, 10), linspace(-50, 200, 10));
Z_grid = ones(size(X_grid)) * sim_d1.target_apogee;
mesh(X_grid, Y_grid, Z_grid, 'FaceAlpha', 0.15, 'EdgeColor', [0.2, 0.7, 0.2], ...
    'LineStyle', ':', 'DisplayName', sprintf('Target Apogee (%d m)', sim_d1.target_apogee));

xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude MSL (m)');
title('3D 6-DOF Flight Trajectory under Crosswind & Active Airbrake Control', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
view(45, 25);
saveas(fig1, fullfile(results_dir, 'figure1_3d_trajectory.png'));

%% FIGURE 2: Altitude & Velocity Dynamics vs Time
fig2 = figure('Name', 'Altitude and Velocity Dynamics', 'Position', [100, 100, 1100, 750], 'Color', 'w');

subplot(2, 2, 1); hold on; grid on; box on;
plot(sim_base.t, sim_base.altitude, 'Color', c_base, 'LineStyle', '--', 'DisplayName', 'Baseline');
plot(sim_d1.t, sim_d1.altitude, 'Color', c_d1, 'DisplayName', 'Design 1 (Hinged)');
plot(sim_d2_nom.t, sim_d2_nom.altitude, 'Color', c_d2, 'DisplayName', 'Design 2 (Sliding)');
yline(sim_d1.target_apogee, 'g--', 'LineWidth', 1.8, 'DisplayName', sprintf('Target (%d m)', sim_d1.target_apogee));
xlabel('Time (s)'); ylabel('Altitude MSL (m)');
title('Altitude vs Time', 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0, 26]);

subplot(2, 2, 2); hold on; grid on; box on;
% Zoomed view near apogee
idx_zoom_base = sim_base.t >= 18.0;
idx_zoom_d1   = sim_d1.t >= 18.0;
idx_zoom_d2   = sim_d2_nom.t >= 18.0;
plot(sim_base.t(idx_zoom_base), sim_base.altitude(idx_zoom_base), 'Color', c_base, 'LineStyle', '--');
plot(sim_d1.t(idx_zoom_d1), sim_d1.altitude(idx_zoom_d1), 'Color', c_d1);
plot(sim_d2_nom.t(idx_zoom_d2), sim_d2_nom.altitude(idx_zoom_d2), 'Color', c_d2);
yline(sim_d1.target_apogee, 'g--', 'LineWidth', 1.8);
xlabel('Time (s)'); ylabel('Altitude MSL (m)');
title('Apogee Arrival Zoom (18 s - 25 s)', 'FontWeight', 'bold');
xlim([18, 25]); ylim([2850, 3500]);

subplot(2, 2, 3); hold on; grid on; box on;
plot(sim_base.t, -sim_base.vel_ned(:, 3), 'Color', c_base, 'LineStyle', '--', 'DisplayName', 'Baseline');
plot(sim_d1.t, -sim_d1.vel_ned(:, 3), 'Color', c_d1, 'DisplayName', 'Design 1');
plot(sim_d2_nom.t, -sim_d2_nom.vel_ned(:, 3), 'Color', c_d2, 'DisplayName', 'Design 2');
yline(0, 'k:', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Vertical Velocity (m/s)');
title('Vertical Velocity vs Time', 'FontWeight', 'bold');
xlim([0, 26]);

subplot(2, 2, 4); hold on; grid on; box on;
plot(sim_base.t, sim_base.mach, 'Color', c_base, 'LineStyle', '--', 'DisplayName', 'Baseline');
plot(sim_d1.t, sim_d1.mach, 'Color', c_d1, 'DisplayName', 'Design 1');
plot(sim_d2_nom.t, sim_d2_nom.mach, 'Color', c_d2, 'DisplayName', 'Design 2');
yline(1.0, 'r:', 'LineWidth', 1.2, 'DisplayName', 'Mach 1.0');
xlabel('Time (s)'); ylabel('Mach Number');
title('Flight Mach Number vs Time', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 26]);

saveas(fig2, fullfile(results_dir, 'figure2_flight_dynamics.png'));

%% FIGURE 3: Airbrake Kinematics & Variable Sliding Speeds
fig3 = figure('Name', 'Airbrake Kinematics & Speeds', 'Position', [150, 150, 1100, 750], 'Color', 'w');

subplot(2, 2, 1); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.act_pos, 'Color', c_d1, 'DisplayName', 'Actual Flap Angle');
plot(sim_d1.t, sim_d1.act_cmd * 90.0, 'Color', [0.85, 0.33, 0.10, 0.4], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'Command Angle');
xlabel('Time (s)'); ylabel('Flap Angle (deg)');
title('Design 1: Downward Flap Angle (0° to 90°)', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]); ylim([0, 95]);

subplot(2, 2, 2); hold on; grid on; box on;
plot(sim_d2_nom.t, sim_d2_nom.act_pos, 'Color', c_d2, 'DisplayName', 'Actual Stroke');
plot(sim_d2_nom.t, sim_d2_nom.act_cmd * 35.0, 'Color', [0.0, 0.45, 0.74, 0.4], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'Command Stroke');
xlabel('Time (s)'); ylabel('Extension Stroke (mm)');
title('Design 2: Radial Sliding Stroke (0 to 35 mm)', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]); ylim([0, 38]);

subplot(2, 2, 3); hold on; grid on; box on;
plot(sim_d2_fast.t, sim_d2_fast.act_pos, 'Color', c_fast, 'DisplayName', 'Fast (45 mm/s)');
plot(sim_d2_nom.t, sim_d2_nom.act_pos, 'Color', c_d2, 'DisplayName', 'Nominal (32 mm/s)');
plot(sim_d2_slow.t, sim_d2_slow.act_pos, 'Color', c_slow, 'DisplayName', 'Slow (16 mm/s)');
xlabel('Time (s)'); ylabel('Extension Stroke (mm)');
title('Design 2: Effect of Actuator Sliding Speed', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([3, 15]); ylim([0, 30]);

subplot(2, 2, 4); hold on; grid on; box on;
plot(sim_base.t, sim_base.CD_total, 'Color', c_base, 'LineStyle', '--', 'DisplayName', 'Baseline Clean');
plot(sim_d1.t, sim_d1.CD_total, 'Color', c_d1, 'DisplayName', 'Design 1 (Hinged)');
plot(sim_d2_nom.t, sim_d2_nom.CD_total, 'Color', c_d2, 'DisplayName', 'Design 2 (Sliding)');
xlabel('Time (s)'); ylabel('Total Drag Coefficient (C_D)');
title('Total Drag Coefficient Modulation', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]);

saveas(fig3, fullfile(results_dir, 'figure3_airbrake_kinematics.png'));

%% FIGURE 4: Aerodynamic & Mechanical Actuation Loads
fig4 = figure('Name', 'Actuator Mechanical Loads', 'Position', [200, 200, 1100, 750], 'Color', 'w');

subplot(2, 2, 1); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.q_inf / 1000.0, 'Color', [0.3, 0.3, 0.3], 'DisplayName', 'Dynamic Pressure');
xlabel('Time (s)'); ylabel('Dynamic Pressure q_\infty (kPa)');
title('Flight Dynamic Pressure History', 'FontWeight', 'bold');
xlim([0, 25]);

subplot(2, 2, 2); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.act_load, 'Color', c_d1, 'DisplayName', 'Aerodynamic Hinge Torque');
yline(1.47, 'r--', 'LineWidth', 1.4, 'DisplayName', 'Savox SC-1258TG Stall Torque (1.47 N*m)');
xlabel('Time (s)'); ylabel('Torque Load (N*m)');
title('Design 1: Aerodynamic Hinge Torque on Servo', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]);

subplot(2, 2, 3); hold on; grid on; box on;
plot(sim_d2_nom.t, sim_d2_nom.act_load, 'Color', c_d2, 'DisplayName', 'Guide Rail Friction Force');
yline(50.0, 'r--', 'LineWidth', 1.4, 'DisplayName', 'Actuonix L16 Stall Force (50 N)');
xlabel('Time (s)'); ylabel('Rail Friction Load (N)');
title('Design 2: Guide Rail Sliding Friction Force', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]);

subplot(2, 2, 4); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.dCD_brakes, 'Color', c_d1, 'DisplayName', '\Delta C_D (Hinged Flap)');
plot(sim_d2_nom.t, sim_d2_nom.dCD_brakes, 'Color', c_d2, 'DisplayName', '\Delta C_D (Sliding Flap)');
xlabel('Time (s)'); ylabel('Airbrake Drag Increment (\Delta C_D)');
title('Control Authority (\Delta C_D Comparison)', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]);

saveas(fig4, fullfile(results_dir, 'figure4_mechanical_loads.png'));

%% FIGURE 5: Avionics Electrical System & Power Bus
fig5 = figure('Name', 'Electronics and Power Bus', 'Position', [250, 250, 1100, 750], 'Color', 'w');

subplot(2, 2, 1); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.current_A, 'Color', c_d1, 'DisplayName', 'Design 1 (Savox Servo)');
plot(sim_d2_nom.t, sim_d2_nom.current_A, 'Color', c_d2, 'DisplayName', 'Design 2 (Actuonix L16)');
xlabel('Time (s)'); ylabel('Current Draw (A)');
title('Total Electrical Bus Current Draw', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]);

subplot(2, 2, 2); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.vbus_V, 'Color', c_d1, 'DisplayName', 'Design 1 Bus');
plot(sim_d2_nom.t, sim_d2_nom.vbus_V, 'Color', c_d2, 'DisplayName', 'Design 2 Bus');
yline(7.4, 'k:', 'LineWidth', 1.2, 'DisplayName', '2S LiPo Nominal (7.4 V)');
xlabel('Time (s)'); ylabel('Battery Bus Voltage (V)');
title('2S LiPo Battery Voltage Sag under Load', 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0, 25]);

subplot(2, 2, 3); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.power_W, 'Color', c_d1, 'DisplayName', 'Design 1 Power');
plot(sim_d2_nom.t, sim_d2_nom.power_W, 'Color', c_d2, 'DisplayName', 'Design 2 Power');
xlabel('Time (s)'); ylabel('Power (Watts)');
title('Instantaneous Electrical Power', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]);

subplot(2, 2, 4); hold on; grid on; box on;
% Cumulative energy in Joules
cum_e_d1 = cumtrapz(sim_d1.t, sim_d1.power_W);
cum_e_d2 = cumtrapz(sim_d2_nom.t, sim_d2_nom.power_W);
plot(sim_d1.t, cum_e_d1, 'Color', c_d1, 'DisplayName', sprintf('Design 1 (%.1f J)', cum_e_d1(end)));
plot(sim_d2_nom.t, cum_e_d2, 'Color', c_d2, 'DisplayName', sprintf('Design 2 (%.1f J)', cum_e_d2(end)));
xlabel('Time (s)'); ylabel('Cumulative Energy (Joules)');
title('Cumulative Electrical Energy Consumed', 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0, 25]);

saveas(fig5, fullfile(results_dir, 'figure5_avionics_power.png'));

%% FIGURE 6: Flight Computer Sensor Fusion & Discrete PID Control
fig6 = figure('Name', 'Flight Computer PID & Prediction', 'Position', [300, 300, 1100, 750], 'Color', 'w');

subplot(2, 2, 1); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.altitude, 'k--', 'LineWidth', 1.2, 'DisplayName', 'True Altitude');
plot(sim_d1.t, sim_d1.h_est, 'Color', c_d1, 'DisplayName', 'Kalman Filter \hat{h}');
xlabel('Time (s)'); ylabel('Altitude MSL (m)');
title('Teensy 4.1 Sensor Fusion (BMP388 + BMI088)', 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0, 25]);

subplot(2, 2, 2); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.h_pred, 'Color', c_d1, 'DisplayName', 'Predicted Apogee h_{pred}');
yline(sim_d1.target_apogee, 'g--', 'LineWidth', 1.8, 'DisplayName', 'Target (3000 m)');
xlabel('Time (s)'); ylabel('Apogee (m)');
title('Dynamic Ballistic Apogee Prediction', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([3, 24]); ylim([2800, 3500]);

subplot(2, 2, 3); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.pid_err, 'Color', [0.8, 0.1, 0.1], 'DisplayName', 'Error (h_{pred} - h_{tgt})');
yline(0, 'k:');
xlabel('Time (s)'); ylabel('Error (m)');
title('PID Apogee Regulation Error', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([3, 24]);

subplot(2, 2, 4); hold on; grid on; box on;
plot(sim_d1.t, sim_d1.pwm_us, 'Color', c_d1, 'DisplayName', 'Design 1 PWM');
plot(sim_d2_nom.t, sim_d2_nom.pwm_us, 'Color', c_d2, 'DisplayName', 'Design 2 PWM');
yline(1000, 'k:', 'DisplayName', 'Min (1000 \mus)');
yline(2000, 'k:', 'DisplayName', 'Max (2000 \mus)');
xlabel('Time (s)'); ylabel('PWM Pulse Width (\mus)');
title('Flight Computer Actuator PWM Command', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 25]); ylim([900, 2100]);

saveas(fig6, fullfile(results_dir, 'figure6_pid_control.png'));

fprintf('\n>> All 6 comparative figures generated and saved to "%s"!\n', results_dir);
fprintf('>> Active airbrake 6-DOF simulation comparison complete.\n\n');
