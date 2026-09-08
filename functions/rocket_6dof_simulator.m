function sim_data = rocket_6dof_simulator(design_type, enable_control, custom_actuator_speed, custom_target_apogee)
% ROCKET_6DOF_SIMULATOR Full 3D 6-Degrees-of-Freedom Flight Dynamics Simulator
% for Sounding Rockets with Active Airbrake Flight Control.
%
% Inputs:
%   design_type           - 'design1_hinged', 'design2_sliding', or 'baseline_none'
%   enable_control        - Boolean (true = active PID airbrake, false = locked retracted)
%   custom_actuator_speed - (Optional) Override linear actuator speed for Design 2 (m/s)
%   custom_target_apogee  - (Optional) Override target apogee (m)
%
% Outputs:
%   sim_data              - Complete time-series history structure of all flight,
%                           aerodynamic, electronic, and control variables.

% 1. Load Parameters
init_airbrake_params;

if nargin >= 4 && ~isempty(custom_target_apogee)
    gnc.target_apogee = custom_target_apogee;
end
if nargin >= 3 && ~isempty(custom_actuator_speed)
    electronics.linear_actuator.max_speed_nominal = custom_actuator_speed;
end
if nargin < 2
    enable_control = true;
end
if nargin < 1
    design_type = 'design1_hinged';
end

if strcmpi(design_type, 'baseline_none')
    enable_control = false;
    design_type = 'design1_hinged'; % Geometry placeholder with u=0
end

% 2. Simulation Time Grid Setup
dt = sim_params.dt;         % Plant integration step (0.002 s = 500 Hz)
dt_fsw = sim_params.dt_fsw; % Flight software step (0.02 s = 50 Hz)
t_vec = 0:dt:sim_params.t_max;
N_steps = length(t_vec);

% 3. Initial Conditions
% Initial orientation: Rail elevation and azimuth
theta0_rad = sim_params.rail_angle * (pi / 180);
psi0_rad   = sim_params.rail_azimuth * (pi / 180);
phi0_rad   = 0.0;

% Quaternion [q0; q1; q2; q3] (scalar first)
q0 = [cos(theta0_rad/2)*cos(psi0_rad/2);
      -sin(theta0_rad/2)*sin(psi0_rad/2);
       sin(theta0_rad/2)*cos(psi0_rad/2);
       cos(theta0_rad/2)*sin(psi0_rad/2)];
q0 = q0 / norm(q0);

% Position in NED frame [X (North); Y (East); Z (Down)]
r_ned = [0.0; 0.0; -sim_params.altitude_pad];

% Velocity in Body frame [u; v; w]
V_b = [0.0; 0.0; 0.0];

% Angular rates in Body frame [p; q; r]
omega_b = [0.0; 0.0; 0.0];

% Actuator continuous state:
% Design 1: [theta (rad); theta_dot (rad/s)]
% Design 2: [s (m); s_dot (m/s)]
act_state = [0.0; 0.0];

% FSW & Sensor persistent structs
fsw_state = [];
sensor_state = [];
fsw_cmd = 0.0;
fsw_out = struct('u_cmd', 0, 'pwm_us', 1000, 'flight_mode', 0, ...
                 'h_est', sim_params.altitude_pad, 'vz_est', 0, ...
                 'h_pred', sim_params.altitude_pad, 'err', 0, 'integral', 0);
elec_diag = struct('pos_actual', 0, 'speed_actual', 0, 'load_reaction', 0, ...
                   'i_actuator', 0, 'v_bus', electronics.battery.v_full, 'power_watts', 0);

% Rail distance tracker
dist_on_rail = 0.0;
on_rail = true;

% 4. Preallocate Logging Buffers
log_t          = zeros(N_steps, 1);
log_pos_ned    = zeros(N_steps, 3);
log_vel_ned    = zeros(N_steps, 3);
log_vel_b      = zeros(N_steps, 3);
log_omega_b    = zeros(N_steps, 3);
log_euler_deg  = zeros(N_steps, 3); % [roll; pitch; yaw]
log_altitude   = zeros(N_steps, 1);
log_mach       = zeros(N_steps, 1);
log_q_inf      = zeros(N_steps, 1);
log_alpha_deg  = zeros(N_steps, 1);
log_CD_total   = zeros(N_steps, 1);
log_dCD_brakes = zeros(N_steps, 1);
log_thrust     = zeros(N_steps, 1);
log_mass       = zeros(N_steps, 1);

% Actuator & Electrical logs
log_act_pos    = zeros(N_steps, 1); % deg or mm
log_act_cmd    = zeros(N_steps, 1); % fraction [0, 1]
log_act_load   = zeros(N_steps, 1); % N*m or N
log_current_A  = zeros(N_steps, 1);
log_vbus_V     = zeros(N_steps, 1);
log_power_W    = zeros(N_steps, 1);

% FSW & PID logs
log_fsw_mode   = zeros(N_steps, 1);
log_h_est      = zeros(N_steps, 1);
log_vz_est     = zeros(N_steps, 1);
log_h_pred     = zeros(N_steps, 1);
log_pid_err    = zeros(N_steps, 1);
log_pid_u      = zeros(N_steps, 1);
log_pwm_us     = zeros(N_steps, 1);

apogee_reached = false;
t_apogee = 0.0;
h_apogee = 0.0;
step_final = N_steps;

% 5. Main 6-DOF Integration Loop
fprintf('   Simulating %s (Control: %d)...', design_type, enable_control);
fsw_timer = 0.0;

for k = 1:N_steps
    t = t_vec(k);
    h_curr = -r_ned(3); % Altitude MSL
    
    % Direction Cosine Matrix: DCM body from earth (C_b_e)
    C_b_e = quat_to_dcm(q0);
    C_e_b = C_b_e'; % Earth from body
    
    % Velocity in NED
    V_ned = C_e_b * V_b;
    vz_up = -V_ned(3);
    
    % Mass, CG, and Inertias
    if t <= rocket.t_burn
        % Fractional propellant burnt
        eta_burn = min(1.0, t / rocket.t_burn);
        m_curr = rocket.m_liftoff - eta_burn * rocket.m_prop;
        x_CG_curr = rocket.x_CG_wet + eta_burn * (rocket.x_CG_dry - rocket.x_CG_wet);
        Ixx = rocket.Ixx_wet + eta_burn * (rocket.Ixx_dry - rocket.Ixx_wet);
        Iyy = rocket.Iyy_wet + eta_burn * (rocket.Iyy_dry - rocket.Iyy_wet);
        Izz = rocket.Izz_wet + eta_burn * (rocket.Izz_dry - rocket.Izz_wet);
        T_curr = interp1(rocket.thrust_lut_t, rocket.thrust_lut_T, t, 'linear', 0.0);
    else
        m_curr = rocket.m_dry;
        x_CG_curr = rocket.x_CG_dry;
        Ixx = rocket.Ixx_dry;
        Iyy = rocket.Iyy_dry;
        Izz = rocket.Izz_dry;
        T_curr = 0.0;
    end
    I_mat = diag([Ixx, Iyy, Izz]);
    
    % Check Rail Release
    if on_rail
        dist_on_rail = norm(r_ned - [0; 0; -sim_params.altitude_pad]);
        if dist_on_rail >= sim_params.rail_length
            on_rail = false;
        end
    end
    
    % Ambient Wind Field (3D)
    wind_az_rad = sim_params.wind_azimuth * (pi / 180);
    wind_speed = sim_params.wind_base_speed * (max(1.0, (h_curr - sim_params.altitude_pad) / 10.0)^sim_params.wind_shear_exp) + ...
                 sim_params.gust_amplitude * sin(2 * pi * sim_params.gust_frequency * t);
    % Wind vector blowing towards azimuth
    V_wind_ned = [-wind_speed * cos(wind_az_rad); -wind_speed * sin(wind_az_rad); 0.0];
    V_wind_b   = C_b_e * V_wind_ned;
    
    % Thrust Force in Body Frame
    F_thrust_b = [T_curr; 0.0; 0.0];
    
    % Current Actuator Reaction Load from previous step (or 0 initially)
    if isfield(elec_diag, 'load_reaction')
        current_load = elec_diag.load_reaction;
    else
        current_load = 0.0;
    end
    
    % 6. Flight Computer (FSW) Discrete Execution at 50 Hz
    fsw_timer = fsw_timer + dt;
    if fsw_timer >= dt_fsw - 1e-6
        fsw_timer = 0.0;
        
        % True acceleration for sensor emulation (specific force = (Thrust + Aero) / mass)
        if ~exist('F_aero_b', 'var')
            F_aero_b = [0; 0; 0];
        end
        a_true_b = (F_thrust_b + F_aero_b) / m_curr;
        
        % Generate noisy sensor readings
        [raw_sensors, sensor_state] = sensor_avionics_model(...
            h_curr, a_true_b, omega_b, sensor_state, electronics, sim_params);
        
        % Run FSW state machine and PID controller
        if enable_control
            [fsw_out, fsw_state] = flight_computer_gnc(...
                t, raw_sensors, fsw_state, gnc, rocket, airbrakes, design_type);
            fsw_cmd = fsw_out.u_cmd;
        else
            fsw_cmd = 0.0;
            fsw_out.u_cmd = 0.0;
            fsw_out.h_est = h_curr;
            fsw_out.vz_est = vz_up;
            fsw_out.h_pred = h_curr;
            fsw_out.err = 0.0;
        end
    end
    
    % 7. Actuator Electro-Mechanical Dynamics Integration
    if strcmpi(design_type, 'design1_hinged')
        tau_hinge_in = current_load;
        F_rail_in = 0.0;
    else
        tau_hinge_in = 0.0;
        F_rail_in = current_load;
    end
    
    [act_dot, elec_diag] = actuator_electronics_model(...
        act_state, fsw_cmd, design_type, tau_hinge_in, F_rail_in, electronics, airbrakes, dt);
    
    act_state = act_state + act_dot * dt;
    % Clamp states
    if strcmpi(design_type, 'design1_hinged')
        act_state(1) = max(0.0, min(pi/2, act_state(1)));
        brake_state_physical = act_state(1); % Angle in radians
    else
        act_state(1) = max(0.0, min(airbrakes.design2.max_stroke, act_state(1)));
        brake_state_physical = act_state(1); % Stroke in meters
    end
    
    % 8. Aerodynamics Forces and Moments
    [F_aero_b, M_aero_b, aero_diag] = airbrake_aerodynamics_6dof(...
        V_b, omega_b, V_wind_b, h_curr, x_CG_curr, brake_state_physical, design_type, rocket, airbrakes, sim_params);
    
    % Update actuator load for next iteration
    if strcmpi(design_type, 'design1_hinged')
        elec_diag.load_reaction = aero_diag.tau_hinge_Nm;
    else
        elec_diag.load_reaction = aero_diag.F_rail_fric_N;
    end
    
    % Gravity in Body Frame
    F_grav_ned = [0.0; 0.0; m_curr * sim_params.g0];
    F_grav_b   = C_b_e * F_grav_ned;
    
    % Total Forces & Moments
    F_total_b = F_aero_b + F_thrust_b + F_grav_b;
    M_total_b = M_aero_b;
    
    % 9. Rail Constraints
    if on_rail
        % Rocket is constrained along body X axis
        F_total_b(2) = 0.0;
        F_total_b(3) = 0.0;
        M_total_b = [0.0; 0.0; 0.0];
        V_b(2) = 0.0;
        V_b(3) = 0.0;
        omega_b = [0.0; 0.0; 0.0];
    end
    
    % 10. Translational Dynamics (Body Frame):
    % m * (dV_b/dt + omega_b x V_b) = F_total_b
    V_b_dot = (F_total_b / m_curr) - cross(omega_b, V_b);
    
    % Rotational Dynamics (Body Frame):
    % I * d(omega_b)/dt + omega_b x (I * omega_b) = M_total_b
    omega_b_dot = I_mat \ (M_total_b - cross(omega_b, I_mat * omega_b));
    
    % Attitude Kinematics (Quaternion Rate):
    p = omega_b(1); q = omega_b(2); r = omega_b(3);
    Omega_q = [ 0, -p, -q, -r;
                p,  0,  r, -q;
                q, -r,  0,  p;
                r,  q, -p,  0 ];
    q_dot = 0.5 * Omega_q * q0;
    
    % Position derivative
    r_ned_dot = V_ned;
    
    % 11. State Integration (Euler-Heun / Runge-Kutta 2)
    V_b     = V_b + V_b_dot * dt;
    omega_b = omega_b + omega_b_dot * dt;
    q0      = q0 + q_dot * dt;
    q0      = q0 / norm(q0); % Re-normalize quaternion
    r_ned   = r_ned + r_ned_dot * dt;
    
    % Compute Euler Angles (deg)
    euler_angles_deg = quat_to_euler(q0) * (180 / pi);
    
    % 12. Log Variables
    log_t(k)          = t;
    log_pos_ned(k, :) = r_ned';
    log_vel_ned(k, :) = V_ned';
    log_vel_b(k, :)   = V_b';
    log_omega_b(k, :) = omega_b';
    log_euler_deg(k, :)= euler_angles_deg';
    log_altitude(k)   = h_curr;
    log_mach(k)       = aero_diag.Mach;
    log_q_inf(k)      = aero_diag.q_inf;
    log_alpha_deg(k)  = aero_diag.alpha_deg;
    log_CD_total(k)   = aero_diag.CD_total;
    log_dCD_brakes(k) = aero_diag.dCD_brakes;
    log_thrust(k)     = T_curr;
    log_mass(k)       = m_curr;
    
    log_act_pos(k)    = elec_diag.pos_actual;
    log_act_cmd(k)    = fsw_cmd;
    log_act_load(k)   = elec_diag.load_reaction;
    log_current_A(k)  = elec_diag.i_total;
    log_vbus_V(k)     = elec_diag.v_bus;
    log_power_W(k)    = elec_diag.power_watts;
    
    log_fsw_mode(k)   = fsw_out.flight_mode;
    log_h_est(k)      = fsw_out.h_est;
    log_vz_est(k)     = fsw_out.vz_est;
    log_h_pred(k)     = fsw_out.h_pred;
    log_pid_err(k)    = fsw_out.err;
    log_pid_u(k)      = fsw_out.u_cmd;
    log_pwm_us(k)     = fsw_out.pwm_us;
    
    % 13. Apogee Check
    if t > rocket.t_burn && vz_up <= 0.0 && ~apogee_reached
        apogee_reached = true;
        t_apogee = t;
        h_apogee = h_curr;
        step_final = min(N_steps, k + round(2.0 / dt)); % Continue for 2 s after apogee
    end
    
    if apogee_reached && k >= step_final
        break;
    end
end

fprintf(' Done! Apogee: %.2f m at t = %.2f s\n', h_apogee, t_apogee);

% Trim logs to actual simulated duration
idx = 1:min(k, N_steps);

sim_data = struct();
sim_data.design_type      = design_type;
sim_data.enable_control   = enable_control;
sim_data.target_apogee    = gnc.target_apogee;
sim_data.t_apogee         = t_apogee;
sim_data.h_apogee         = h_apogee;
sim_data.apogee_error     = h_apogee - gnc.target_apogee;
sim_data.t                = log_t(idx);
sim_data.pos_ned          = log_pos_ned(idx, :);
sim_data.vel_ned          = log_vel_ned(idx, :);
sim_data.vel_b            = log_vel_b(idx, :);
sim_data.omega_b          = log_omega_b(idx, :);
sim_data.euler_deg        = log_euler_deg(idx, :);
sim_data.altitude         = log_altitude(idx);
sim_data.mach             = log_mach(idx);
sim_data.q_inf            = log_q_inf(idx);
sim_data.alpha_deg        = log_alpha_deg(idx);
sim_data.CD_total         = log_CD_total(idx);
sim_data.dCD_brakes       = log_dCD_brakes(idx);
sim_data.thrust           = log_thrust(idx);
sim_data.mass             = log_mass(idx);
sim_data.act_pos          = log_act_pos(idx);
sim_data.act_cmd          = log_act_cmd(idx);
sim_data.act_load         = log_act_load(idx);
sim_data.current_A        = log_current_A(idx);
sim_data.vbus_V           = log_vbus_V(idx);
sim_data.power_W          = log_power_W(idx);
sim_data.fsw_mode         = log_fsw_mode(idx);
sim_data.h_est            = log_h_est(idx);
sim_data.vz_est           = log_vz_est(idx);
sim_data.h_pred           = log_h_pred(idx);
sim_data.pid_err          = log_pid_err(idx);
sim_data.pid_u            = log_pid_u(idx);
sim_data.pwm_us           = log_pwm_us(idx);

end

%% Helper Quaternion & DCM Functions
function C = quat_to_dcm(q)
    % Quat scalar first [q0; q1; q2; q3]
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    C = [ q0^2 + q1^2 - q2^2 - q3^2, 2*(q1*q2 + q0*q3),       2*(q1*q3 - q0*q2);
          2*(q1*q2 - q0*q3),       q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 + q0*q1);
          2*(q1*q3 + q0*q2),       2*(q2*q3 - q0*q1),       q0^2 - q1^2 - q2^2 + q3^2 ];
end

function euler = quat_to_euler(q)
    % Returns [roll; pitch; yaw] in radians
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    
    sin_pitch = -2.0 * (q1*q3 - q0*q2);
    sin_pitch = max(-1.0, min(1.0, sin_pitch));
    pitch = asin(sin_pitch);
    
    roll = atan2(2.0 * (q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 + q3^2);
    yaw  = atan2(2.0 * (q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2);
    
    euler = [roll; pitch; yaw];
end
