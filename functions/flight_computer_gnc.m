function [fsw_out, fsw_state] = flight_computer_gnc(t, raw_sensors, fsw_state, gnc, rocket, airbrakes, design_type)
% FLIGHT_COMPUTER_GNC Emulates Teensy 4.1 flight computer executing at 50 Hz.
% Includes sensor conditioning, 1D vertical Kalman Filter fusion,
% flight phase state machine, dynamic apogee prediction, and discrete PID control.
%
% Inputs:
%   t           - Current time (s)
%   raw_sensors - Sensor measurements struct:
%                   .h_baro  - BMP388 barometric altitude (m)
%                   .acc_b   - BMI088 3-axis accelerometer [ax; ay; az] (m/s^2)
%                   .gyro_b  - BMI088 3-axis gyro [p; q; r] (rad/s)
%   fsw_state   - Persistent flight software state struct (KF, PID, state machine)
%   gnc         - GNC parameter configuration
%   rocket      - Rocket parameters
%   airbrakes   - Airbrake configuration
%   design_type - 'design1_hinged' or 'design2_sliding'
%
% Outputs:
%   fsw_out     - Output commands struct (u_cmd, pwm_us, flight_state, h_pred, etc.)
%   fsw_state   - Updated persistent state struct

dt_fsw = 0.020; % 50 Hz execution period

% Initialize FSW state if first run
if isempty(fsw_state) || ~isfield(fsw_state, 'initialized') || ~fsw_state.initialized
    fsw_state = struct();
    fsw_state.initialized   = true;
    fsw_state.flight_mode   = 0; % 0: IDLE, 1: BOOST, 2: BURNOUT_GUARD, 3: ACTIVE_BRAKE, 4: RETRACT
    fsw_state.t_burnout     = 0.0;
    
    % Kalman Filter state: [altitude (m); vertical velocity (m/s)]
    fsw_state.kf_x          = [raw_sensors.h_baro; 0.0];
    fsw_state.kf_P          = [1.0, 0.0; 0.0, 4.0];
    
    % PID Controller persistent state
    fsw_state.pid_integral  = 0.0;
    fsw_state.pid_prev_err  = 0.0;
    fsw_state.pid_prev_d    = 0.0;
    fsw_state.u_prev        = 0.0;
    fsw_state.h_pred_filtered = raw_sensors.h_baro;
end

% 1. Sensor Fusion: 1D Vertical Kalman Filter
% Accelerometer axial measurement (along rocket body X, assuming mostly vertical during boost/early coast)
ax_meas = raw_sensors.acc_b(1); 
% Net vertical acceleration: a_z_inertial = ax_meas - g
az_inertial = ax_meas - 9.80665;

% State Transition Matrix:
% [h; vz]_{k} = [1, dt; 0, 1] [h; vz]_{k-1} + [0.5*dt^2; dt] * az
A_kf = [1.0, dt_fsw; 0.0, 1.0];
B_kf = [0.5 * dt_fsw^2; dt_fsw];
H_kf = [1.0, 0.0];

% Process noise & measurement noise
Q_kf = [0.25 * dt_fsw^4, 0.5 * dt_fsw^3; 0.5 * dt_fsw^3, dt_fsw^2] * gnc.kf_q_acc;
R_kf = gnc.kf_r_baro;

% Predict step
x_pred = A_kf * fsw_state.kf_x + B_kf * az_inertial;
P_pred = A_kf * fsw_state.kf_P * A_kf' + Q_kf;

% Update step with BMP388 Barometer measurement
y_meas = raw_sensors.h_baro;
innov = y_meas - H_kf * x_pred;
S_kf = H_kf * P_pred * H_kf' + R_kf;
K_kf = (P_pred * H_kf') / S_kf;

fsw_state.kf_x = x_pred + K_kf * innov;
fsw_state.kf_P = (eye(2) - K_kf * H_kf) * P_pred;

h_est  = fsw_state.kf_x(1);
vz_est = fsw_state.kf_x(2);

% Estimate ambient air density at estimated altitude
[~, ~, ~, rho_est] = atmosisa(max(0, h_est));

% 2. Flight Phase State Machine
MODE_IDLE          = 0;
MODE_BOOST         = 1;
MODE_BURNOUT_GUARD = 2;
MODE_ACTIVE_BRAKE  = 3;
MODE_RETRACT       = 4;

switch fsw_state.flight_mode
    case MODE_IDLE
        if t > 0.05 && (ax_meas > 15.0 || vz_est > 10.0)
            fsw_state.flight_mode = MODE_BOOST;
        end
        u_cmd = 0.0;
        
    case MODE_BOOST
        % Motor burnout detection: acceleration drops below threshold after minimum burn time, or timeout backup
        if (t >= gnc.burnout_time_min && ax_meas < gnc.burnout_acc_thresh) || (t >= rocket.t_burn + 0.10)
            fsw_state.flight_mode = MODE_BURNOUT_GUARD;
            fsw_state.t_burnout = t;
        end
        u_cmd = 0.0;
        
    case MODE_BURNOUT_GUARD
        % Guard delay to allow burnout shock and nozzle pressure equilibration to pass
        if (t - fsw_state.t_burnout) >= gnc.guard_delay_s
            fsw_state.flight_mode = MODE_ACTIVE_BRAKE;
        end
        u_cmd = 0.0;
        
    case MODE_ACTIVE_BRAKE
        % Check for apogee approach or low velocity cutoff
        if vz_est <= gnc.min_control_vel_z || vz_est <= gnc.apogee_detect_vel
            fsw_state.flight_mode = MODE_RETRACT;
            u_cmd = 0.0;
        else
            % Active Airbrake Apogee Regulation
            % 1. Compute predicted apogee
            % Predict with clean rocket (u=0) and full brake (u=1)
            [~, h_pred_clean, h_pred_full] = airbrake_apogee_predictor(...
                h_est, vz_est, rocket.m_dry, rho_est, rocket, airbrakes, design_type, fsw_state.u_prev);
            
            % Predict with current airbrake deployment
            [h_pred_raw, ~, ~] = airbrake_apogee_predictor(...
                h_est, vz_est, rocket.m_dry, rho_est, rocket, airbrakes, design_type, fsw_state.u_prev);
            
            % Smooth predicted apogee with low-pass filter
            alpha_pred = 0.25;
            fsw_state.h_pred_filtered = (1 - alpha_pred) * fsw_state.h_pred_filtered + alpha_pred * h_pred_raw;
            h_pred = fsw_state.h_pred_filtered;
            
            % Apogee Target Error: e = h_pred_clean - h_target
            % If even clean rocket will not reach target apogee, keep brakes 0
            if h_pred_clean < gnc.target_apogee
                err = h_pred_clean - gnc.target_apogee; % negative error
            else
                % In controllable window: error relative to target
                err = h_pred - gnc.target_apogee;
            end
            
            % 2. Discrete PID Controller with Anti-Windup
            Kp = gnc.pid.Kp;
            Ki = gnc.pid.Ki;
            Kd = gnc.pid.Kd;
            N_filt = gnc.pid.N_filter;
            
            % Proportional Term
            P_term = Kp * err;
            
            % Derivative Term with First-Order Low-Pass Filter
            % D(z) = Kd * N * (err - prev_err) / (1 + N*dt) + prev_D / (1 + N*dt)
            d_err = (err - fsw_state.pid_prev_err) / dt_fsw;
            D_term = (Kd * N_filt * d_err + fsw_state.pid_prev_d) / (1.0 + N_filt * dt_fsw);
            fsw_state.pid_prev_d = D_term;
            fsw_state.pid_prev_err = err;
            
            % Integral Term with Conditional Clamping Anti-Windup
            tentative_I = fsw_state.pid_integral + Ki * err * dt_fsw;
            tentative_u = P_term + tentative_I + D_term;
            
            % Anti-windup clamping
            if tentative_u >= gnc.pid.out_max
                if err < 0 % only integrate if it reduces saturation
                    fsw_state.pid_integral = tentative_I;
                end
                u_pid = gnc.pid.out_max;
            elseif tentative_u <= gnc.pid.out_min
                if err > 0 % only integrate if it reduces saturation
                    fsw_state.pid_integral = tentative_I;
                end
                u_pid = gnc.pid.out_min;
            else
                fsw_state.pid_integral = tentative_I;
                u_pid = tentative_u;
            end
            
            % 3. Slew Rate Limiter (to avoid mechanical shock / flutter)
            max_du = gnc.pid.slew_rate_lim * dt_fsw;
            du = u_pid - fsw_state.u_prev;
            du_clamped = max(-max_du, min(max_du, du));
            u_cmd = fsw_state.u_prev + du_clamped;
            u_cmd = max(0.0, min(1.0, u_cmd));
            
            fsw_state.u_prev = u_cmd;
        end
        
    case MODE_RETRACT
        % Retracted to 0% to safely deploy drogue / main parachutes
        u_cmd = 0.0;
        fsw_state.u_prev = 0.0;
        
    otherwise
        u_cmd = 0.0;
end

% 3. Generate Hardware PWM Pulse Width (1000 - 2000 us)
pwm_us = 1000.0 + u_cmd * 1000.0;

% Pack Diagnostic and Output Structure
fsw_out = struct();
fsw_out.u_cmd          = u_cmd;
fsw_out.pwm_us         = pwm_us;
fsw_out.flight_mode    = fsw_state.flight_mode;
fsw_out.h_est          = h_est;
fsw_out.vz_est         = vz_est;
fsw_out.h_pred         = fsw_state.h_pred_filtered;
fsw_out.err            = fsw_state.pid_prev_err;
fsw_out.integral       = fsw_state.pid_integral;

end
