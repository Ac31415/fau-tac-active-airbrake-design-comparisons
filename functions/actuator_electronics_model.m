function [state_dot, elec_diag] = actuator_electronics_model(state, cmd_frac, design_type, tau_load, F_load, electronics, airbrakes, dt)
% ACTUATOR_ELECTRONICS_MODEL High-fidelity simulation of COTS avionics actuators:
%   Design 1: Savox SC-1258TG Digital High-Torque Titanium-Gear Coreless Servo
%   Design 2: Actuonix L16-R Micro Linear Actuator (with variable speed options)
%
% Inputs:
%   state       - Current actuator continuous state:
%                   Design 1: [theta (rad); theta_dot (rad/s)]
%                   Design 2: [s (m); s_dot (m/s)]
%   cmd_frac    - Normalized flight computer command [0, 1]
%   design_type - 'design1_hinged' or 'design2_sliding'
%   tau_load    - Aerodynamic hinge torque load (N*m) [for Design 1]
%   F_load      - Aerodynamic rail friction load (N) [for Design 2]
%   electronics - Electronics parameter structure
%   airbrakes   - Airbrakes parameter structure
%   dt          - Integration time step (s)
%
% Outputs:
%   state_dot   - Derivative of state vector [d(pos)/dt; d(vel)/dt]
%   elec_diag   - Diagnostic struct (current draw, bus voltage sag, power, stall status)

% Clamp command in valid range
cmd_frac = max(0.0, min(1.0, cmd_frac));

% Base electronics power draw (Teensy MCU + sensors ~ 180 mA)
i_avionics_base = 0.18; 

elec_diag = struct();

switch lower(design_type)
    case 'design1_hinged'
        % --- DESIGN 1: SAVOX SC-1258TG DIGITAL SERVO ---
        theta     = state(1); % Angle in radians
        theta_dot = state(2); % Angular rate in rad/s
        
        servo_cfg = electronics.servo;
        theta_max = airbrakes.design1.max_angle_deg * (pi / 180); % pi/2
        theta_target = cmd_frac * theta_max;
        
        % Check deadband
        deadband_rad = servo_cfg.deadband_deg * (pi / 180);
        pos_error = theta_target - theta;
        if abs(pos_error) < deadband_rad
            pos_error = 0.0;
        end
        
        % Slew rate limit (no load speed)
        omega_max = servo_cfg.no_load_speed * (pi / 180); % ~13.09 rad/s
        
        % Torque available at nominal battery bus
        v_bus_est = electronics.battery.v_nom; 
        tau_avail = servo_cfg.stall_torque * (v_bus_est / servo_cfg.v_nom);
        
        % Aerodynamic load opposes opening (positive theta_dot)
        % If opening and aero torque exceeds stall torque -> stall!
        if theta_dot > 0 && tau_load >= tau_avail
            torque_derate = 0.0; % Stalled by aerodynamic drag
            is_stalled = true;
        elseif theta_dot > 0
            torque_derate = max(0.1, 1.0 - (tau_load / tau_avail));
            is_stalled = false;
        else
            % When closing, aero torque actually assists closing!
            torque_derate = 1.0;
            is_stalled = false;
        end
        
        effective_omega_max = omega_max * torque_derate;
        
        % 2nd-order electromechanical servo dynamics
        wn = servo_cfg.wn;
        zeta = servo_cfg.zeta;
        
        theta_ddot = (wn^2) * pos_error - 2 * zeta * wn * theta_dot;
        
        % Velocity saturation / rate limiting
        predicted_theta_dot = theta_dot + theta_ddot * dt;
        if abs(predicted_theta_dot) > effective_omega_max
            predicted_theta_dot = sign(predicted_theta_dot) * effective_omega_max;
            theta_ddot = (predicted_theta_dot - theta_dot) / max(dt, 1e-4);
        end
        
        % Hard stop saturation
        if (theta <= 0 && theta_ddot < 0) || (theta >= theta_max && theta_ddot > 0)
            theta_ddot = 0.0;
            theta_dot = 0.0;
        end
        
        state_dot = [theta_dot; theta_ddot];
        
        % Electrical current calculation
        norm_speed = abs(theta_dot) / omega_max;
        norm_torque = min(1.0, tau_load / tau_avail);
        if is_stalled
            i_actuator = servo_cfg.i_stall;
        else
            i_actuator = servo_cfg.i_idle + servo_cfg.i_run_base * norm_speed + ...
                (servo_cfg.i_stall - servo_cfg.i_run_base) * (norm_torque^1.5);
        end
        
        i_total = i_avionics_base + i_actuator;
        v_bus = electronics.battery.v_full - i_total * electronics.battery.R_internal;
        
        elec_diag.pos_actual     = theta * (180 / pi); % deg
        elec_diag.pos_target     = theta_target * (180 / pi); % deg
        elec_diag.speed_actual   = theta_dot * (180 / pi); % deg/s
        elec_diag.load_reaction  = tau_load; % N*m
        elec_diag.load_capacity  = tau_avail; % N*m
        elec_diag.i_actuator     = i_actuator; % A
        elec_diag.i_total        = i_total; % A
        elec_diag.v_bus          = v_bus; % V
        elec_diag.power_watts    = v_bus * i_total; % W
        elec_diag.is_stalled     = is_stalled;
        
    case 'design2_sliding'
        % --- DESIGN 2: ACTUONIX L16-R LINEAR ACTUATOR ---
        s     = state(1); % Extension stroke in meters
        s_dot = state(2); % Extension velocity in m/s
        
        lin_cfg = electronics.linear_actuator;
        s_max = airbrakes.design2.max_stroke; % 0.035 m
        s_target = cmd_frac * s_max;
        
        pos_error = s_target - s;
        
        % Force-speed derating curve of linear actuator
        v_bus_est = electronics.battery.v_nom;
        F_stall_avail = lin_cfg.stall_force * (v_bus_est / lin_cfg.v_nom);
        
        if s_dot > 0 && F_load >= F_stall_avail
            v_derate = 0.0; % Stalled by guide-rail friction!
            is_stalled = true;
        elseif s_dot > 0
            v_derate = max(0.1, 1.0 - (F_load / F_stall_avail));
            is_stalled = false;
        else
            % Retracting: friction opposes retracting motion too
            v_derate = max(0.2, 1.0 - 0.5 * (F_load / F_stall_avail));
            is_stalled = false;
        end
        
        v_max_effective = lin_cfg.max_speed_nominal * v_derate;
        
        % 1st-order speed response model
        tau = lin_cfg.tau;
        v_cmd = (pos_error / 0.05); % Proportional linear speed command
        v_cmd = max(-v_max_effective, min(v_max_effective, v_cmd));
        
        s_ddot = (v_cmd - s_dot) / tau;
        
        % Hard stop saturation
        if (s <= 0 && s_ddot < 0) || (s >= s_max && s_ddot > 0)
            s_ddot = 0.0;
            s_dot = 0.0;
        end
        
        state_dot = [s_dot; s_ddot];
        
        % Current draw
        norm_v = abs(s_dot) / lin_cfg.max_speed_nominal;
        norm_F = min(1.0, F_load / F_stall_avail);
        if is_stalled
            i_actuator = lin_cfg.i_stall;
        else
            i_actuator = lin_cfg.i_idle + lin_cfg.i_run * norm_v + ...
                (lin_cfg.i_stall - lin_cfg.i_run) * norm_F;
        end
        
        i_total = i_avionics_base + i_actuator;
        v_bus = electronics.battery.v_full - i_total * electronics.battery.R_internal;
        
        elec_diag.pos_actual     = s * 1000.0; % mm
        elec_diag.pos_target     = s_target * 1000.0; % mm
        elec_diag.speed_actual   = s_dot * 1000.0; % mm/s
        elec_diag.load_reaction  = F_load; % N
        elec_diag.load_capacity  = F_stall_avail; % N
        elec_diag.i_actuator     = i_actuator; % A
        elec_diag.i_total        = i_total; % A
        elec_diag.v_bus          = v_bus; % V
        elec_diag.power_watts    = v_bus * i_total; % W
        elec_diag.is_stalled     = is_stalled;
end

end
