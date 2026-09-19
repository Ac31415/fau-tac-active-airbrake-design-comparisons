function [F_aero_b, M_aero_b, aero_diag] = airbrake_aerodynamics_6dof(V_b, omega_b, V_wind_b, h, x_CG, brake_state, design_type, rocket, airbrakes, sim_params)
% AIRBRAKE_AERODYNAMICS_6DOF Computes full 3D aerodynamic forces, moments,
% and actuator mechanical loading (hinge torque / rail friction).
%
% Inputs:
%   V_b         - Body velocity vector [u; v; w] (m/s)
%   omega_b     - Body angular rates [p; q; r] (rad/s)
%   V_wind_b    - Ambient wind vector in body frame [u_w; v_w; w_w] (m/s)
%   h           - Altitude MSL (m)
%   x_CG        - Current CG location from nose tip (m)
%   brake_state - Flap deployment state:
%                   Design 1: flap angle theta (rad) in [0, pi/2]
%                   Design 2: stroke extension s (m) in [0, max_stroke]
%   design_type - 'design1_hinged' or 'design2_sliding'
%   rocket      - Rocket parameters structure
%   airbrakes   - Airbrake configuration structure
%   sim_params  - Simulation environment structure
%
% Outputs:
%   F_aero_b    - Aerodynamic forces in body frame [Fx; Fy; Fz] (N)
%   M_aero_b    - Aerodynamic moments in body frame [Mx; My; Mz] (N*m)
%   aero_diag   - Diagnostic struct containing CD, dynamic pressure,
%                 actuator torque / rail friction forces, angles of attack.

% 1. Atmospheric conditions
[T_air, a_sound, P_air, rho_air] = atmosisa(max(0, h));

% 2. Relative airspeed in body frame
V_rel_b = V_b - V_wind_b;
u_rel = V_rel_b(1);
v_rel = V_rel_b(2);
w_rel = V_rel_b(3);

V_inf = norm(V_rel_b);
V_safe = max(V_inf, 0.1); % Prevent division by zero
q_inf = 0.5 * rho_air * (V_inf^2);
Mach = V_inf / a_sound;

% Aerodynamic angles
alpha = atan2(w_rel, max(u_rel, 0.1)); % Angle of attack (rad)
beta  = asin(max(-1.0, min(1.0, v_rel / V_safe))); % Sideslip angle (rad)

% Angular rates
p = omega_b(1);
q = omega_b(2);
r = omega_b(3);

A_ref = rocket.A_ref;
L_ref = rocket.L_ref;

% 3. Clean Rocket Aerodynamic Coefficients
CD0 = rocket.CD0_clean;
% Compressibility drag rise (Prandtl-Glauert subsonic / transonic correction)
if Mach < 0.8
    CD_comp = CD0 / sqrt(max(0.1, 1 - Mach^2));
elseif Mach <= 1.1
    CD_comp = CD0 * (1.0 + 2.2 * ((Mach - 0.8) / 0.3)^2);
else
    CD_comp = CD0 * (1.6 / sqrt(max(0.1, Mach^2 - 1)));
end

% 4. Active Airbrake Aerodynamics & Mechanical Reaction Loads
dCD_brakes = 0.0;
tau_hinge_total = 0.0;
F_rail_friction_total = 0.0;
F_normal_per_flap = 0.0;

switch lower(design_type)
    case 'design1_hinged'
        % brake_state is flap opening angle theta (rad) in [0, pi/2]
        theta_rad = max(0.0, min(pi/2, brake_state));
        theta_deg = theta_rad * (180 / pi);
        
        % Projected area: N * W * L * sin(theta)
        A_proj = airbrakes.design1.total_area * sin(theta_rad);
        
        % Flap drag coefficient: peaks when normal to flow (sin^2 variation)
        CD_flap = airbrakes.design1.CD_flap_90 * (sin(theta_rad)^2);
        dCD_brakes = CD_flap * (A_proj / A_ref);
        
        % Mechanical hinge torque model:
        % Aerodynamic normal force on each flap
        A_single_flap = airbrakes.design1.flap_width * airbrakes.design1.flap_length;
        F_normal_per_flap = q_inf * airbrakes.design1.CD_flap_90 * sin(theta_rad) * A_single_flap;
        
        % Hinge torque on each flap about hinge pivot
        r_cp = airbrakes.design1.r_cp_flap;
        tau_hinge_single = F_normal_per_flap * r_cp * sin(theta_rad);
        
        % Total torque required by central servo pushrod mechanism
        tau_hinge_total = (airbrakes.num_flaps * tau_hinge_single) / airbrakes.design1.link_ratio;
        
    case 'design2_sliding'
        % Design 2: Radially outward sliding flaps
        % Only the max extended area of each sliding flap is needed!
        if isfield(airbrakes.design2, 'single_flap_max_area')
            A_single_max = airbrakes.design2.single_flap_max_area;
        elseif isfield(airbrakes.design2, 'single_area')
            A_single_max = airbrakes.design2.single_area;
        elseif isfield(airbrakes, 'single_area')
            A_single_max = airbrakes.single_area;
        elseif isfield(airbrakes.design2, 'total_area')
            A_single_max = airbrakes.design2.total_area / airbrakes.num_flaps;
        elseif isfield(airbrakes.design2, 'flap_width') && isfield(airbrakes.design2, 'max_stroke')
            A_single_max = airbrakes.design2.flap_width * airbrakes.design2.max_stroke;
        else
            A_single_max = 0.00064516; % Default 1.0 in^2 = 0.00064516 m^2
        end
        
        % Normalize deployment fraction u_frac in [0, 1]
        if brake_state > 1.0
            u_frac = min(1.0, brake_state / 100.0);
        elseif isfield(airbrakes.design2, 'max_stroke') && airbrakes.design2.max_stroke > 0 && brake_state > 1.0
            u_frac = max(0.0, min(1.0, brake_state / airbrakes.design2.max_stroke));
        else
            u_frac = max(0.0, min(1.0, brake_state));
        end
        
        % Projected area of each flap and all flaps combined
        A_single_extended = A_single_max * u_frac;
        A_proj = airbrakes.num_flaps * A_single_extended;
        
        % Flat plate normal to flow (CD = 1.28)
        CD_plate = airbrakes.design2.CD_plate;
        dCD_brakes = CD_plate * (A_proj / A_ref);
        
        % Mechanical guide-rail friction model:
        % Aerodynamic drag exerts a normal force against guide rails
        F_normal_per_flap = q_inf * CD_plate * A_single_extended;
        
        % Friction on guide rails opposing actuator linear motion
        F_fric_single = airbrakes.design2.mu_rail * F_normal_per_flap + airbrakes.design2.rail_preload;
        F_rail_friction_total = airbrakes.num_flaps * F_fric_single;
end

% Total Drag Coefficient
CD_total = CD_comp + dCD_brakes;

% 5. Aerodynamic Forces in Body Coordinates [Fx; Fy; Fz]
% Rocket convention: X along centerline out the nose, Z down, Y out the right side
% Axial force (negative along body X)
Fx_aero = -q_inf * A_ref * (CD_total * cos(alpha) * cos(beta));

% Side force (body Y)
Fy_aero = q_inf * A_ref * (rocket.CYb * beta);

% Normal force (body Z, negative upward)
Fz_aero = -q_inf * A_ref * (rocket.CNa * alpha);

F_aero_b = [Fx_aero; Fy_aero; Fz_aero];

% 6. Aerodynamic Moments in Body Coordinates [Mx; My; Mz]
% Rolling moment (roll damping)
Mx_aero = q_inf * A_ref * L_ref * (rocket.Clp * (p * L_ref / (2 * V_safe)));

% Static stability margin in calibers: positive when CP is aft of CG
static_margin_calibers = (rocket.x_CP - x_CG) / L_ref;

% Pitching moment: restoring moment (nose-down when alpha > 0) + pitch damping + airbrake offset
Cm_alpha = -rocket.CNa * static_margin_calibers; % Negative for pitch stability

% Airbrake axial force produces a moment if angle of attack is non-zero
arm_brakes = (airbrakes.x_mount - x_CG); % Positive if airbrakes are aft of CG
dCm_brakes = -(dCD_brakes * A_ref / (A_ref * L_ref)) * arm_brakes * sin(alpha);

My_aero = q_inf * A_ref * L_ref * (Cm_alpha * alpha + ...
    rocket.Cmq * (q * L_ref / (2 * V_safe)) + dCm_brakes);

% Yawing moment: restoring weathercocking moment (turns nose into wind, right when beta > 0) + yaw damping
Cn_beta = +rocket.CNa * static_margin_calibers; % Positive for yaw weathercocking stability

Mz_aero = q_inf * A_ref * L_ref * (Cn_beta * beta + ...
    rocket.Cnr * (r * L_ref / (2 * V_safe)));

M_aero_b = [Mx_aero; My_aero; Mz_aero];

% 7. Diagnostic Package
aero_diag = struct();
aero_diag.V_inf         = V_inf;
aero_diag.Mach          = Mach;
aero_diag.q_inf         = q_inf;
aero_diag.rho           = rho_air;
aero_diag.alpha_deg     = alpha * (180 / pi);
aero_diag.beta_deg      = beta * (180 / pi);
aero_diag.CD_clean      = CD_comp;
aero_diag.dCD_brakes    = dCD_brakes;
aero_diag.CD_total      = CD_total;
aero_diag.tau_hinge_Nm  = tau_hinge_total;
aero_diag.F_rail_fric_N = F_rail_friction_total;
aero_diag.F_norm_flap_N = F_normal_per_flap;

end
