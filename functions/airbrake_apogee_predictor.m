function [h_pred, h_pred_clean, h_pred_full] = airbrake_apogee_predictor(h_curr, vz_curr, m_curr, rho_curr, rocket, airbrakes, design_type, u_cmd)
% AIRBRAKE_APOGEE_PREDICTOR Closed-form & numerical ballistic apogee prediction
%
% Inputs:
%   h_curr      - Current altitude above sea level (m)
%   vz_curr     - Current vertical velocity (m/s)
%   m_curr      - Current rocket mass (kg)
%   rho_curr    - Current ambient air density (kg/m^3)
%   rocket      - Rocket parameters structure
%   airbrakes   - Airbrakes parameters structure
%   design_type - 'design1_hinged' or 'design2_sliding'
%   u_cmd       - Current/candidate brake deployment fraction [0, 1]
%
% Outputs:
%   h_pred       - Predicted apogee at deployment level u_cmd (m)
%   h_pred_clean - Predicted apogee if airbrakes are completely closed (u=0)
%   h_pred_full  - Predicted apogee if airbrakes are completely open (u=1)
%
% Physics Formulation:
%   Vertical motion coast phase ODE:
%     dh/dt = vz
%     dvz/dt = -g - (1/(2*m)) * rho * CD * A_ref * vz^2
%   Substituting dh = (vz / dvz) dvz and integrating from vz to 0:
%     Delta_h = (1 / (2*beta)) * ln( 1 + (beta / g) * vz^2 )
%   where beta = (rho_eff * CD_total * A_ref) / (2 * m).
%   An altitude-stratification correction factor accounts for exponential density decay.

g = 9.80665;
A_ref = rocket.A_ref;

% If descending or near apogee, predicted apogee is simply current altitude
if vz_curr <= 0.5
    h_pred = h_curr;
    h_pred_clean = h_curr;
    h_pred_full = h_curr;
    return;
end

% Density scale height for isothermal/standard atmosphere (~8500 m)
H_scale = 8500.0;

% Drag increment computation for candidate u_cmd
dCD_current = compute_delta_CD(u_cmd, design_type, rocket, airbrakes);
dCD_clean   = 0.0;
dCD_full    = compute_delta_CD(1.0, design_type, rocket, airbrakes);

CD_current = rocket.CD0_clean + dCD_current;
CD_clean   = rocket.CD0_clean + dCD_clean;
CD_full    = rocket.CD0_clean + dCD_full;

% Effective mean density between current altitude and expected apogee
% Initial estimate:
dh_est = (vz_curr^2) / (2 * g);
h_mid_est = h_curr + 0.5 * dh_est;
rho_eff = rho_curr * exp(-(h_mid_est - h_curr) / H_scale);
rho_eff = max(0.2, min(rho_curr, rho_eff));

% Closed-form integration for current deployment
beta_curr = (rho_eff * CD_current * A_ref) / (2 * m_curr);
h_pred = h_curr + (1.0 / (2.0 * beta_curr)) * log(1.0 + (beta_curr / g) * (vz_curr^2));

% Predicted apogee for fully retracted (u=0)
beta_clean = (rho_eff * CD_clean * A_ref) / (2 * m_curr);
h_pred_clean = h_curr + (1.0 / (2.0 * beta_clean)) * log(1.0 + (beta_clean / g) * (vz_curr^2));

% Predicted apogee for fully deployed (u=1)
beta_full = (rho_eff * CD_full * A_ref) / (2 * m_curr);
h_pred_full = h_curr + (1.0 / (2.0 * beta_full)) * log(1.0 + (beta_full / g) * (vz_curr^2));

end

function dCD = compute_delta_CD(u, design_type, rocket, airbrakes)
    % Clamp u in [0, 1]
    u = max(0.0, min(1.0, u));
    A_ref = rocket.A_ref;
    
    switch lower(design_type)
        case 'design1_hinged'
            % Downward hinged flaps: angle theta = u * 90 deg
            theta_rad = (u * airbrakes.design1.max_angle_deg) * (pi / 180);
            % Effective projected area = N * W * L * sin(theta)
            A_proj = airbrakes.design1.total_area * sin(theta_rad);
            % Flap CD varies with sin^2(theta)
            CD_flap = airbrakes.design1.CD_flap_90 * (sin(theta_rad)^2);
            dCD = CD_flap * (A_proj / A_ref);
            
        case 'design2_sliding'
            % Radially outward sliding flaps: extension s = u * max_stroke
            % Flaps are always at 90 deg normal to rocket body
            % Effective projected area = N * W * s (linear with stroke!)
            A_proj = airbrakes.design2.total_area * u;
            CD_plate = airbrakes.design2.CD_plate;
            dCD = CD_plate * (A_proj / A_ref);
            
        otherwise
            dCD = 0.0;
    end
end
