function active_design = set_airbrake_design(design_id)
% SET_AIRBRAKE_DESIGN Select active airbrake mechanism for 6-DOF Simulink model.
%
% Usage:
%   set_airbrake_design(1)        % Select Design 1 (Downward Hinged Flaps)
%   set_airbrake_design(2)        % Select Design 2 (Radially Outward Sliding Flaps)
%   set_airbrake_design('hinged') % Select Design 1
%   set_airbrake_design('sliding')% Select Design 2
%   current = set_airbrake_design % Query currently active design
%
% Designs:
%   Design 1: Downward Hinged Flaps (0 - 90 deg, Savox SC-1258TG Titanium Servo)
%   Design 2: Radially Outward Sliding Flaps (90 deg, Actuonix L16-R Micro Linear Actuator)

model_name = 'Rocket_Active_Airbrake_6DOF';
blk = [model_name, '/Design_Mode_Select'];

% Ensure model is loaded
if ~bdIsLoaded(model_name)
    fprintf('==> Loading Simulink model "%s"...\n', model_name);
    load_system(model_name);
end

opt1 = 'Design 1: Downward Hinged Flaps (0 - 90 deg, Savox SC-1258TG)';
opt2 = 'Design 2: Radially Outward Sliding Flaps (90 deg, Actuonix L16-R)';

if nargin == 0
    % Query mode
    try
        curr_val = get_param(blk, 'design_popup');
        fprintf('------------------------------------------------------------\n');
        fprintf('CURRENT SIMULINK ACTIVE AIRBRAKE DESIGN:\n');
        fprintf('  %s\n', curr_val);
        fprintf('------------------------------------------------------------\n');
        if nargout > 0
            if contains(curr_val, 'Design 1')
                active_design = 1;
            else
                active_design = 2;
            end
        end
        return;
    catch
        curr_val = get_param(blk, 'Value');
        fprintf('Current block value: %s\n', curr_val);
        return;
    end
end

% Parse input
if ischar(design_id) || isstring(design_id)
    str = lower(char(design_id));
    if contains(str, '1') || contains(str, 'hing') || contains(str, 'down') || contains(str, 'savox')
        target_mode = 1;
    elseif contains(str, '2') || contains(str, 'slid') || contains(str, 'rad') || contains(str, 'actuonix')
        target_mode = 2;
    else
        error('Unrecognized design identifier "%s". Use 1 (Hinged) or 2 (Sliding).', design_id);
    end
elseif isnumeric(design_id)
    if design_id == 1
        target_mode = 1;
    elseif design_id == 2
        target_mode = 2;
    else
        error('Invalid design number %d. Valid options are 1 or 2.', design_id);
    end
else
    error('Input must be a number (1 or 2) or a string.');
end

% Apply to mask
if target_mode == 1
    selected_str = opt1;
else
    selected_str = opt2;
end

set_param(blk, 'design_popup', selected_str);

% Save system
save_system(model_name);

fprintf('============================================================\n');
fprintf(' [OK] SIMULINK ACTIVE AIRBRAKE UPDATED TO DESIGN %d\n', target_mode);
fprintf('============================================================\n');
if target_mode == 1
    fprintf('  * Mechanism:     Downward Hinged Flaps (0 to 90 deg normal)\n');
    fprintf('  * Actuator:      Savox SC-1258TG Coreless Titanium-Gear Servo\n');
    fprintf('  * Max Speed:     750 deg/s (~0.12 s to full 90 deg deployment)\n');
    fprintf('  * Dynamic Load:  Aerodynamic hinge torque counter-force (up to 3.84 N*m)\n');
    fprintf('  * Peak Power:    ~24.8 W (2.93 A @ 8.27 V)\n');
else
    fprintf('  * Mechanism:     Radially Outward Sliding Flaps (90 deg constant)\n');
    fprintf('  * Actuator:      Actuonix L16-R Micro Linear Actuator (35 mm stroke)\n');
    fprintf('  * Max Speed:     32 mm/s (~1.1 s to full 35 mm deployment)\n');
    fprintf('  * Dynamic Load:  Guide-rail normal friction drag (up to 32.7 N)\n');
    fprintf('  * Peak Power:    ~9.2 W (1.10 A @ 8.35 V)\n');
end
fprintf('------------------------------------------------------------\n');
fprintf('You can also double-click the blue "AIRBRAKE SELECTOR" block\n');
fprintf('in the Simulink canvas to change this via interactive dialog.\n');
fprintf('============================================================\n');

if nargout > 0
    active_design = target_mode;
end
end
