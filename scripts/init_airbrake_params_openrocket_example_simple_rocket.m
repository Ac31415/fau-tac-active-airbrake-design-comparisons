%% init_airbrake_params.m
% Master parameter definition for 3D 6-DOF Active Airbrake Flight Simulation
% Compares Hinged Flaps (Design 1) vs Radial Sliding Flaps (Design 2)
% Models actual flight electronics: Teensy 4.1 MCU, BMP388 Barometer,
% BMI088 6-DOF IMU, Savox SC-1258TG Servo, Actuonix L16 Linear Actuator,
% and 2S LiPo power bus.
%
% Copyright (c) 2026 Aerospace Systems Laboratory

fprintf('==> Initializing 6-DOF Active Airbrake Simulation Parameters...\n');

%% 1. Simulation & Environment Settings
sim_params = struct();
sim_params.t_start    = 0.0;     % Start time (s)
sim_params.t_max      = 1200.0;    % Max simulation time (s)
sim_params.dt         = 0.002;   % Continuous plant integration step (s) - 500 Hz
sim_params.dt_fsw     = 0.020;   % Flight software sample time (s) - 50 Hz
sim_params.g0         = 9.80665; % Standard gravity at sea level (m/s^2)
sim_params.R_earth    = 6371000; % Mean Earth radius (m)

% Launch Rail / Pad Configuration
sim_params.rail_length = 3.65;    % Launch rail length (m) ~ 12 ft
sim_params.rail_angle  = 86.0;    % Launch elevation angle (deg) from horizontal
sim_params.rail_azimuth = 90.0;   % Launch azimuth (deg, 90 = East)
sim_params.altitude_pad = 100.0;  % Launch site elevation MSL (m)

% Atmospheric & Wind Settings
sim_params.T0_sl       = 288.15;  % Sea level temperature (K)
sim_params.P0_sl       = 101325;  % Sea level pressure (Pa)
sim_params.rho0_sl     = 1.225;   % Sea level air density (kg/m^3)
sim_params.gamma_air   = 1.4;     % Ratio of specific heats
sim_params.R_gas       = 287.05;  % Specific gas constant (J/kg*K)
sim_params.L_lapse     = 0.0065;  % Temperature lapse rate (K/m)

% Ambient Wind Field (3D: North, East, Down)
sim_params.wind_base_speed = 2.0; % Ground wind speed (m/s)
sim_params.wind_azimuth    = 90; % Direction wind is coming FROM (deg, 270 = from West to East)
% check the params below later for wind field model
sim_params.wind_shear_exp  = 0.14;% Power-law boundary layer shear exponent
sim_params.gust_amplitude  = 2.5; % Gust magnitude (m/s)
sim_params.gust_frequency  = 0.3; % Gust frequency (Hz)

%% 2. Rocket Airframe Physical Parameters
rocket = struct();
rocket.name         = 'AeroStrike-100 Active Control Sounding Rocket';
rocket.diameter     = 0.025;               % Body diameter (m) ~ 4.0 inches
rocket.length       = 0.425;                % Total rocket length (m)
rocket.A_ref        = (pi/4)*rocket.diameter^2; % Reference cross-sectional area (m^2)
rocket.L_ref        = rocket.diameter;     % Reference length for moment coeffs (m)

% Masses & Mass Depletion
rocket.m_dry        = 0.0482;                % Dry mass without motor propellant (kg)
rocket.m_prop       = 0.0231;                % Propellant mass (kg)
rocket.m_liftoff    = rocket.m_dry + rocket.m_prop; % Total liftoff mass (kg) = 0.0713 kg

% Center of Gravity & Center of Pressure (measured from nose tip)
rocket.x_CG_wet     = 0.248;                % Initial CG from nose tip (m)
rocket.x_CG_dry     = 0.241;                % Final dry CG from nose tip (m)
rocket.x_CP         = 0.30;                % Center of pressure from nose tip (m)
rocket.static_margin_wet = (rocket.x_CP - rocket.x_CG_wet) / rocket.diameter; % ~2.94 calibers
rocket.static_margin_dry = (rocket.x_CP - rocket.x_CG_dry) / rocket.diameter; % ~4.51 calibers

% Moment coefficients
rocket.cxx_wet      = 0;                   % Roll moment coeff wet
rocket.cxx_dry      = 0;                   % Roll moment coeff dry
rocket.cyy_wet      = 3.34e-4;             % Pitch moment coeff wet
rocket.cyy_dry      = 1.08e-4;             % Pitch moment coeff dry
rocket.czz_wet      = -2.38e-4;            % Yaw moment coeff wet
rocket.czz_dry      = 3.66e-4;             % Yaw moment coeff dry

% Moments of Inertia [kg*m^2]
rocket.Ixx_wet      = rocket.cxx_wet * rocket.m_liftoff * (rocket.diameter / 2)^2;           % Roll inertia wet
rocket.Ixx_dry      = rocket.cxx_dry * rocket.m_dry * (rocket.diameter / 2)^2;               % Roll inertia dry
rocket.Iyy_wet      = rocket.cyy_wet * rocket.m_liftoff * (rocket.diameter / 2)^2;           % Pitch inertia wet
rocket.Iyy_dry      = rocket.cyy_dry * rocket.m_dry * (rocket.diameter / 2)^2;               % Pitch inertia dry
rocket.Izz_wet      = rocket.czz_wet * rocket.m_liftoff * (rocket.diameter / 2)^2;           % Yaw inertia wet
rocket.Izz_dry      = rocket.czz_dry * rocket.m_dry * (rocket.diameter / 2)^2;               % Yaw inertia dry

% Propulsion System: High-Power Solid Rocket Motor (e.g. Cesaroni / Aerotech L-Class)
rocket.motor_name   = 'CTI L1050 / Aerotech L1150 High-Impulse';
rocket.t_burn       = 1.83;                % Motor burn duration (s)
rocket.thrust_peak  = 14.1;              % Peak thrust (N)
rocket.thrust_avg   = 4.81;              % Average thrust (N)
rocket.total_impulse= 8.82;              % Total impulse (N*s)

% Realistic 2-stage thrust profile (ignition spike -> plateau -> tail-off)
t_prof = [0.0, 0.15, 0.40, 2.70, 3.05, 3.20];
T_prof = [0.0, 1680.0, 1380.0, 1260.0, 480.0, 0.0];
rocket.thrust_lut_t = t_prof;
rocket.thrust_lut_T = T_prof;

% Clean Rocket Aerodynamic Coefficients
rocket.CD0_clean    = 0.38;                % Zero-lift subsonic drag coefficient
rocket.CNa          = 5.80;                % Normal force coefficient slope (1/rad)
rocket.CYb          = -5.80;               % Side force coefficient slope (1/rad)
rocket.Cma          = -14.2;               % Pitching moment coefficient slope (1/rad)
rocket.Cnb          = 14.2;                % Yawing moment coefficient slope (1/rad)
rocket.Clp          = -0.75;               % Roll damping coefficient (1/rad)
rocket.Cmq          = -26.0;               % Pitch damping coefficient (1/rad)
rocket.Cnr          = -26.0;               % Yaw damping coefficient (1/rad)

%% 3. Airbrake Mechanism Designs (Comparison Targets)
% Both designs are positioned near the CG/midbody (x_brakes = 1.30 m from nose tip)
airbrakes = struct();
airbrakes.x_mount    = 1.30;               % Axial location of airbrake unit (m)
airbrakes.num_flaps  = 4;                  % 4 flaps arranged symmetrically at 90-deg intervals

% --- DESIGN 1: Downward-Opening Pivoting Flaps (Variable Angle 0 -> 90 deg) ---
airbrakes.design1 = struct();
airbrakes.design1.name        = 'Design 1: Downward Pivoting Flaps (0-90 deg)';
airbrakes.design1.flap_width  = 0.038;     % Flap width per flap (m) = 38 mm
airbrakes.design1.flap_length = 0.060;     % Flap length per flap (m) = 60 mm
airbrakes.design1.max_angle_deg = 90.0;    % Max deployment angle (deg)
airbrakes.design1.total_area  = airbrakes.num_flaps * ...
    airbrakes.design1.flap_width * airbrakes.design1.flap_length; % 0.00912 m^2 (~112% of rocket A_ref)
airbrakes.design1.CD_flap_90  = 1.25;      % Flat plate drag coefficient normal to flow
airbrakes.design1.r_cp_flap   = 0.028;     % Distance from hinge to flap CP (m)
airbrakes.design1.link_ratio  = 1.25;      % Mechanical advantage of servo pushrod linkage

% --- DESIGN 2: Radially Outward Sliding Flaps (Variable Sliding Stroke & Speed) ---
airbrakes.design2 = struct();
airbrakes.design2.name        = 'Design 2: Radially Outward Sliding Flaps (90 deg constant)';
airbrakes.design2.flap_width  = 0.038;     % Width along circumference (m) = 38 mm
airbrakes.design2.max_stroke  = 0.035;     % Max radial extension stroke (m) = 35 mm
airbrakes.design2.total_area  = airbrakes.num_flaps * ...
    airbrakes.design2.flap_width * airbrakes.design2.max_stroke; % 0.00532 m^2 (~65% of rocket A_ref)
airbrakes.design2.CD_plate    = 1.28;      % Drag coefficient of flat plate at 90 deg
airbrakes.design2.mu_rail     = 0.20;      % Friction coefficient of guide rails under aero normal load
airbrakes.design2.rail_preload= 2.0;       % Internal seal/rail spring preload per flap (N)

%% 4. Actual Existing Electronics & Actuator Hardware Specifications

% --- Flight Computer: PJRC Teensy 4.1 ---
electronics = struct();
electronics.fcu = struct();
electronics.fcu.name          = 'PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz)';
electronics.fcu.loop_freq_hz  = 50.0;      % Main FSW rate (50 Hz)
electronics.fcu.dt            = 1.0 / electronics.fcu.loop_freq_hz; % 0.02 s
electronics.fcu.proc_delay    = 0.005;     % Sensor read to PWM latency (5 ms)
electronics.fcu.pwm_freq_hz   = 200.0;     % High-speed servo PWM frequency (200 Hz)
electronics.fcu.pwm_min_us    = 1000.0;    % 0% deployment pulse width (us)
electronics.fcu.pwm_max_us    = 2000.0;    % 100% deployment pulse width (us)
electronics.fcu.pwm_res_bits  = 16;        % 16-bit PWM timer resolution

% --- Sensor 1: Bosch BMP388 Precision Barometer ---
electronics.baro = struct();
electronics.baro.name         = 'Bosch BMP388 High-Precision Barometric Altimeter';
electronics.baro.rate_hz      = 50.0;      % Baro output data rate (ODR)
electronics.baro.noise_rms_m  = 0.12;      % Altitude RMS noise (m) in normal mode
electronics.baro.bias_drift_m = 0.50;      % Altitude slow thermal bias drift (m)
electronics.baro.quant_pa     = 0.16;      % Pressure resolution (Pa)

% --- Sensor 2: Bosch BMI088 High-Performance 6-DOF IMU ---
electronics.imu = struct();
electronics.imu.name          = 'Bosch BMI088 Low-Drift 6-DOF IMU';
electronics.imu.rate_hz       = 100.0;     % IMU internal ODR (Hz)
electronics.imu.acc_range_g   = 24.0;      % Accelerometer range (+/- 24g for rocketry)
electronics.imu.acc_noise_density = 160e-6 * 9.81; % Accel noise (m/s^2 / sqrt(Hz))
electronics.imu.acc_bias_init = [0.08, -0.05, 0.12]; % Initial bias [x,y,z] (m/s^2)
electronics.imu.acc_drift_rw  = 1e-4;      % Accel bias random walk (m/s^2 / s)
electronics.imu.gyro_range_dps= 2000.0;    % Gyro range (+/- 2000 deg/s)
electronics.imu.gyro_noise_density = 0.014 * (pi/180); % Gyro noise (rad/s / sqrt(Hz))
electronics.imu.gyro_bias_init= [0.2, -0.15, 0.3] * (pi/180); % Initial bias (rad/s)
electronics.imu.gyro_drift_rw = 5e-5;      % Gyro bias random walk (rad/s / s)

% --- Actuator for Design 1: Savox SC-1258TG High-Torque Digital Titanium-Gear Servo ---
electronics.servo = struct();
electronics.servo.name        = 'Savox SC-1258TG Digital Coreless Titanium-Gear Servo';
electronics.servo.v_nom       = 7.4;       % Nominal supply voltage (V) - 2S LiPo
electronics.servo.stall_torque= 1.47;      % Stall torque (N*m) = 15.0 kg*cm @ 7.4V
electronics.servo.no_load_speed = 750.0;   % Max no-load angular speed (deg/s) = 0.08s / 60 deg
electronics.servo.wn          = 52.0;      % Natural frequency (rad/s) ~ 8.3 Hz
electronics.servo.zeta        = 0.72;      % Damping ratio
electronics.servo.deadband_deg= 0.25;      % Mechanical & electronic deadband (deg)
electronics.servo.i_idle      = 0.09;      % Idle current (A)
electronics.servo.i_run_base  = 0.85;      % Base operating current (A)
electronics.servo.i_stall     = 3.60;      % Stall current (A)
electronics.servo.gear_ratio  = 280.0;     % Internal gear reduction

% --- Actuator for Design 2: Actuonix L16-R Micro Linear Actuator (or Leadscrew Stepper) ---
electronics.linear_actuator = struct();
electronics.linear_actuator.name         = 'Actuonix L16-R Micro Linear Actuator (High-Speed Option)';
electronics.linear_actuator.v_nom        = 7.4;      % Nominal voltage (V)
electronics.linear_actuator.stroke_m     = 0.035;    % Stroke length (m) = 35 mm
electronics.linear_actuator.max_speed_nominal = 0.032; % Max unloaded speed (m/s) = 32 mm/s
electronics.linear_actuator.max_speed_fast    = 0.045; % Optional high-speed gear (m/s) = 45 mm/s
electronics.linear_actuator.max_speed_slow    = 0.016; % Optional high-force gear (m/s) = 16 mm/s
electronics.linear_actuator.stall_force  = 50.0;     % Peak stall thrust force (N) ~ 5.1 kgf
electronics.linear_actuator.tau          = 0.030;    % Time constant for motor acceleration (s)
electronics.linear_actuator.i_idle       = 0.04;     % Idle current (A)
electronics.linear_actuator.i_run        = 0.42;     % Running current (A)
electronics.linear_actuator.i_stall      = 1.10;     % Stall current (A)

% --- Power Distribution: 2S 800mAh 25C LiPo Battery ---
electronics.battery = struct();
electronics.battery.name      = 'Tattu 2S 800mAh 45C LiPo';
electronics.battery.v_full    = 8.40;      % Fully charged open-circuit voltage (V)
electronics.battery.v_nom     = 7.40;      % Nominal voltage (V)
electronics.battery.capacity_mah = 800;    % Capacity (mAh)
electronics.battery.R_internal= 0.045;     % Internal resistance (Ohms) = 45 mOhm

%% 5. Guidance, Navigation, and Control (GNC) / PID Controller Parameters
gnc = struct();
gnc.target_apogee = 3000.0;   % Target apogee MSL (m)

% Flight State Machine Thresholds
gnc.burnout_acc_thresh = -5.0; % Acceleration drop (m/s^2) indicating motor burnout
gnc.burnout_time_min   = 2.80; % Minimum time before burnout detection is armed (s)
gnc.guard_delay_s      = 0.30; % Coast stabilization wait time after burnout (s)
gnc.min_control_vel_z  = 18.0; % Minimum vertical velocity to keep brakes active (m/s)
gnc.apogee_detect_vel  = 2.0;  % Vertical velocity threshold to trigger apogee retraction (m/s)

% Discrete Sensor Fusion (Kalman Filter for Altitude and Vertical Velocity)
gnc.kf_q_acc  = 2.5;          % Process noise variance on vertical acceleration (m/s^2)^2
gnc.kf_r_baro = 0.12^2;       % Measurement noise variance from BMP388 (m^2)

% Apogee Predictor Configuration
gnc.predictor_method = 'analytic_closed_form'; % Fast closed-form with density stratification

% PID Controller Gains for Apogee Regulation
% The control error is defined as: e = h_predicted_at_current_drag - h_target
% When e > 0, the rocket is predicted to overshoot -> command more airbrake drag!
gnc.pid = struct();
gnc.pid.Kp          = 0.0035;  % Proportional gain (1/m)
gnc.pid.Ki          = 0.0012;  % Integral gain (1/(m*s))
gnc.pid.Kd          = 0.0008;  % Derivative gain (s/m)
gnc.pid.N_filter    = 25.0;    % Derivative low-pass filter coefficient
gnc.pid.antiwindup  = 'clamping'; % Anti-windup method
gnc.pid.out_min     = 0.0;     % Min brake command (0 = fully retracted)
gnc.pid.out_max     = 1.0;     % Max brake command (1 = fully deployed)
gnc.pid.slew_rate_lim = 3.0;   % Max deployment rate change per second (1/s)

fprintf('==> Simulation parameters loaded successfully!\n');
fprintf('    Target Apogee: %.1f m (%.0f ft)\n', gnc.target_apogee, gnc.target_apogee*3.28084);
fprintf('    Liftoff Mass: %.2f kg, Dry Mass: %.2f kg\n', rocket.m_liftoff, rocket.m_dry);
fprintf('    Rocket Diameter: %.3f m, Motor Burn: %.1f s\n', rocket.diameter, rocket.t_burn);
