function [raw_sensors, sensor_state] = sensor_avionics_model(h_true, a_b_true, omega_b_true, sensor_state, electronics, sim_params)
% SENSOR_AVIONICS_MODEL Emulates Bosch BMP388 Barometer and BMI088 6-DOF IMU
% with sensor noise, in-run bias drift, and quantization.
%
% Inputs:
%   h_true         - True altitude MSL (m)
%   a_b_true       - True body specific acceleration [ax; ay; az] (m/s^2)
%   omega_b_true   - True body angular rates [p; q; r] (rad/s)
%   sensor_state   - Persistent bias and noise state
%   electronics    - Electronics parameter structure
%   sim_params     - Simulation parameters
%
% Outputs:
%   raw_sensors    - Measured sensor signals struct (.h_baro, .acc_b, .gyro_b)
%   sensor_state   - Updated sensor persistent state

dt = sim_params.dt_fsw; % 50 Hz ODR

if isempty(sensor_state) || ~isfield(sensor_state, 'initialized') || ~sensor_state.initialized
    sensor_state = struct();
    sensor_state.initialized = true;
    sensor_state.acc_bias    = electronics.imu.acc_bias_init(:);
    sensor_state.gyro_bias   = electronics.imu.gyro_bias_init(:);
    sensor_state.baro_bias   = 0.20; % 20 cm initial offset
end

% 1. Bosch BMP388 Barometric Altimeter Emulation
% Atmospheric pressure from altitude:
T0 = sim_params.T0_sl;
P0 = sim_params.P0_sl;
L  = sim_params.L_lapse;
R  = sim_params.R_gas;
g  = sim_params.g0;

% Standard barometric formula
P_true = P0 * (1 - (L * max(0, h_true)) / T0) .^ (g / (R * L));

% Add sensor noise and thermal drift
sigma_P = (P_true * g / (R * T0)) * electronics.baro.noise_rms_m;
P_noise = randn() * sigma_P;
P_meas = P_true + P_noise;

% Invert pressure to measured altitude
h_baro_meas = (T0 / L) * (1 - (P_meas / P0) .^ (R * L / g));
% Add slow drift
sensor_state.baro_bias = sensor_state.baro_bias + randn() * 0.001;
h_baro_meas = h_baro_meas + sensor_state.baro_bias;

% 2. Bosch BMI088 6-DOF IMU Emulation
% Random walk on biases
sensor_state.acc_bias  = sensor_state.acc_bias + randn(3, 1) * (electronics.imu.acc_drift_rw * sqrt(dt));
sensor_state.gyro_bias = sensor_state.gyro_bias + randn(3, 1) * (electronics.imu.gyro_drift_rw * sqrt(dt));

% White noise based on noise spectral density
sigma_acc = electronics.imu.acc_noise_density * sqrt(electronics.imu.rate_hz);
acc_noise = randn(3, 1) * sigma_acc;

sigma_gyro = electronics.imu.gyro_noise_density * sqrt(electronics.imu.rate_hz);
gyro_noise = randn(3, 1) * sigma_gyro;

acc_meas = a_b_true(:) + sensor_state.acc_bias + acc_noise;
gyro_meas = omega_b_true(:) + sensor_state.gyro_bias + gyro_noise;

% Accelerometer saturation (+/- 24g)
g_limit = electronics.imu.acc_range_g * 9.80665;
acc_meas = max(-g_limit, min(g_limit, acc_meas));

% Gyroscope saturation (+/- 2000 deg/s)
gyro_limit = electronics.imu.gyro_range_dps * (pi / 180);
gyro_meas = max(-gyro_limit, min(gyro_limit, gyro_meas));

% Pack outputs
raw_sensors = struct();
raw_sensors.h_baro = h_baro_meas;
raw_sensors.P_pa   = P_meas;
raw_sensors.acc_b  = acc_meas;
raw_sensors.gyro_b = gyro_meas;

end
