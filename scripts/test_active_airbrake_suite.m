%% test_active_airbrake_suite.m
% Automated test and verification suite for 6-DOF Active Airbrake Simulation.
% Tests parameter loading, standalone 6-DOF engine, apogee targeting accuracy,
% variable sliding speeds, and Simulink model compilation and simulation.
%
% Copyright (c) 2026 Aerospace Systems Laboratory

clear; clc;
fprintf('=========================================================================\n');
fprintf('   RUNNING 6-DOF ACTIVE AIRBRAKE SIMULATION TEST SUITE                  \n');
fprintf('=========================================================================\n\n');

proj_root = fileparts(fileparts(mfilename('fullpath')));
if isempty(proj_root), proj_root = pwd; end
addpath(fullfile(proj_root, 'scripts'));
addpath(fullfile(proj_root, 'functions'));
addpath(fullfile(proj_root, 'models'));

test_results = struct('name', {}, 'passed', {}, 'details', {});

%% Test 1: Parameter Initialization
try
    init_airbrake_params;
    assert(exist('sim_params', 'var') && exist('rocket', 'var') && ...
           exist('airbrakes', 'var') && exist('electronics', 'var') && exist('gnc', 'var'), ...
           'Master structs not found');
    test_results(end+1) = struct('name', 'Parameter Initialization', 'passed', true, ...
        'details', sprintf('Target: %d m, Liftoff Mass: %.2f kg', gnc.target_apogee, rocket.m_liftoff));
    fprintf(' [PASS] Test 1: Parameter Initialization\n');
catch ME
    test_results(end+1) = struct('name', 'Parameter Initialization', 'passed', false, 'details', ME.message);
    fprintf(' [FAIL] Test 1: Parameter Initialization (%s)\n', ME.message);
end

%% Test 2: Apogee Predictor Convergence
try
    % Test closed-form predictor at h = 1500m, vz = 180 m/s
    h_test = 1500; vz_test = 180; m_test = 8.2; rho_test = 1.05;
    [h_pred, h_clean, h_full] = airbrake_apogee_predictor(h_test, vz_test, m_test, rho_test, rocket, airbrakes, 'design1_hinged', 0.5);
    assert(h_clean > h_pred && h_pred > h_full, 'Predictor drag ordering violated');
    test_results(end+1) = struct('name', 'Apogee Predictor', 'passed', true, ...
        'details', sprintf('Clean: %.1f m, Mid: %.1f m, Full: %.1f m', h_clean, h_pred, h_full));
    fprintf(' [PASS] Test 2: Apogee Predictor Consistency\n');
catch ME
    test_results(end+1) = struct('name', 'Apogee Predictor', 'passed', false, 'details', ME.message);
    fprintf(' [FAIL] Test 2: Apogee Predictor Consistency (%s)\n', ME.message);
end

%% Test 3: Standalone 6-DOF Simulator (Design 1 - Hinged Flap)
try
    sim_d1 = rocket_6dof_simulator('design1_hinged', true);
    err_d1 = abs(sim_d1.h_apogee - sim_d1.target_apogee);
    assert(err_d1 < 50.0, sprintf('Apogee error too large: %.1f m', err_d1));
    test_results(end+1) = struct('name', '6-DOF Engine (Design 1)', 'passed', true, ...
        'details', sprintf('Apogee: %.1f m (Error: %+.1f m)', sim_d1.h_apogee, sim_d1.apogee_error));
    fprintf(' [PASS] Test 3: 6-DOF Engine Design 1 (Hinged Flaps)\n');
catch ME
    test_results(end+1) = struct('name', '6-DOF Engine (Design 1)', 'passed', false, 'details', ME.message);
    fprintf(' [FAIL] Test 3: 6-DOF Engine Design 1 (%s)\n', ME.message);
end

%% Test 4: Standalone 6-DOF Simulator (Design 2 - Sliding Flap)
try
    sim_d2 = rocket_6dof_simulator('design2_sliding', true, 0.032);
    err_d2 = abs(sim_d2.h_apogee - sim_d2.target_apogee);
    assert(err_d2 < 50.0, sprintf('Apogee error too large: %.1f m', err_d2));
    test_results(end+1) = struct('name', '6-DOF Engine (Design 2)', 'passed', true, ...
        'details', sprintf('Apogee: %.1f m (Error: %+.1f m)', sim_d2.h_apogee, sim_d2.apogee_error));
    fprintf(' [PASS] Test 4: 6-DOF Engine Design 2 (Sliding Flaps)\n');
catch ME
    test_results(end+1) = struct('name', '6-DOF Engine (Design 2)', 'passed', false, 'details', ME.message);
    fprintf(' [FAIL] Test 4: 6-DOF Engine Design 2 (%s)\n', ME.message);
end

%% Test 5: Simulink Model Simulation (Design 1 & Design 2)
try
    model_name = 'Rocket_Active_Airbrake_6DOF';
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end
    
    % Test Design 1
    set_param([model_name, '/Design_Mode_Select'], 'Value', '1');
    sim_out1 = sim(model_name);
    alt_out1 = sim_out1.yout{1}.Values.Data;
    max_alt1 = max(alt_out1);
    
    % Test Design 2
    set_param([model_name, '/Design_Mode_Select'], 'Value', '2');
    sim_out2 = sim(model_name);
    alt_out2 = sim_out2.yout{1}.Values.Data;
    max_alt2 = max(alt_out2);
    
    % Reset to default Design 1
    set_param([model_name, '/Design_Mode_Select'], 'Value', '1');
    save_system(model_name);
    
    test_results(end+1) = struct('name', 'Simulink Model Execution', 'passed', true, ...
        'details', sprintf('D1 Apogee: %.1f m, D2 Apogee: %.1f m', max_alt1, max_alt2));
    fprintf(' [PASS] Test 5: Simulink Model Execution (Both Designs)\n');
catch ME
    test_results(end+1) = struct('name', 'Simulink Model Execution', 'passed', false, 'details', ME.message);
    fprintf(' [FAIL] Test 5: Simulink Model Execution (%s)\n', ME.message);
end

%% Print Summary
fprintf('\n=========================================================================\n');
fprintf('                           TEST SUITE SUMMARY                            \n');
fprintf('=========================================================================\n');
all_passed = true;
for i = 1:length(test_results)
    tr = test_results(i);
    status = 'PASS';
    if ~tr.passed
        status = 'FAIL';
        all_passed = false;
    end
    fprintf(' [%s] %-28s : %s\n', status, tr.name, tr.details);
end
fprintf('=========================================================================\n');
if all_passed
    fprintf('==> ALL TESTS PASSED! Simulation suite is fully verified and ready.\n\n');
else
    fprintf('==> SOME TESTS FAILED. Review errors above.\n\n');
end
