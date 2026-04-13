%% build_model.m
% =========================================================
% AUTO-BUILDS the Vehicle Physics Simulink Model
% Run this script once inside MATLAB R2024 to generate
% the complete .slx file with all blocks and connections.
%
% Prerequisites:
%   - simulink_block_interface.m must be on the MATLAB path
%   - Simulink license available
%   - CSV data files in the working directory
%
% Usage:
%   >> build_model
% =========================================================

close_system('VehiclePhysics', 0);   % close without saving if already open
mdl = 'VehiclePhysics';
new_system(mdl);
open_system(mdl);

%% =========================================================
%  MODEL SETTINGS
% =========================================================
set_param(mdl, ...
    'SolverType',       'Fixed-step', ...
    'Solver',           'FixedStepDiscrete', ...
    'FixedStep',        '0.1', ...
    'StopTime',         '600', ...
    'SystemTargetFile', 'grt.tlc');

%% =========================================================
%  LAYOUT CONSTANTS  (grid positions for tidy wiring)
% =========================================================
bw = 160;   % block width
bh = 50;    % block height
cx = 80;    % column x-positions
col = @(n) cx + (n-1)*(bw+80);   % column centres
row = @(n) 60 + (n-1)*(bh+55);   % row centres
blk = @(x,y) [x-bw/2, y-bh/2, x+bw/2, y+bh/2];

%% =========================================================
%  HELPER — add a MATLAB Function block with source code
% =========================================================
function addMFBlock(mdl, name, pos, code_str)
    path = [mdl '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', path, ...
              'Position', pos);
    % Inject the function source
    sf = get_param(path, 'SFunctionModules');   %#ok — not used directly
    rt = sfroot();
    ch = rt.find('-isa','Stateflow.EMChart','Path', path);
    if ~isempty(ch)
        ch.Script = code_str;
    end
end

%% =========================================================
%  VEHICLE PARAMETER CONSTANTS  (Model Workspace)
% =========================================================
hws = get_param(mdl, 'modelworkspace');
hws.assignin('drivetrain_efficiency', 0.91);
hws.assignin('tire_coeff',           0.00475);
hws.assignin('g_accel',              9.81);
hws.assignin('prim_red',             1);
hws.assignin('sec_red',              1);
hws.assignin('diff_red',             4.1);
hws.assignin('g1',                   3.78);
hws.assignin('g2',                   2.12);
hws.assignin('g3',                   1.36);
hws.assignin('g4',                   1.03);
hws.assignin('frontal_area',         1.6);
hws.assignin('drag_coeff',           0.30);
hws.assignin('mass',                 1020);
hws.assignin('wheel_radius',         0.298);
hws.assignin('air_density',          1.225);
hws.assignin('time_step',            0.1);
hws.assignin('rpm_shift_up',         2300);
hws.assignin('rpm_shift_down',       1398);
hws.assignin('init_rpm',             4500);
hws.assignin('init_gear',            int8(1));

% Load power curve for lookup table
pc = readtable('70_power_curve - Copy.csv');
hws.assignin('rpm_bp',    pc.rpm);
hws.assignin('torque_bp', pc.torque);

% Pre-compute total track distance
coords   = readCoords_local('sem_apme_2025-track_coordinates.csv');
track_dist_m = totalDistance_local(coords) * 1000;
hws.assignin('track_dist_m', track_dist_m);
fprintf('Track distance: %.1f m\n', track_dist_m);

%% =========================================================
%  BLOCK DEFINITIONS
%  Format: {name, type, position, [extra params]}
% =========================================================

% ---- Column 1: Initial conditions / unit-delay feedback --
add_block('simulink/Signal Routing/Bus Creator', [mdl '/StateBus'], ...
    'Position', blk(col(1), row(3)), 'Inputs', '5');

% RPM (Unit Delay — feeds back from gearChange output)
add_block('simulink/Discrete/Unit Delay', [mdl '/RPM_Delay'], ...
    'Position', blk(col(1), row(1)), ...
    'InitialCondition', 'init_rpm', 'SampleTime', '0.1');

% Gear (Unit Delay)
add_block('simulink/Discrete/Unit Delay', [mdl '/Gear_Delay'], ...
    'Position', blk(col(1), row(2)), ...
    'InitialCondition', 'init_gear', 'SampleTime', '0.1');

% Speed (Unit Delay — feeds back final_speed_ms as init_speed_ms)
add_block('simulink/Discrete/Unit Delay', [mdl '/Speed_Delay'], ...
    'Position', blk(col(1), row(4)), ...
    'InitialCondition', '0', 'SampleTime', '0.1');

% Distance accumulator (Discrete Integrator)
add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/DistIntegrator'], ...
    'Position', blk(col(1), row(5)), ...
    'gainval', '1', 'SampleTime', '0.1', 'InitialCondition', '0');

% ---- Column 2: Torque Lookup Table -----------------------
add_block('simulink/Lookup Tables/1-D Lookup Table', [mdl '/TorqueLookup'], ...
    'Position',      blk(col(2), row(1)), ...
    'BreakpointsForDimension1', 'rpm_bp', ...
    'Table',         'torque_bp', ...
    'InterpMethod',  'Linear', ...
    'ExtrapMethod',  'Clip');

% ---- Column 2: Throttle Calculator ----------------------
addMFBlock(mdl, 'ThrottleCalc', blk(col(2), row(3)), ...
    [ 'function throttle = ThrottleCalc(distance_covered, dist, speed_kmh)' newline ...
      '    if distance_covered < dist' newline ...
      '        if speed_kmh <= 15' newline ...
      '            throttle = 30;' newline ...
      '        elseif speed_kmh >= 35' newline ...
      '            throttle = 0;' newline ...
      '        else' newline ...
      '            throttle = 30;' newline ...
      '        end' newline ...
      '    else' newline ...
      '        throttle = 0;' newline ...
      '    end' newline ...
      'end' ]);

% ---- Column 3: Speed Calc --------------------------------
addMFBlock(mdl, 'SpeedCalc', blk(col(3), row(1)), ...
    [ 'function [init_speed_ms, speed_kmh] = SpeedCalc(rpm, gear, g1, g2, g3, g4, diff_red, wheel_radius)' newline ...
      '    ratios = [g1, g2, g3, g4];' newline ...
      '    combined = ratios(gear) * diff_red;' newline ...
      '    init_speed_ms = (rpm * 2 * pi * wheel_radius) / (60 * combined);' newline ...
      '    speed_kmh = init_speed_ms * 3.6;' newline ...
      'end' ]);

% Constant blocks for vehicle params fed into SpeedCalc
addConst(mdl, 'K_g1',          'g1',          col(2)-40, row(1)-80);
addConst(mdl, 'K_g2',          'g2',          col(2)-40, row(1)-40);
addConst(mdl, 'K_g3',          'g3',          col(2)-40, row(1));
addConst(mdl, 'K_g4',          'g4',          col(2)-40, row(1)+40);
addConst(mdl, 'K_diff_red',    'diff_red',    col(2)-40, row(1)+80);
addConst(mdl, 'K_wheel_r',     'wheel_radius',col(2)-40, row(1)+120);

% ---- Column 3: Drag Effect -------------------------------
addMFBlock(mdl, 'DragEffect', blk(col(3), row(2)), ...
    [ 'function [drag_force, drag_accel] = DragEffect(speed_ms, Cd, A, m, rho)' newline ...
      '    drag_force = 0.5 * Cd * A * speed_ms^2 * rho;' newline ...
      '    drag_accel = drag_force / m;' newline ...
      'end' ]);

addConst(mdl, 'K_Cd',      'drag_coeff',   col(2)-40, row(2)-60);
addConst(mdl, 'K_A',       'frontal_area', col(2)-40, row(2)-20);
addConst(mdl, 'K_mass',    'mass',         col(2)-40, row(2)+20);
addConst(mdl, 'K_rho',     'air_density',  col(2)-40, row(2)+60);

% ---- Column 4: Torque Gear Ratio -------------------------
addMFBlock(mdl, 'TorqueGearRatio', blk(col(4), row(1)), ...
    [ 'function [corr_torque, thr_torque] = TorqueGearRatio(torque, gear, g1, g2, g3, g4, pr, sr, dr, eta, throttle)' newline ...
      '    ratios = [g1, g2, g3, g4];' newline ...
      '    thr_torque = torque * pr * sr * ratios(gear) * eta * dr;' newline ...
      '    corr_torque = thr_torque * (throttle / 100);' newline ...
      'end' ]);

addConst(mdl, 'K_prim',  'prim_red',              col(3)-40, row(1)-80);
addConst(mdl, 'K_sec',   'sec_red',               col(3)-40, row(1)-40);
addConst(mdl, 'K_eta',   'drivetrain_efficiency', col(3)-40, row(1));

% ---- Column 4: Deceleration ------------------------------
addMFBlock(mdl, 'DecelerationCalc', blk(col(4), row(2)), ...
    [ 'function neg_accel = DecelerationCalc(tire_coeff, g, drag_accel)' newline ...
      '    neg_accel = -(tire_coeff * g) - drag_accel;' newline ...
      'end' ]);

addConst(mdl, 'K_tire', 'tire_coeff', col(3)-40, row(2)-30);
addConst(mdl, 'K_g',    'g_accel',   col(3)-40, row(2)+30);

% ---- Column 5: Acceleration Calc -------------------------
addMFBlock(mdl, 'AccelerationCalc', blk(col(5), row(1)), ...
    [ 'function accel = AccelerationCalc(corr_torque, wheel_radius, mass)' newline ...
      '    accel = (corr_torque / wheel_radius) / mass;' newline ...
      'end' ]);

% ---- Column 5: Net Acceleration --------------------------
add_block('simulink/Math Operations/Add', [mdl '/NetAccel'], ...
    'Position', blk(col(5), row(2)), 'Inputs', '++');

% ---- Column 6: Speed Update (Euler) ----------------------
addMFBlock(mdl, 'SpeedUpdate', blk(col(6), row(1)), ...
    [ 'function final_speed = SpeedUpdate(init_speed, total_accel)' newline ...
      '    final_speed = init_speed + (total_accel / 10);' newline ...
      '    if final_speed < 0' newline ...
      '        final_speed = 0;' newline ...
      '    end' newline ...
      'end' ]);

% ---- Column 6: Kinetic Energy ----------------------------
addMFBlock(mdl, 'KineticEnergy', blk(col(6), row(2)), ...
    [ 'function KE = KineticEnergy(mass, final_speed)' newline ...
      '    KE = 0.5 * mass * final_speed^2;' newline ...
      'end' ]);

% ---- Column 7: Final Speed to RPM -----------------------
addMFBlock(mdl, 'SpeedToRPM', blk(col(7), row(1)), ...
    [ 'function rpm = SpeedToRPM(final_speed, gear, g1, g2, g3, g4, diff_red, wheel_radius)' newline ...
      '    ratios = [g1, g2, g3, g4];' newline ...
      '    combined = ratios(gear) * diff_red;' newline ...
      '    rpm = (final_speed * combined * 60) / (2 * pi * wheel_radius);' newline ...
      'end' ]);

% ---- Column 7: Distance Per Step -------------------------
addMFBlock(mdl, 'DistanceCalc', blk(col(7), row(2)), ...
    [ 'function delta_d = DistanceCalc(v_init, v_final, dt)' newline ...
      '    delta_d = ((v_init + v_final) / 2) * dt;' newline ...
      'end' ]);

addConst(mdl, 'K_dt', 'time_step', col(6)-40, row(2)+60);

% ---- Column 8: Gear Change Logic -------------------------
addMFBlock(mdl, 'GearChange', blk(col(8), row(1)), ...
    [ 'function [rpm_out, gear_out] = GearChange(rpm, gear, g1, g2, g3, g4, up, dn)' newline ...
      '    ratios = [g1, g2, g3, g4];' newline ...
      '    n = numel(ratios);' newline ...
      '    if rpm < dn && gear > 1' newline ...
      '        rpm_out  = (ratios(gear-1) * rpm) / ratios(gear);' newline ...
      '        gear_out = int8(gear - 1);' newline ...
      '    elseif rpm > up && gear < n' newline ...
      '        rpm_out  = (ratios(gear+1) * rpm) / ratios(gear);' newline ...
      '        gear_out = int8(gear + 1);' newline ...
      '    else' newline ...
      '        rpm_out  = rpm;' newline ...
      '        gear_out = int8(gear);' newline ...
      '    end' newline ...
      'end' ]);

addConst(mdl, 'K_rpm_up', 'rpm_shift_up',   col(7)-40, row(1)-30);
addConst(mdl, 'K_rpm_dn', 'rpm_shift_down',  col(7)-40, row(1)+30);

% ---- Column 8: Simulation End Check ----------------------
addMFBlock(mdl, 'EndCheck', blk(col(8), row(2)), ...
    [ 'function has_ended = EndCheck(distance_covered, dist)' newline ...
      '    has_ended = uint8(distance_covered >= dist);' newline ...
      'end' ]);

addConst(mdl, 'K_dist', 'track_dist_m', col(7)-40, row(2)+60);

% ---- Output Scopes / Sinks -------------------------------
c9 = col(9);
addScope(mdl, 'Scope_Speed',    c9, row(1),   'Vehicle Speed (km/h)');
addScope(mdl, 'Scope_RPM',      c9, row(2),   'Engine RPM');
addScope(mdl, 'Scope_Gear',     c9, row(3),   'Gear');
addScope(mdl, 'Scope_KE',       c9, row(4),   'Kinetic Energy (J)');
addScope(mdl, 'Scope_Distance', c9, row(5),   'Distance Covered (m)');

% To-Workspace blocks for logging
addToWS(mdl, 'Log_Speed',    c9+200, row(1), 'speed_log');
addToWS(mdl, 'Log_RPM',      c9+200, row(2), 'rpm_log');
addToWS(mdl, 'Log_KE',       c9+200, row(4), 'ke_log');
addToWS(mdl, 'Log_Distance', c9+200, row(5), 'dist_log');

% Speed unit conversion (m/s → km/h) before scope
add_block('simulink/Math Operations/Gain', [mdl '/SpeedToKmh'], ...
    'Position', blk(c9-80, row(1)), 'Gain', '3.6');

%% =========================================================
%  SIGNAL CONNECTIONS
% =========================================================
% Column flow: Delays → TorqueLookup/ThrottleCalc → SpeedCalc
%              → DragEffect → TorqueGearRatio → DecelerationCalc
%              → AccelerationCalc → NetAccel → SpeedUpdate
%              → SpeedToRPM/DistanceCalc → GearChange → Delays (feedback)

c = @(src, srcport, dst, dstport) ...
    add_line(mdl, [src '/' num2str(srcport)], [dst '/' num2str(dstport)], ...
             'autorouting', 'on');

% RPM delay → TorqueLookup and SpeedCalc
c('RPM_Delay',    1, 'TorqueLookup',   1);
c('RPM_Delay',    1, 'SpeedCalc',      1);

% Gear delay → SpeedCalc, TorqueGearRatio, SpeedToRPM, GearChange
c('Gear_Delay',   1, 'SpeedCalc',      2);
c('Gear_Delay',   1, 'TorqueGearRatio',2);
c('Gear_Delay',   1, 'SpeedToRPM',     2);
c('Gear_Delay',   1, 'GearChange',     2);

% Speed delay → SpeedCalc input not needed (SpeedCalc uses RPM)
%             → SpeedUpdate as init_speed and DistanceCalc
c('Speed_Delay',  1, 'SpeedUpdate',    1);
c('Speed_Delay',  1, 'DistanceCalc',   1);

% Distance integrator → ThrottleCalc and EndCheck
c('DistIntegrator',1, 'ThrottleCalc',  1);
c('DistIntegrator',1, 'EndCheck',      1);

% Constants → SpeedCalc
c('K_g1',      1, 'SpeedCalc', 3);
c('K_g2',      1, 'SpeedCalc', 4);
c('K_g3',      1, 'SpeedCalc', 5);
c('K_g4',      1, 'SpeedCalc', 6);
c('K_diff_red',1, 'SpeedCalc', 7);
c('K_wheel_r', 1, 'SpeedCalc', 8);

% SpeedCalc → DragEffect, ThrottleCalc
c('SpeedCalc', 1, 'DragEffect',   1);   % init_speed_ms
c('SpeedCalc', 2, 'ThrottleCalc', 3);   % speed_kmh

% Constants → DragEffect
c('K_Cd',   1, 'DragEffect', 2);
c('K_A',    1, 'DragEffect', 3);
c('K_mass', 1, 'DragEffect', 4);
c('K_rho',  1, 'DragEffect', 5);

% Track distance constant → ThrottleCalc
c('K_dist', 1, 'ThrottleCalc', 2);

% TorqueLookup + ThrottleCalc + gear constants → TorqueGearRatio
c('TorqueLookup',   1, 'TorqueGearRatio', 1);
% gear already connected above
c('K_g1',           1, 'TorqueGearRatio', 3);
c('K_g2',           1, 'TorqueGearRatio', 4);
c('K_g3',           1, 'TorqueGearRatio', 5);
c('K_g4',           1, 'TorqueGearRatio', 6);
c('K_prim',         1, 'TorqueGearRatio', 7);
c('K_sec',          1, 'TorqueGearRatio', 8);
c('K_diff_red',     1, 'TorqueGearRatio', 9);
c('K_eta',          1, 'TorqueGearRatio', 10);
c('ThrottleCalc',   1, 'TorqueGearRatio', 11);

% DragEffect → DecelerationCalc
c('DragEffect', 2, 'DecelerationCalc', 3);   % drag_accel
c('K_tire',     1, 'DecelerationCalc', 1);
c('K_g',        1, 'DecelerationCalc', 2);

% TorqueGearRatio → AccelerationCalc
c('TorqueGearRatio', 1, 'AccelerationCalc', 1);  % corrected_torque
c('K_wheel_r',       1, 'AccelerationCalc', 2);
c('K_mass',          1, 'AccelerationCalc', 3);

% AccelerationCalc + DecelerationCalc → NetAccel
c('AccelerationCalc', 1, 'NetAccel', 1);
c('DecelerationCalc', 1, 'NetAccel', 2);

% NetAccel + Speed → SpeedUpdate
c('NetAccel', 1, 'SpeedUpdate', 2);

% SpeedUpdate → KineticEnergy, SpeedToRPM, DistanceCalc, Speed_Delay
c('SpeedUpdate', 1, 'KineticEnergy', 2);
c('SpeedUpdate', 1, 'SpeedToRPM',   1);
c('SpeedUpdate', 1, 'DistanceCalc', 2);
c('SpeedUpdate', 1, 'Speed_Delay',  1);

% Constants → KineticEnergy
c('K_mass', 1, 'KineticEnergy', 1);

% SpeedToRPM constants
c('K_g1',      1, 'SpeedToRPM', 3);
c('K_g2',      1, 'SpeedToRPM', 4);
c('K_g3',      1, 'SpeedToRPM', 5);
c('K_g4',      1, 'SpeedToRPM', 6);
c('K_diff_red',1, 'SpeedToRPM', 7);
c('K_wheel_r', 1, 'SpeedToRPM', 8);

% DistanceCalc
c('K_dt', 1, 'DistanceCalc', 3);
c('DistanceCalc', 1, 'DistIntegrator', 1);  % delta_d → integrator

% SpeedToRPM + GearChange
c('SpeedToRPM', 1, 'GearChange', 1);   % rpm input
c('K_g1',       1, 'GearChange', 3);
c('K_g2',       1, 'GearChange', 4);
c('K_g3',       1, 'GearChange', 5);
c('K_g4',       1, 'GearChange', 6);
c('K_rpm_up',   1, 'GearChange', 7);
c('K_rpm_dn',   1, 'GearChange', 8);

% GearChange → feedback delays
c('GearChange', 1, 'RPM_Delay',  1);   % rpm_out
c('GearChange', 2, 'Gear_Delay', 1);   % gear_out

% EndCheck
c('K_dist',        1, 'EndCheck', 2);

% ---- Scopes & Logging ------------------------------------
c('SpeedUpdate',   1, 'SpeedToKmh',    1);
c('SpeedToKmh',    1, 'Scope_Speed',   1);
c('SpeedToKmh',    1, 'Log_Speed',     1);
c('GearChange',    1, 'Scope_RPM',     1);
c('GearChange',    1, 'Log_RPM',       1);
c('GearChange',    2, 'Scope_Gear',    1);
c('KineticEnergy', 1, 'Scope_KE',      1);
c('KineticEnergy', 1, 'Log_KE',        1);
c('DistIntegrator',1, 'Scope_Distance',1);
c('DistIntegrator',1, 'Log_Distance',  1);

%% =========================================================
%  SAVE AND OPEN
% =========================================================
Simulink.BlockDiagram.arrangeSystem(mdl);   % auto-layout
save_system(mdl, 'VehiclePhysics.slx');
fprintf('\n✓ Model saved as VehiclePhysics.slx\n');
fprintf('  Open with: open_system(''VehiclePhysics'')\n\n');


%% =========================================================
%  LOCAL HELPER FUNCTIONS (used during build only)
% =========================================================

function addConst(mdl, name, value_str, x, y)
    bw_ = 80; bh_ = 30;
    add_block('simulink/Sources/Constant', [mdl '/' name], ...
        'Position', [x-bw_/2, y-bh_/2, x+bw_/2, y+bh_/2], ...
        'Value', value_str);
end

function addScope(mdl, name, x, y, label)
    bw_ = 100; bh_ = 40;
    add_block('simulink/Sinks/Scope', [mdl '/' name], ...
        'Position', [x-bw_/2, y-bh_/2, x+bw_/2, y+bh_/2]);
    set_param([mdl '/' name], 'Name', name);
    % Label the scope title
    try
        scopeConfig = get_param([mdl '/' name], 'ScopeConfiguration');
        scopeConfig.Title = label;
    catch
        % Older Simulink API — title set via Name only
    end
end

function addToWS(mdl, name, x, y, var_name)
    bw_ = 110; bh_ = 40;
    add_block('simulink/Sinks/To Workspace', [mdl '/' name], ...
        'Position', [x-bw_/2, y-bh_/2, x+bw_/2, y+bh_/2], ...
        'VariableName', var_name, ...
        'SaveFormat',   'timeseries');
end

function coords = readCoords_local(filename)
    T      = readtable(filename);
    coords = [T.latitude, T.longitude];
end

function total_km = totalDistance_local(coords)
    total_km = 0;
    for i = 2:size(coords, 1)
        R    = 6371.0;
        toR  = pi / 180;
        lat1 = coords(i-1,1)*toR; lon1 = coords(i-1,2)*toR;
        lat2 = coords(i,  1)*toR; lon2 = coords(i,  2)*toR;
        dlat = lat2-lat1; dlon = lon2-lon1;
        a    = sin(dlat/2)^2 + cos(lat1)*cos(lat2)*sin(dlon/2)^2;
        total_km = total_km + R * 2 * atan2(sqrt(a), sqrt(1-a));
    end
end
