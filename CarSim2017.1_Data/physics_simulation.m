%% Vehicle Physics Simulation
% Converted from physics_prototype.py
% Structured for Simulink block integration
% MATLAB R2024 - No external toolboxes required
%
% NOTE: Requires '70_power_curve - Copy.csv', 'driving_log.csv',
%       and 'sem_apme_2025-track_coordinates.csv' in the working directory.

clear; clc;

%% =========================================================
%  1. GLOBAL PARAMETERS  (set once, read by all sub-functions)
% ==========================================================
p = vehicleParams();      % struct holding all constants & state
p = loadDataFiles(p);     % attach power-curve and driving-log tables
p = initState(p);         % initialise run-time state variables

%% =========================================================
%  2. TRACK SETUP
% ==========================================================
coords = readCoords('sem_apme_2025-track_coordinates.csv');
p.dist = totalDistance(coords) * 1000;   % metres

%% =========================================================
%  3. MAIN SIMULATION LOOP
% ==========================================================
filename = 'simulation_log_mine.csv';
fid = fopen(filename, 'w');
fprintf(fid, 'time,speed_kmh\n');

% --- first step (t = 0) ---
p = simStep(p, fid);

while p.has_ended == 0
    p.time = timeRound(p.time);
    p = hasEndedBasic(p);

    if p.final_speed_ms > 3
        p = simStep(p, fid);
        fprintf('Rounded RPM: %.1f\n', p.rounded_rpm);
    else
        fprintf('Simulation has ended.\n');
        break;
    end
end

fclose(fid);
fprintf('\n=== Simulation complete. Results written to %s ===\n', filename);


%% =========================================================
%  SIMULINK-READY FUNCTION BLOCKS
%  Each function below maps 1-to-1 to a Simulink MATLAB
%  Function block.  Inputs/outputs are explicit – no globals.
% ==========================================================

% ---------------------------------------------------------
% Block: vehicleParams
%   Returns a struct of all fixed vehicle parameters.
%   In Simulink: use a "Model Workspace" or "Bus Creator"
%   to expose individual fields as signals.
% ---------------------------------------------------------
function p = vehicleParams()
    p.drivetrain_efficiency = 0.91;
    p.tire_coeff            = 0.00475;
    p.g                     = 9.81;
    p.prim_red              = 1;
    p.sec_red               = 1;
    p.diff_red              = 4.1;
    p.gears                 = [3.78, 2.12, 1.36, 1.03];  % 1-4
    p.idle_rpm              = 750;
    p.frontal_area          = 1.6;    % m^2
    p.drag_coeff            = 0.30;
    p.mass                  = 1020;   % kg
    p.wheel_radius          = 0.298;  % m
    p.time_step             = 0.1;    % s
    p.air_density           = 1.225;  % kg/m^3 (sea level)
    p.rpm_shift_up          = 2300;
    p.rpm_shift_down        = 1398;
end

% ---------------------------------------------------------
% Block: initState
%   Initialises all run-time state variables.
% ---------------------------------------------------------
function p = initState(p)
    p.rpm              = 4500;
    p.gear             = 1;
    p.throttle         = 0;
    p.init_speed_ms    = 0;
    p.final_speed_ms   = 0;
    p.speed_kmh        = 0;
    p.distance_covered = 0;
    p.time             = 0;
    p.has_ended        = 0;
    p.accel            = 0;
    p.neg_acceleration = 0;
    p.total_accel      = 0;
    p.drag_force       = 0;
    p.drag_accel       = 0;
    p.corrected_torque = 0;
    p.throttle_torque  = 0;
    p.torque           = 0;
    p.hp               = 0;
    p.rounded_rpm      = 0;
    p.KE_final         = 0;
    p.Total_KE         = 0;
    p.dist             = 0;
end

% ---------------------------------------------------------
% Block: loadDataFiles
%   Loads CSV tables into the parameter struct.
%   In Simulink: implement as an Initialization callback or
%   a From-File block feeding a lookup table.
% ---------------------------------------------------------
function p = loadDataFiles(p)
    p.power_curve  = readtable('70_power_curve - Copy.csv');
    p.driving_log  = readtable('driving_log.csv');
end

% ---------------------------------------------------------
% Block: simStep
%   One complete physics time-step.
%   Simulink equivalent: Atomic Subsystem triggered every Ts.
%   Inputs : parameter/state struct p, file-id fid
%   Outputs: updated parameter/state struct p
% ---------------------------------------------------------
function p = simStep(p, fid)
    p.torque = getInterpolatedTorque(p.rpm, p.power_curve);
    p        = throttleCalc(p);
    p        = speedCalc(p);
    p        = dragEffect(p);
    p        = torqueGearRatioCalc(p);
    p        = decelerationCalc(p);
    p        = accelerationCalc(p);
    p        = accelTotal(p);
    p        = speedUpdate(p);
    p        = kineticEnergy(p);
    p.rpm    = finalSpeedToRpm(p);
    p        = gearChange(p);

    fprintf('RPM: %.1f | Speed: %.2f km/h | Gear: %d | Time: %.1f s\n', ...
            p.rpm, p.final_speed_ms*3.6, p.gear, p.time);

    new_distance       = distancePerTime(p);
    p.distance_covered = p.distance_covered + new_distance;

    fprintf('Distance traveled: %.2f m\n', p.distance_covered);

    % --- log to CSV ---
    fprintf(fid, '%.1f,%.4f\n', p.time, p.final_speed_ms * 3.6);

    p.time = p.time + p.time_step;
    p.time = timeRound(p.time);

    fprintf('Kinetic Energy: %.2f J\n', p.Total_KE);
    fprintf('--------------------------------------------------\n');
end

% ---------------------------------------------------------
% Block: throttleCalc
%   Simulink inputs : distance_covered, dist, speed_kmh
%   Simulink outputs: throttle
% ---------------------------------------------------------
function p = throttleCalc(p)
    if p.distance_covered < p.dist
        if p.speed_kmh <= 15
            p.throttle = 30;
        elseif p.speed_kmh >= 35
            p.throttle = 0;
        end
    else
        p.throttle = 0;
    end
    fprintf('Throttle: %.1f %%\n', p.throttle);
end

% ---------------------------------------------------------
% Block: hasEndedBasic
%   Simulink inputs : distance_covered, dist
%   Simulink outputs: has_ended (boolean)
% ---------------------------------------------------------
function p = hasEndedBasic(p)
    if p.distance_covered >= p.dist
        p.has_ended = 1;
    else
        p.has_ended = 0;
    end
    fprintf('Has ended: %d\n', p.has_ended);
end

% ---------------------------------------------------------
% Block: dragEffect
%   Simulink inputs : init_speed_ms, drag_coeff,
%                     frontal_area, mass, air_density
%   Simulink outputs: drag_force, drag_accel
% ---------------------------------------------------------
function p = dragEffect(p)
    p.drag_force = 0.5 * p.drag_coeff * p.frontal_area ...
                   * (p.init_speed_ms^2) * p.air_density;
    p.drag_accel = p.drag_force / p.mass;
    fprintf('Drag Force: %.2f N | Drag Accel: %.4f m/s^2\n', ...
            p.drag_force, p.drag_accel);
end

% ---------------------------------------------------------
% Block: speedCalc
%   Converts RPM + gear to vehicle speed.
%   Simulink inputs : rpm, gear, gears, diff_red,
%                     wheel_radius
%   Simulink outputs: init_speed_ms, speed_kmh
% ---------------------------------------------------------
function p = speedCalc(p)
    if p.gear < 1 || p.gear > numel(p.gears)
        error('speedCalc: invalid gear %d', p.gear);
    end
    gear_ratio     = p.gears(p.gear);
    combined_ratio = gear_ratio * p.diff_red;
    fprintf('Combined ratio: %.4f\n', combined_ratio);
    p.init_speed_ms = (p.rpm * 2 * pi * p.wheel_radius) / (60 * combined_ratio);
    p.speed_kmh     = p.init_speed_ms * 3.6;
end

% ---------------------------------------------------------
% Block: decelerationCalc
%   Simulink inputs : tire_coeff, g, drag_accel
%   Simulink outputs: neg_acceleration
% ---------------------------------------------------------
function p = decelerationCalc(p)
    p.neg_acceleration = -(p.tire_coeff * p.g) - p.drag_accel;
    fprintf('Neg Acceleration: %.4f m/s^2\n', p.neg_acceleration);
end

% ---------------------------------------------------------
% Block: torqueGearRatioCalc
%   Applies gear + drivetrain ratios and throttle position.
%   Simulink inputs : torque, gear, gears, prim_red, sec_red,
%                     diff_red, drivetrain_efficiency, throttle
%   Simulink outputs: throttle_torque, corrected_torque
% ---------------------------------------------------------
function p = torqueGearRatioCalc(p)
    if p.gear < 1 || p.gear > numel(p.gears)
        error('torqueGearRatioCalc: invalid gear %d', p.gear);
    end
    gear_ratio        = p.gears(p.gear);
    p.throttle_torque = p.torque * p.prim_red * p.sec_red ...
                        * gear_ratio * p.drivetrain_efficiency * p.diff_red;
    p.corrected_torque = p.throttle_torque * (p.throttle / 100);
    fprintf('Throttle Torque: %.2f Nm | Corrected Torque: %.2f Nm\n', ...
            p.throttle_torque, p.corrected_torque);
    fprintf('Raw Torque: %.2f Nm | Throttle: %.1f %%\n', ...
            p.torque, p.throttle);
end

% ---------------------------------------------------------
% Block: accelerationCalc
%   Simulink inputs : corrected_torque, wheel_radius, mass
%   Simulink outputs: accel
% ---------------------------------------------------------
function p = accelerationCalc(p)
    % Force at wheel / effective mass
    p.accel = (p.corrected_torque / p.wheel_radius) / p.mass;
    fprintf('Acceleration: %.4f m/s^2\n', p.accel);
end

% ---------------------------------------------------------
% Block: accelTotal
%   Simulink inputs : accel, neg_acceleration
%   Simulink outputs: total_accel
% ---------------------------------------------------------
function p = accelTotal(p)
    p.total_accel = p.accel + p.neg_acceleration;
    fprintf('Total Acceleration: %.4f m/s^2\n', p.total_accel);
end

% ---------------------------------------------------------
% Block: speedUpdate
%   Euler integration of speed.
%   NOTE: Original Python divides total_accel by 10 rather
%         than multiplying by time_step; preserved here to
%         match original behaviour exactly.
%   Simulink inputs : init_speed_ms, total_accel
%   Simulink outputs: final_speed_ms
% ---------------------------------------------------------
function p = speedUpdate(p)
    p.final_speed_ms = p.init_speed_ms + (p.total_accel / 10);
    fprintf('Init speed: %.4f m/s | Final speed: %.2f km/h\n', ...
            p.init_speed_ms, p.final_speed_ms * 3.6);
    if p.final_speed_ms < 0
        p.final_speed_ms = 0;
    end
end

% ---------------------------------------------------------
% Block: kineticEnergy
%   Simulink inputs : mass, final_speed_ms
%   Simulink outputs: KE_final, Total_KE
% ---------------------------------------------------------
function p = kineticEnergy(p)
    p.KE_final = 0.5 * p.mass * (p.final_speed_ms^2);
    p.Total_KE = p.KE_final;
    fprintf('KE Final: %.2f J | Total KE: %.2f J\n', p.KE_final, p.Total_KE);
end

% ---------------------------------------------------------
% Block: distancePerTime
%   Trapezoidal distance over one time step.
%   Simulink inputs : init_speed_ms, final_speed_ms, time_step
%   Simulink outputs: distance (scalar, metres)
% ---------------------------------------------------------
function distance = distancePerTime(p)
    distance = ((p.init_speed_ms + p.final_speed_ms) / 2) * p.time_step;
end

% ---------------------------------------------------------
% Block: finalSpeedToRpm
%   Converts final speed back to engine RPM.
%   Simulink inputs : final_speed_ms, gear, gears,
%                     diff_red, wheel_radius
%   Simulink outputs: rpm
% ---------------------------------------------------------
function rpm = finalSpeedToRpm(p)
    if p.gear < 1 || p.gear > numel(p.gears)
        error('finalSpeedToRpm: invalid gear %d', p.gear);
    end
    gear_ratio     = p.gears(p.gear);
    combined_ratio = gear_ratio * p.diff_red;
    rpm = (p.final_speed_ms * combined_ratio * 60) / (2 * pi * p.wheel_radius);
    fprintf('New RPM: %.1f\n', rpm);
end

% ---------------------------------------------------------
% Block: gearChange
%   Automatic upshift / downshift logic.
%   Simulink inputs : rpm, gear, gears,
%                     rpm_shift_up, rpm_shift_down
%   Simulink outputs: rpm, gear
% ---------------------------------------------------------
function p = gearChange(p)
    num_gears = numel(p.gears);
    if p.rpm < p.rpm_shift_down && p.gear > 1
        % Downshift
        p.rpm  = (p.gears(p.gear - 1) * p.rpm) / p.gears(p.gear);
        p.gear = p.gear - 1;
        fprintf('Gear changed DOWN to %d\n', p.gear);
    elseif p.rpm > p.rpm_shift_up && p.gear < num_gears
        % Upshift
        p.rpm  = (p.gears(p.gear + 1) * p.rpm) / p.gears(p.gear);
        p.gear = p.gear + 1;
        fprintf('Gear changed UP to %d\n', p.gear);
    else
        fprintf('No gear change. Current gear: %d\n', p.gear);
    end
end

% ---------------------------------------------------------
% Block: getInterpolatedTorque
%   Linear interpolation on the power-curve table.
%   Simulink equivalent: 1-D Lookup Table block with
%   linear interpolation, fed by current RPM signal.
%   Inputs : current_rpm (scalar), power_curve (table)
%   Output : torque (Nm)
%
%   NOTE: The +70 / -70 offset from the original Python is
%         preserved; review against calibration data.
% ---------------------------------------------------------
function torque = getInterpolatedTorque(current_rpm, power_curve)
    rpm_data    = power_curve.rpm;
    torque_data = power_curve.torque;

    if current_rpm <= min(rpm_data)
        torque = torque_data(1);
        return;
    end
    if current_rpm >= max(rpm_data)
        torque = torque_data(end);
        return;
    end

    % Find bracketing indices
    idx_above = find(rpm_data > current_rpm, 1, 'first');
    idx_below = idx_above - 1;

    x1 = rpm_data(idx_below);    y1 = torque_data(idx_below) + 70;
    x2 = rpm_data(idx_above);    y2 = torque_data(idx_above) + 70;

    torque = y1 + (current_rpm - x1) * (y2 - y1) / (x2 - x1);
    torque = torque - 70;

    fprintf('Interpolated Torque at %.0f RPM: %.2f Nm\n', current_rpm, torque);
end

% ---------------------------------------------------------
% Utility: timeRound
%   Rounds simulation time to 1 decimal place.
%   Simulink: not needed (Simulink manages its own clock).
% ---------------------------------------------------------
function t = timeRound(t)
    t = round(t, 1);
end

% ---------------------------------------------------------
% Utility: readCoords
%   Reads lat/lon from a CSV file.
%   Simulink: load offline; pass coords array as a parameter.
% ---------------------------------------------------------
function coords = readCoords(filename)
    T      = readtable(filename);
    coords = [T.latitude, T.longitude];
end

% ---------------------------------------------------------
% Utility: totalDistance
%   Sums haversine distances over a coordinate list (km).
%   Simulink: pre-compute and supply as a constant.
% ---------------------------------------------------------
function total_km = totalDistance(coords)
    total_km = 0;
    for i = 2:size(coords, 1)
        d        = haversine(coords(i-1,1), coords(i-1,2), ...
                             coords(i,  1), coords(i,  2));
        total_km = total_km + d;
        fprintf('Cumulative distance: %.4f km\n', total_km);
    end
end

% ---------------------------------------------------------
% Utility: haversine
%   Great-circle distance between two lat/lon points (km).
%   Simulink: implement as a MATLAB Function block if
%   real-time track distance is needed.
%   Inputs : lat1, lon1, lat2, lon2 (degrees)
%   Output : distance (km)
% ---------------------------------------------------------
function d = haversine(lat1, lon1, lat2, lon2)
    R = 6371.0;   % Earth radius, km
    deg2rad = pi / 180;
    lat1 = lat1 * deg2rad;  lon1 = lon1 * deg2rad;
    lat2 = lat2 * deg2rad;  lon2 = lon2 * deg2rad;
    dlat = lat2 - lat1;
    dlon = lon2 - lon1;
    a    = sin(dlat/2)^2 + cos(lat1) * cos(lat2) * sin(dlon/2)^2;
    c    = 2 * atan2(sqrt(a), sqrt(1 - a));
    d    = R * c;
end

% ---------------------------------------------------------
% Utility: calculateBearing
%   Forward azimuth between two lat/lon points (degrees).
%   Simulink: MATLAB Function block for steering / heading.
%   Inputs : lat1, lon1, lat2, lon2 (degrees)
%   Output : bearing (0-360 degrees)
% ---------------------------------------------------------
function bearing = calculateBearing(lat1, lon1, lat2, lon2)
    deg2rad = pi / 180;
    lat1 = lat1 * deg2rad;  lon1 = lon1 * deg2rad;
    lat2 = lat2 * deg2rad;  lon2 = lon2 * deg2rad;
    dlon = lon2 - lon1;
    x    = sin(dlon) * cos(lat2);
    y    = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    bearing = mod(atan2(x, y) * (180/pi) + 360, 360);
end
