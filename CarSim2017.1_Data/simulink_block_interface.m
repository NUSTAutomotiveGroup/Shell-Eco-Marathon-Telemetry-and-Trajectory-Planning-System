%% simulink_block_interface.m
% =========================================================
% SIMULINK INTEGRATION REFERENCE
% Vehicle Physics Simulation — MATLAB R2024
% =========================================================
%
% This file defines wrapper functions for use directly as
% MATLAB Function blocks in Simulink.  Each function has
% explicit scalar/vector inputs & outputs (no structs, no
% global state) so Simulink can infer port data types and
% sizes automatically.
%
% HOW TO USE IN SIMULINK
% -------------------------------------------------------
% 1. Drag a "MATLAB Function" block from the Simulink
%    Library Browser into your model.
% 2. Double-click the block and paste the desired function
%    body (excluding the function signature — Simulink
%    generates that from the block's port names).
% 3. Each input listed below becomes an inport; each output
%    becomes an outport.
% 4. Connect signals using the port names as guides.
%
% RECOMMENDED SOLVER
%   Type  : Fixed-step
%   Solver: discrete (no continuous states)
%   Ts    : 0.1 s  (matches time_step in vehicleParams)
%
% CONSTANTS / PARAMETERS
%   Store fixed vehicle parameters (mass, ratios, etc.) in
%   the Model Workspace (Modeling > Model Workspace) as
%   MATLAB variables so all blocks can share them without
%   re-declaring them.
% =========================================================


%% ---- BLOCK 1: Throttle Calculator ----------------------
% Ports:
%   IN  : distance_covered, dist, speed_kmh
%   OUT : throttle  (0-100 %)
function throttle = blk_throttleCalc(distance_covered, dist, speed_kmh)
    if distance_covered < dist
        if speed_kmh <= 15
            throttle = 30;
        elseif speed_kmh >= 35
            throttle = 0;
        else
            throttle = 30;   % hold value in hysteresis band
        end
    else
        throttle = 0;
    end
end


%% ---- BLOCK 2: Drag Effect ------------------------------
% Ports:
%   IN  : init_speed_ms, drag_coeff, frontal_area,
%          mass, air_density
%   OUT : drag_force (N), drag_accel (m/s^2)
function [drag_force, drag_accel] = blk_dragEffect( ...
        init_speed_ms, drag_coeff, frontal_area, mass, air_density)
    drag_force = 0.5 * drag_coeff * frontal_area * init_speed_ms^2 * air_density;
    drag_accel = drag_force / mass;
end


%% ---- BLOCK 3: Speed from RPM ---------------------------
% Ports:
%   IN  : rpm, gear, diff_red, wheel_radius
%          gear_ratios (4-element row vector — use Bus or
%          four separate ports in Simulink)
%   OUT : init_speed_ms (m/s), speed_kmh (km/h)
function [init_speed_ms, speed_kmh] = blk_speedCalc( ...
        rpm, gear, g1, g2, g3, g4, diff_red, wheel_radius)
    gear_ratios    = [g1, g2, g3, g4];
    gear_ratio     = gear_ratios(gear);
    combined_ratio = gear_ratio * diff_red;
    init_speed_ms  = (rpm * 2 * pi * wheel_radius) / (60 * combined_ratio);
    speed_kmh      = init_speed_ms * 3.6;
end


%% ---- BLOCK 4: Deceleration (rolling + aero) ------------
% Ports:
%   IN  : tire_coeff, g, drag_accel
%   OUT : neg_acceleration (m/s^2, negative value)
function neg_acceleration = blk_decelerationCalc(tire_coeff, g, drag_accel)
    neg_acceleration = -(tire_coeff * g) - drag_accel;
end


%% ---- BLOCK 5: Wheel Torque with Gear Ratios ------------
% Ports:
%   IN  : torque, gear, g1..g4, prim_red, sec_red,
%          diff_red, drivetrain_efficiency, throttle
%   OUT : corrected_torque (Nm), throttle_torque (Nm)
function [corrected_torque, throttle_torque] = blk_torqueGearRatioCalc( ...
        torque, gear, g1, g2, g3, g4, ...
        prim_red, sec_red, diff_red, drivetrain_efficiency, throttle)
    gear_ratios    = [g1, g2, g3, g4];
    gear_ratio     = gear_ratios(gear);
    throttle_torque  = torque * prim_red * sec_red * gear_ratio ...
                       * drivetrain_efficiency * diff_red;
    corrected_torque = throttle_torque * (throttle / 100);
end


%% ---- BLOCK 6: Longitudinal Acceleration ----------------
% Ports:
%   IN  : corrected_torque (Nm), wheel_radius (m), mass (kg)
%   OUT : accel (m/s^2)
function accel = blk_accelerationCalc(corrected_torque, wheel_radius, mass)
    accel = (corrected_torque / wheel_radius) / mass;
end


%% ---- BLOCK 7: Net Acceleration -------------------------
% Ports:
%   IN  : accel, neg_acceleration
%   OUT : total_accel (m/s^2)
function total_accel = blk_accelTotal(accel, neg_acceleration)
    total_accel = accel + neg_acceleration;
end


%% ---- BLOCK 8: Speed Update (Euler Integration) ---------
% Ports:
%   IN  : init_speed_ms, total_accel
%   OUT : final_speed_ms (m/s)
%
% NOTE: Divisor of 10 matches original Python; replace with
%       time_step (0.1 s) once validated for your use case.
function final_speed_ms = blk_speedUpdate(init_speed_ms, total_accel)
    final_speed_ms = init_speed_ms + (total_accel / 10);
    if final_speed_ms < 0
        final_speed_ms = 0;
    end
end


%% ---- BLOCK 9: Kinetic Energy ---------------------------
% Ports:
%   IN  : mass, final_speed_ms
%   OUT : KE_final (J)
function KE_final = blk_kineticEnergy(mass, final_speed_ms)
    KE_final = 0.5 * mass * final_speed_ms^2;
end


%% ---- BLOCK 10: Distance per Time Step ------------------
% Ports:
%   IN  : init_speed_ms, final_speed_ms, time_step
%   OUT : delta_distance (m)
function delta_distance = blk_distancePerTime( ...
        init_speed_ms, final_speed_ms, time_step)
    delta_distance = ((init_speed_ms + final_speed_ms) / 2) * time_step;
end


%% ---- BLOCK 11: Speed to RPM ----------------------------
% Ports:
%   IN  : final_speed_ms, gear, g1..g4,
%          diff_red, wheel_radius
%   OUT : rpm
function rpm = blk_finalSpeedToRpm( ...
        final_speed_ms, gear, g1, g2, g3, g4, diff_red, wheel_radius)
    gear_ratios    = [g1, g2, g3, g4];
    gear_ratio     = gear_ratios(gear);
    combined_ratio = gear_ratio * diff_red;
    rpm = (final_speed_ms * combined_ratio * 60) / (2 * pi * wheel_radius);
end


%% ---- BLOCK 12: Gear Change Logic -----------------------
% Ports:
%   IN  : rpm, gear, g1..g4, rpm_shift_up, rpm_shift_down
%   OUT : rpm_out, gear_out
%
% Simulink note: gear is an integer — set port data type
% to int8 or uint8 in the block's Port & Data Types dialog.
function [rpm_out, gear_out] = blk_gearChange( ...
        rpm, gear, g1, g2, g3, g4, rpm_shift_up, rpm_shift_down)
    gear_ratios = [g1, g2, g3, g4];
    num_gears   = numel(gear_ratios);
    if rpm < rpm_shift_down && gear > 1
        rpm_out  = (gear_ratios(gear-1) * rpm) / gear_ratios(gear);
        gear_out = gear - 1;
    elseif rpm > rpm_shift_up && gear < num_gears
        rpm_out  = (gear_ratios(gear+1) * rpm) / gear_ratios(gear);
        gear_out = gear + 1;
    else
        rpm_out  = rpm;
        gear_out = gear;
    end
end


%% ---- BLOCK 13: Torque Lookup (1-D Interpolation) -------
% Simulink native alternative:
%   Use a "1-D Lookup Table" block with:
%     Breakpoints: rpm column from power-curve CSV
%     Table data : torque column (+ 70 offset if required)
%   Set Interpolation to "Linear" and Extrapolation to
%   "Clip".  This is preferred over a MATLAB Function block
%   for real-time performance.
%
% MATLAB Function block fallback (requires data in workspace):
%   Pre-load rpm_bp and torque_bp as workspace variables.
% Ports:
%   IN  : current_rpm, rpm_bp (Nx1), torque_bp (Nx1)
%   OUT : torque
function torque = blk_getInterpolatedTorque(current_rpm, rpm_bp, torque_bp)
    % Offset carried over from original Python — review vs calibration
    torque_bp_adj = torque_bp + 70;
    torque = interp1(rpm_bp, torque_bp_adj, current_rpm, 'linear', 'extrap');
    torque = torque - 70;
end


%% ---- BLOCK 14: Simulation End Condition ----------------
% Ports:
%   IN  : distance_covered, dist
%   OUT : has_ended (boolean / uint8)
function has_ended = blk_hasEndedBasic(distance_covered, dist)
    has_ended = uint8(distance_covered >= dist);
end


%% ---- UTILITY: Haversine (offline pre-computation) ------
% Not needed as a Simulink block unless the track is dynamic.
% Call from the model's InitFcn callback to compute dist.
function d = util_haversine(lat1, lon1, lat2, lon2)
    R    = 6371.0;
    toR  = pi / 180;
    lat1 = lat1*toR; lon1 = lon1*toR;
    lat2 = lat2*toR; lon2 = lon2*toR;
    dlat = lat2 - lat1;  dlon = lon2 - lon1;
    a    = sin(dlat/2)^2 + cos(lat1)*cos(lat2)*sin(dlon/2)^2;
    d    = R * 2 * atan2(sqrt(a), sqrt(1-a));
end
