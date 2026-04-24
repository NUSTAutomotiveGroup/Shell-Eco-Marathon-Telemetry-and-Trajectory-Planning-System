pc = readtable('70_power_curve - Copy.csv');

% Smooth the 1D curve first
rpm_fine    = (min(pc.rpm) : 10 : max(pc.rpm))';
torque_fine = interp1(pc.rpm, pc.torque, rpm_fine, 'pchip');
torque_base = smoothdata(torque_fine, 'gaussian', 150);

% Define throttle breakpoints 0-100%
throttle_bp = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

% Build 2D map — rows = RPM, columns = throttle
% Engine torque response is nonlinear with throttle:
% low throttle = disproportionately low torque (throttle plate restriction)
throttle_scale = [0, 0.05, 0.12, 0.22, 0.34, 0.48, 0.62, 0.75, 0.86, 0.94, 1.0];

torque_map = zeros(length(rpm_fine), length(throttle_bp));
for i = 1:length(throttle_bp)
    torque_map(:, i) = torque_base * throttle_scale(i);
end

% Verify the map looks correct
figure;
surf(throttle_bp, rpm_fine, torque_map);
xlabel('Throttle (%)');
ylabel('RPM');
zlabel('Torque (Nm)');
title('2D Torque Map');
colorbar;

% Save to workspace
rpm_bp      = rpm_fine;
hws = get_param('Huginn', 'modelworkspace');
hws.assignin('rpm_bp',      rpm_bp);
hws.assignin('throttle_bp', throttle_bp);
hws.assignin('torque_map',  torque_map);


filename = 'Fuel_Map.csv';   % replace with your filename
raw      = readmatrix(filename);
% Extract axes and table body
throttle_bp = raw(1, 2:end);   % first row, skip corner cell
rpm_bp      = raw(2:end, 1);   % first column, skip corner cell
fuel_map    = raw(2:end, 2:end); % body of the table
fprintf('RPM breakpoints:      %d points (%d to %d RPM)\n', ...
length(rpm_bp), min(rpm_bp), max(rpm_bp));
fprintf('Throttle breakpoints: %d points (%d to %d %%)\n', ...
length(throttle_bp), min(throttle_bp), max(throttle_bp));
fprintf('Fuel map size:        %d x %d\n', size(fuel_map));
%% ---- Step 2: Validate ----
% Check for monotonically increasing breakpoints
if any(diff(rpm_bp) <= 0)
error('RPM breakpoints are not monotonically increasing — check CSV');
end
if any(diff(throttle_bp) <= 0)
error('Throttle breakpoints are not monotonically increasing — check CSV');
end
% Check for NaN or negative values
if any(isnan(fuel_map(:)))
warning('NaN values found in fuel map — replacing with 0');
fuel_map(isnan(fuel_map)) = 0;
end
if any(fuel_map(:) < 0)
warning('Negative fuel values found — replacing with 0');
fuel_map(fuel_map < 0) = 0;
end
%% ---- Step 3: Interpolate onto finer grid ----
% Finer RPM grid — 10 RPM steps
rpm_fine      = (min(rpm_bp) : 10 : max(rpm_bp))';
% Finer throttle grid — 1% steps
throttle_fine = (min(throttle_bp) : 1 : max(throttle_bp))';
% Interpolate using meshgrid and interp2
[T_orig, R_orig] = meshgrid(throttle_bp, rpm_bp);
[T_fine, R_fine] = meshgrid(throttle_fine, rpm_fine);
fuel_map_fine = interp2(T_orig, R_orig, fuel_map, T_fine, R_fine, 'linear');
% Clip any extrapolated negatives
fuel_map_fine = max(fuel_map_fine, 0);
fprintf('Interpolated map size: %d RPM x %d throttle points\n', ...
length(rpm_fine), length(throttle_fine));
%% ---- Step 4: Plot to verify ----
figure;
surf(throttle_fine, rpm_fine, fuel_map_fine, 'EdgeColor', 'none');
xlabel('Throttle (%)');
ylabel('RPM');
zlabel('Fuel Consumption');
title('2D Fuel Map — Interpolated');
colorbar;
view(45, 30);
% Also plot as contour for easier reading
figure;
contourf(throttle_fine, rpm_fine, fuel_map_fine, 20);
xlabel('Throttle (%)');
ylabel('RPM');
title('2D Fuel Map — Contour');
colorbar;
%% ---- Step 5: Load into Simulink Model Workspace ----
mdl = 'Huginn';   % replace with your model name
hws = get_param(mdl, 'modelworkspace');
hws.assignin('rpm_bp_fuel',      rpm_fine);
hws.assignin('throttle_bp_fuel', throttle_fine);
hws.assignin('fuel_map',         fuel_map_fine);
fprintf('\nLoaded into model workspace:\n');
fprintf('  rpm_bp_fuel:      %d points\n', length(rpm_fine));
fprintf('  throttle_bp_fuel: %d points\n', length(throttle_fine));
fprintf('  fuel_map:         [%d x %d]\n', size(fuel_map_fine));
%% ---- Step 6: Configure the 2D Lookup Table block ----
% If you already have a 2D Lookup Table block named FuelMap in your model:
try
set_param([mdl '/FuelMap'], ...
'BreakpointsForDimension1', 'rpm_bp_fuel', ...
'BreakpointsForDimension2', 'throttle_bp_fuel', ...
'Table',                    'fuel_map', ...
'InterpMethod',             'Linear point-slope', ...
'ExtrapMethod',             'Clip');
fprintf('FuelMap block configured successfully\n');
catch
fprintf('FuelMap block not found — add a 2-D Lookup Table block manually\n');
fprintf('Set Breakpoints 1 = rpm_bp_fuel\n');
fprintf('Set Breakpoints 2 = throttle_bp_fuel\n');
fprintf('Set Table data   = fuel_map\n');
end
%% ---- Step 7: Save interpolated map back to CSV ----
% Useful for documentation or sharing
out_header  = ['rpm_throttle', ...
arrayfun(@(x) sprintf(',%d', x), throttle_fine', ...
'UniformOutput', false)];
out_header  = [out_header{:}];
fid = fopen('fuel_map_interpolated.csv', 'w');
fprintf(fid, '%s\n', out_header);
for i = 1:length(rpm_fine)
fprintf(fid, '%d', rpm_fine(i));
fprintf(fid, ',%.6f', fuel_map_fine(i, :));
fprintf(fid, '\n');
end
fclose(fid);
fprintf('Interpolated map saved to fuel_map_interpolated.csv\n');

fprintf('Torque map size: %d RPM points x %d throttle points\n', ...
        length(rpm_bp), length(throttle_bp));