%% track_speed_display.m
% Displays vehicle speed as height on the track map in 3D
% Independently of any Simulink model
%
% INPUTS REQUIRED:
%   station_data : vector of distance values (metres) from your model output
%   speed_data   : vector of speed values (m/s or km/h) from your model output
%   coords CSV   : sem_apme_2025-track_coordinates.csv in working directory
%
% USAGE:
%   1. Run your model to generate station and speed outputs
%   2. Load them into workspace as station_data and speed_data
%   3. Run this script
%
% Example to generate test data (replace with your model output):
%   station_data = linspace(0, 3676, 500)';
%   speed_data   = 20 + 10*sin(station_data/300) + randn(500,1);

%% =========================================================
%  STEP 1 — Load track coordinates
% =========================================================
coords   = readtable('sem_apme_2025-track_coordinates.csv');
lat      = coords.latitude;
lon      = coords.longitude;
alt      = coords.altitude;
dist_m   = coords.distance * 1000;   % convert km to metres

% Convert lat/lon to local XY metres for plotting
% Use first point as origin
lat0 = lat(1);
lon0 = lon(1);
R    = 6371000;   % Earth radius metres

x_track = R * cosd(lat0) * (lon - lon0) * pi/180;   % East metres
y_track = R * (lat - lat0) * pi/180;                  % North metres

fprintf('Track loaded: %d waypoints, %.1f m total\n', ...
        length(lat), max(dist_m));

%% =========================================================
%  STEP 2 — Load your model speed output
%  Replace this section with your actual model data
% =========================================================

% --- REPLACE BELOW WITH YOUR MODEL OUTPUT ---
% station_data should be distance in metres along track
% speed_data   should be speed in km/h

% Example placeholder — delete and replace with your data:


fprintf('Speed data loaded: %d points\n', length(station_data));
fprintf('Speed range: %.1f to %.1f km/h\n', ...
        min(speed_data), max(speed_data));

%% =========================================================
%  STEP 3 — Interpolate speed onto track waypoints
% =========================================================
% Map speed from station (distance) onto each track waypoint

% Clip station data to track length
station_data = min(station_data, max(dist_m));
station_data = max(station_data, 0);

% Interpolate speed at each track waypoint distance
speed_on_track = speed_data;;

% Scale speed to use as height above track
% Speed height is offset above the real altitude
speed_scale  = 0.5;    % metres per km/h — adjust for visual clarity
speed_height = alt + speed_on_track * speed_scale;

%% =========================================================
%  STEP 4 — Plot
% =========================================================
figure('Name', 'Track Speed Profile', ...
       'Position', [100, 100, 1200, 700], ...
       'Color', [0.12 0.12 0.15]);

%% --- Plot 1: 3D track with speed as height ---
ax1 = subplot(2, 2, [1 3]);

% Colour map speed values
speed_norm = (speed_on_track - min(speed_on_track)) / ...
             (max(speed_on_track) - min(speed_on_track) + eps);

% Plot track base (actual altitude)
plot3(x_track, y_track, alt, ...
      'Color', [0.4 0.4 0.4], 'LineWidth', 1);
hold on;

% Plot speed surface as coloured line above track
% Use patch for gradient colouring
for i = 1:length(x_track)-1
    c = speed_norm(i);
    % Colour: blue (slow) → green → red (fast)
    col = [c, 1-abs(2*c-1), 1-c];
    col = max(0, min(1, col));
    
    % Vertical line from track to speed height
    plot3([x_track(i) x_track(i)], ...
          [y_track(i) y_track(i)], ...
          [alt(i) speed_height(i)], ...
          'Color', [col 0.3], 'LineWidth', 0.5);
end

% Plot speed line on top
surface_x = [x_track'; x_track'];
surface_y = [y_track'; y_track'];
surface_z = [alt';     speed_height'];
surface_c = [speed_on_track'; speed_on_track'];

surf(surface_x, surface_y, surface_z, surface_c, ...
     'EdgeColor', 'interp', 'FaceAlpha', 0, ...
     'LineWidth', 2);

colormap(ax1, jet);
cb1 = colorbar;
cb1.Label.String = 'Speed (km/h)';
cb1.Color = 'white';
clim([min(speed_on_track) max(speed_on_track)]);

% Mark start and end
plot3(x_track(1),   y_track(1),   speed_height(1),   ...
      'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x_track(end), y_track(end), speed_height(end), ...
      'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('East (m)',  'Color', 'white');
ylabel('North (m)', 'Color', 'white');
zlabel('Speed height (m)', 'Color', 'white');
title('Track Speed Map — Speed as Height', 'Color', 'white');
grid on;
ax1.Color        = [0.12 0.12 0.15];
ax1.GridColor    = [0.3 0.3 0.3];
ax1.XColor       = 'white';
ax1.YColor       = 'white';
ax1.ZColor       = 'white';
view(45, 30);
legend({'Track altitude', 'Start', 'Finish'}, ...
       'TextColor', 'white', 'Color', [0.2 0.2 0.2]);

%% --- Plot 2: Speed vs distance (top right) ---
ax2 = subplot(2, 2, 2);

% Colour the speed line
hold on;
for i = 1:length(station_data)-1
    c   = (speed_data(i) - min(speed_data)) / ...
          (max(speed_data) - min(speed_data) + eps);
    col = [c, 1-abs(2*c-1), 1-c];
    col = max(0, min(1, col));
    plot(station_data(i:i+1), speed_data(i:i+1), ...
         'Color', col, 'LineWidth', 2);
end

xlabel('Distance (m)', 'Color', 'white');
ylabel('Speed (km/h)', 'Color', 'white');
title('Speed vs Distance', 'Color', 'white');
grid on;
ax2.Color     = [0.12 0.12 0.15];
ax2.GridColor = [0.3 0.3 0.3];
ax2.XColor    = 'white';
ax2.YColor    = 'white';

% Add threshold lines if they exist
yline(25, '--', 'Upper threshold', 'Color', [1 0.4 0.4], ...
      'LabelHorizontalAlignment', 'left');
yline(23, '--', 'Lower threshold', 'Color', [0.4 1 0.4], ...
      'LabelHorizontalAlignment', 'left');

%% --- Plot 3: Elevation profile with speed overlay (bottom right) ---
ax3 = subplot(2, 2, 4);

yyaxis left;
plot(dist_m, alt, 'w-', 'LineWidth', 1.5);
ylabel('Altitude (m)', 'Color', 'white');
ylim([min(alt)-1, max(alt)+1]);

yyaxis right;
plot(dist_m, speed_on_track, 'Color', [0.3 0.8 1], 'LineWidth', 1.5);
ylabel('Speed (km/h)', 'Color', 'white');

xlabel('Distance (m)', 'Color', 'white');
title('Elevation & Speed Profile', 'Color', 'white');
grid on;
ax3.Color     = [0.12 0.12 0.15];
ax3.GridColor = [0.3 0.3 0.3];
ax3.XColor    = 'white';
ax3.YColor    = 'white';
legend({'Altitude', 'Speed'}, 'TextColor', 'white', ...
       'Color', [0.2 0.2 0.2]);

%% =========================================================
%  STEP 5 — Print summary statistics
% =========================================================
fprintf('\n=== Speed Summary ===\n');
fprintf('Mean speed:   %.2f km/h\n', mean(speed_data));
fprintf('Max speed:    %.2f km/h\n', max(speed_data));
fprintf('Min speed:    %.2f km/h\n', min(speed_data));
fprintf('Std dev:      %.2f km/h\n', std(speed_data));

% Find slowest and fastest sections
[~, fast_idx] = max(speed_on_track);
[~, slow_idx] = min(speed_on_track);
fprintf('Fastest point: %.1f km/h at %.0f m\n', ...
        speed_on_track(fast_idx), dist_m(fast_idx));
fprintf('Slowest point: %.1f km/h at %.0f m\n', ...
        speed_on_track(slow_idx), dist_m(slow_idx));
