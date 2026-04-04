data = readmatrix('sem_apme_2025-track_coordinates - Copy.csv'); 
x = data(:, 1);
y = data(:, 2);

% 2. Smooth the data
% Window size '10' can be adjusted; larger values = smoother curve
y_smooth = smoothdata(y, 'movmean', 20); 

% 3. Visualize the result (Optional but recommended)
plot(x, y, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Original');
hold on;
plot(x, y_smooth, 'r', 'LineWidth', 2, 'DisplayName', 'Smoothed');
legend;
title('Curve Smoothing in MATLAB');

% 4. Export to CSV
% Combine x and smoothed y into one matrix
output_matrix = [x, y_smooth];
writematrix(output_matrix, 'smoothed_output.csv');