function PlotAzimuthElevation(t, relPos_ENZ)

numPoints = length(t);
azimuth = zeros(1, numPoints);
elevation = zeros(1, numPoints);

for i = 1:numPoints
    % [deg] Azimuth from East and North components (Position)
    azimuth(i) = atan2d(relPos_ENZ(1, i), relPos_ENZ(2, i));

    % [deg] Elevation from position vector
    r_horiz = sqrt(relPos_ENZ(1, i)^2 + relPos_ENZ(2, i)^2);
    elevation(i) = asind(relPos_ENZ(3, i) / norm(relPos_ENZ(:, i)));
end

% [] Create Figure
Window = figure('Color', 'w', 'Name', 'AzimuthElevation', 'NumberTitle', 'Off');

%% Subplot 1: Azimuth
Axes1 = subplot(2, 1, 1, ...
    'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
    'NextPlot', 'Add', 'Parent', Window, ...
    'XGrid', 'on', 'Ygrid', 'on');

title('Azimuth', 'FontSize', 16, 'Parent', Axes1);
xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
ylabel('Azimuth (°)', 'FontSize', 12, 'Parent', Axes1);
plot(t, azimuth, 'k-', 'LineWidth', 2, 'Parent', Axes1);

%% Subplot 2: Elevation
Axes2 = subplot(2, 1, 2, ...
    'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
    'NextPlot', 'Add', 'Parent', Window, ...
    'XGrid', 'on', 'Ygrid', 'on');

title('Elevation', 'FontSize', 16, 'Parent', Axes2);
xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
ylabel('Elevation (°)', 'FontSize', 12, 'Parent', Axes2);
plot(t, elevation, 'k-', 'LineWidth', 2, 'Parent', Axes2);

% Save Figure
saveas(Window, 'AzimuthElevation.png');

end
