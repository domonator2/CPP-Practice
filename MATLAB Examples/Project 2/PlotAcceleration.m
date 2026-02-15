function PlotAcceleration(t, A_body, A_ENZ, C)

% [g's] Convert accelerations to g's
ax_body = (A_body(1, :) * 1000) / C.g0;
ay_body = (A_body(2, :) * 1000) / C.g0;
az_body = (A_body(3, :) * 1000) / C.g0;

% [g's] Total inertial acceleration magnitude
a_total = sqrt(A_ENZ(1,:).^2 + A_ENZ(2,:).^2 + A_ENZ(3,:).^2);
a_total_gs = (a_total * 1000) / C.g0;

% [] Create Figure
Window = figure('Color', 'w', 'Name', 'Acceleration', 'NumberTitle', 'Off');

%% Subplot 1: Body-X Acceleration
Axes1 = subplot(2, 2, 1, ...
        'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
        'NextPlot', 'Add', 'Parent', Window, ...
        'XGrid', 'on', 'Ygrid', 'on');

title('Body-X', 'FontSize', 16, 'Parent', Axes1);
xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
ylabel('Acceleration (g''s)', 'FontSize', 12, 'Parent', Axes1);
plot(t, ax_body, 'k-', 'LineWidth', 2, 'Parent', Axes1);

%% Subplot 2: Body-Y Acceleration
Axes2 = subplot(2, 2, 2, ...
        'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
        'NextPlot', 'Add', 'Parent', Window, ...
        'XGrid', 'on', 'Ygrid', 'on');

title('Body-Y', 'FontSize', 16, 'Parent', Axes2);
xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
ylabel('Acceleration (g''s)', 'FontSize', 12, 'Parent', Axes2);
plot(t, ay_body, 'k-', 'LineWidth', 2, 'Parent', Axes2);

%% Subplot 3: Body-Z Acceleration
Axes3 = subplot(2, 2, 3, ...
        'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
        'NextPlot', 'Add', 'Parent', Window, ...
        'XGrid', 'on', 'Ygrid', 'on');

title('Body-Z', 'FontSize', 16, 'Parent', Axes3);
xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
ylabel('Acceleration (g''s)', 'FontSize', 12, 'Parent', Axes3);
plot(t, az_body, 'k-', 'LineWidth', 2, 'Parent', Axes3);

%% Subplot 4: Inertial Acceleration
Axes4 = subplot(2, 2, 4, ...
        'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
        'NextPlot', 'Add', 'Parent', Window, ...
        'XGrid', 'on', 'Ygrid', 'on');

title('Inertial', 'FontSize', 16, 'Parent', Axes4);
xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes4);
ylabel('Acceleration (g''s)', 'FontSize', 12, 'Parent', Axes4);
plot(t, a_total_gs, 'k-', 'LineWidth', 2, 'Parent', Axes4);

% Save Figure
saveas(Window, 'Acceleration.png');

end
