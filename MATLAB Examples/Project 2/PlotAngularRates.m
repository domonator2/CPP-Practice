function PlotAngularRates(t, omega_body)

    % [deg/s] Convert from rad/s to deg/s
    omega_x = rad2deg(omega_body(1, :));
    omega_y = rad2deg(omega_body(2, :));
    omega_z = rad2deg(omega_body(3, :));
    omega_total = rad2deg(sqrt(omega_body(1,:).^2 + omega_body(2,:).^2 + omega_body(3,:).^2));

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'AngularRates', 'NumberTitle', 'Off');

    %% Subplot 1: Body-X Angular Rate
    Axes1 = subplot(2, 2, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Body-X', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('\omega_x (°/s)', 'FontSize', 12, 'Parent', Axes1);
    plot(t, omega_x, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Body-Y Angular Rate
    Axes2 = subplot(2, 2, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Body-Y', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('\omega_y (°/s)', 'FontSize', 12, 'Parent', Axes2);
    plot(t, omega_y, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Body-Z Angular Rate
    Axes3 = subplot(2, 2, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Body-Z', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('\omega_z (°/s)', 'FontSize', 12, 'Parent', Axes3);
    plot(t, omega_z, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    %% Subplot 4: Total Angular Rate
    Axes4 = subplot(2, 2, 4, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Total', 'FontSize', 16, 'Parent', Axes4);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes4);
    ylabel('\omega (°/s)', 'FontSize', 12, 'Parent', Axes4);
    plot(t, omega_total, 'k-', 'LineWidth', 2, 'Parent', Axes4);

    % Save Figure
    saveas(Window, 'AngularRates.png');

end
