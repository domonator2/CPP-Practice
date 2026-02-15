function PlotSpeeds(t, relVel_ENZ, DynamicPressure)

    % [m/s] Relative Speed (total velocity magnitude)
    relativeSpeed = sqrt(relVel_ENZ(1,:).^2 + relVel_ENZ(2,:).^2 + relVel_ENZ(3,:).^2) * 1000;

    % [m/s] Ground Speed (horizontal velocity magnitude)
    groundSpeed = sqrt(relVel_ENZ(1,:).^2 + relVel_ENZ(2,:).^2) * 1000;

    % [kPa] Dynamic Pressure
    dynPressure_kPa = DynamicPressure / 1000;

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'Speed', 'NumberTitle', 'Off');

    %% Subplot 1: Relative Speed
    Axes1 = subplot(3, 1, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Relative', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('Speed (m/s)', 'FontSize', 12, 'Parent', Axes1);
    plot(t, relativeSpeed, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Ground Speed
    Axes2 = subplot(3, 1, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Ground', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('Speed (m/s)', 'FontSize', 12, 'Parent', Axes2);
    plot(t, groundSpeed, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Dynamic Pressure
    Axes3 = subplot(3, 1, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Dynamic', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('Pressure (kPa)', 'FontSize', 12, 'Parent', Axes3);
    plot(t, dynPressure_kPa, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    % Save Figure
    saveas(Window, 'Speed.png');

end
