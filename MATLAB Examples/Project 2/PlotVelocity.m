function PlotVelocity(t, relVel_ENZ)

    % [m/s] Extract Components and Convert
    E = relVel_ENZ(1, :) * 1000;
    N = relVel_ENZ(2, :) * 1000;
    Z = relVel_ENZ(3, :) * 1000;

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'Velocity', 'NumberTitle', 'Off');

    %% Subplot 1: Eastern Speed
    Axes1 = subplot(3, 1, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('East', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('Speed (m/s)', 'FontSize', 12, 'Parent', Axes1);
    plot(t, E, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Northern Speed
    Axes2 = subplot(3, 1, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('North', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('Speed (m/s)', 'FontSize', 12, 'Parent', Axes2);
    plot(t, N, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Zenith Speed
    Axes3 = subplot(3, 1, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Zenith', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('Speed (m/s)', 'FontSize', 12, 'Parent', Axes3);
    plot(t, Z, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    % Save Figure
    saveas(Window, 'Velocity.png');

end
