function PlotPosition(t, relPos_ENZ)

    % [km] Extract Components
    E = relPos_ENZ(1, :);
    N = relPos_ENZ(2, :);
    Z = relPos_ENZ(3, :);

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'Position', 'NumberTitle', 'Off');

    %% Subplot 1: Eastern Displacement
    Axes1 = subplot(3, 1, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('East', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('Displacement (km)', 'FontSize', 12, 'Parent', Axes1);
    plot(t, E, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Northern Displacement
    Axes2 = subplot(3, 1, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('North', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('Displacement (km)', 'FontSize', 12, 'Parent', Axes2);
    plot(t, N, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Zenith Displacement
    Axes3 = subplot(3, 1, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Zenith', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('Displacement (km)', 'FontSize', 12, 'Parent', Axes3);
    plot(t, Z, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    % Save Figure
    saveas(Window, 'Position.png');

end
