function PlotRollPitchYaw(t, euler_angles)

    % [deg] Convert from radians to degrees
    roll = rad2deg(euler_angles(1, :));
    pitch = rad2deg(euler_angles(2, :));
    yaw = rad2deg(euler_angles(3, :));

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'RollPitchYaw', 'NumberTitle', 'Off');

    %% Subplot 1: Roll
    Axes1 = subplot(3, 1, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Roll', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('Roll (°)', 'FontSize', 12, 'Parent', Axes1);
    plot(t, roll, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Pitch
    Axes2 = subplot(3, 1, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Pitch', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('Pitch (°)', 'FontSize', 12, 'Parent', Axes2);
    plot(t, pitch, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Yaw
    Axes3 = subplot(3, 1, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Yaw', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('Yaw (°)', 'FontSize', 12, 'Parent', Axes3);
    plot(t, yaw, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    % Save Figure
    saveas(Window, 'RollPitchYaw.png');

end
