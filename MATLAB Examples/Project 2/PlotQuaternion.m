function PlotQuaternion(t, quaternion)

    % [] Extract quaternion components
    q1 = quaternion(1, :);
    q2 = quaternion(2, :);
    q3 = quaternion(3, :);
    q4 = quaternion(4, :);

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'Quaternion', 'NumberTitle', 'Off');

    %% Subplot 1: Q1
    Axes1 = subplot(2, 2, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Q_1', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('Q_1', 'FontSize', 12, 'Parent', Axes1);
    plot(t, q1, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Q2
    Axes2 = subplot(2, 2, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Q_2', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('Q_2', 'FontSize', 12, 'Parent', Axes2);
    plot(t, q2, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Q3
    Axes3 = subplot(2, 2, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Q_3', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('Q_3', 'FontSize', 12, 'Parent', Axes3);
    plot(t, q3, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    %% Subplot 4: Q4
    Axes4 = subplot(2, 2, 4, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Q_4', 'FontSize', 16, 'Parent', Axes4);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes4);
    ylabel('Q_4', 'FontSize', 12, 'Parent', Axes4);
    plot(t, q4, 'k-', 'LineWidth', 2, 'Parent', Axes4);

    % Save Figure
    saveas(Window, 'Quaternion.png');

end
