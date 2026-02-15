function PlotDisplacements(t, relPos_ENZ, GS_R_ECEF, CG_R_ECEF, C)

    numPoints = length(t);

    % [km] Altitude Above Mean Equator
    altitude = zeros(1, numPoints);
    for i = 1:numPoints
        rcge = norm(CG_R_ECEF(:, i));
        altitude(i) = rcge - C.Re;
    end

    % [km] Linear Range (Euclidean distance)
    linearRange = sqrt(relPos_ENZ(1,:).^2 + relPos_ENZ(2,:).^2 + relPos_ENZ(3,:).^2);

    % [km] Haversine Range (great circle distance)
    haversineRange = zeros(1, numPoints);
    for i = 1:numPoints
        R_cg = CG_R_ECEF(:, i);
        R_gs = GS_R_ECEF;

        % Unit vectors
        u_cg = R_cg / norm(R_cg);
        u_gs = R_gs / norm(R_gs);

        % Haversine formula (clamp dot product to avoid numerical errors)
        dot_product = dot(u_cg, u_gs);
        dot_product = max(-1, min(1, dot_product));  % Clamp to [-1, 1]
        central_angle = acos(dot_product);
        haversineRange(i) = C.Re * central_angle;
    end

    % [] Create Figure
    Window = figure('Color', 'w', 'Name', 'Displacement', 'NumberTitle', 'Off');

    %% Subplot 1: Altitude
    Axes1 = subplot(3, 1, 1, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Altitude', 'FontSize', 16, 'Parent', Axes1);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes1);
    ylabel('Altitude (km)', 'FontSize', 12, 'Parent', Axes1);
    plot(t, altitude, 'k-', 'LineWidth', 2, 'Parent', Axes1);

    %% Subplot 2: Linear Range
    Axes2 = subplot(3, 1, 2, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Linear', 'FontSize', 16, 'Parent', Axes2);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes2);
    ylabel('Range (km)', 'FontSize', 12, 'Parent', Axes2);
    plot(t, linearRange, 'k-', 'LineWidth', 2, 'Parent', Axes2);

    %% Subplot 3: Haversine Range
    Axes3 = subplot(3, 1, 3, ...
            'FontName', 'Arial', 'FontSize', 8, 'FontWeight', 'bold', ...
            'NextPlot', 'Add', 'Parent', Window, ...
            'XGrid', 'on', 'Ygrid', 'on');

    title('Haversine', 'FontSize', 16, 'Parent', Axes3);
    xlabel('Time (s)', 'FontSize', 12, 'Parent', Axes3);
    ylabel('Range (km)', 'FontSize', 12, 'Parent', Axes3);
    plot(t, haversineRange, 'k-', 'LineWidth', 2, 'Parent', Axes3);

    % Save Figure
    saveas(Window, 'Displacement.png');

end
