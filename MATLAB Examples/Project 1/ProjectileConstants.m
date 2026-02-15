function C = ProjectileConstants

    % [yyyy, MM, dd, hh, mm, ss] Coordinated Universal Time
    C.UTC = [2025 12 13 14 15 16];

    % [rad] Ground Station Latitude
    C.phi_gs = deg2rad(dms2degrees([32 44 52]));

    % [rad] Ground Station Longitude
    C.lambda_gs = -deg2rad(dms2degrees([97 5 34]));

    % [km] Ground Station Altitude Above Mean Equator
    C.h_gs = 0.185;

    % [km/s] Initial Speed
    C.v0 = 0.827;

    % [deg] Initial Azimuth
    C.azimuth0 = 107;

    % [deg] Initial Elevation
    C.elevation0 = 60;

    % [kg] Projectile Mass
    C.mcg = 35;

    % [] Drag Coefficient
    C.cd = 0.82;

    % [km^2] Profile Area
    C.ar = (6.00225e-3) * pi * 1e-6;

    % [km/s^2] Acceleration Due to Gravity at Sea Level
    C.g0 = 0.00980665;

    % [km] Mean Equatorial Radius of the Earth
    C.Re = 6378.137;

    % [km^3/s^2] Gravitational Parameter of the Earth
    C.Gm = 3.986004418e5;

    % [rad/s] Rotational Speed of the Earth
    C.we = 2 * pi / 86164.1;

    % [rad/s, rad/s, rad/s] Rotational Velocity of the Earth in ECEF
    C.We = C.we * [0; 0; 1];

end
