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
C.elevation0 = 45;

% [deg] Initial Roll Angle
C.roll0 = 0;

% [kg] Projectile Mass
C.mcg = 35;

% [] Magnus Coefficient
C.cl = 0.173;

% [] Drag Coefficient
C.cd = 0.820;

% [m^2] Reference Area (6006.25π mm^2)
C.ar = 6006.25 * pi * 1e-6;

% [kg*m^2] Moment of Inertia Components
C.Ixx = 0.0630656250;
C.Iyy = 1.3440328125;
C.Izz = 1.3440328125;
C.Ixy = 0.1579431538;
C.Ixz = 0.1579431538;
C.Iyz = 0.1579431538;

% [kg*m^2] Inertia Tensor in Body Frame
C.I_body = [C.Ixx, C.Ixy, C.Ixz;...
    C.Ixy, C.Iyy, C.Iyz;...
    C.Ixz, C.Iyz, C.Izz];

% [m] Position of Center of Pressure Relative to CG in Body Frame
C.R_CP_CG_B = [-.001; 0; 0];

% [rad/s] Initial Angular Velocity
C.omega0_body = [0; 0; 0];

% [m/s^2] Acceleration Due to Gravity at Sea Level
C.g0 = 9.80665;

% [km] Mean Equatorial Radius of the Earth
C.Re = 6378.137;

% [km^3/s^2] Gravitational Parameter of the Earth
C.Gm = 398600.435436;

% [rad/s] Rotational Speed of the Earth
C.we = 2 * pi / 86164.1;

% [rad/s, rad/s, rad/s] Rotational Velocity of the Earth in ECEF
C.We = C.we * [0; 0; 1];

end
