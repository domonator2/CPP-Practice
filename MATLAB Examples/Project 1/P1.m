tic;
% [s] Start timer

clc;
% [] Clears the command window

clear;
% [] Clears the variable workspace

format('Compact');
% [] Formats the command window output to single-spaced output.

format('longG');
% [] Formats the command window output to print 16 digits for double-precision variables.

close('All');
% [] Closes all figures.

%% Load Constants

C = ProjectileConstants;
% [] Loads the parameters of the simulation.

%% Initial Conditions

% [km/s] Initial Velocity in East Direction
VE0 = C.v0 * sind(C.azimuth0) * cosd(C.elevation0);

% [km/s] Initial Velocity in North Direction
VN0 = C.v0 * cosd(C.azimuth0) * cosd(C.elevation0);

% [km/s] Initial Velocity in Zenith Direction
VZ0 = C.v0 * sind(C.elevation0);

% [km, km, km, km/s, km/s, km/s] Initial State Vector in ENZ Coordinates
So = [0; 0; 0; VE0; VN0; VZ0];

% [s] Modeling Time Span
to = [0, 100];

% [] ODE Options with High Accuracy and Event Detection
Options = odeset('RelTol', 1e-10, 'Events', @(t,S)ProjectileEvent(t,S,C));

%% Numerical Integration

% [s, km, km/s] Numerically Integrates the Projectile Equations of Motion
[t, S] = ode45(@(t,S)ProjectileEOM(t,S,C), to, So, Options);

%% Post-Processing

% [s] Transpose of the Time Vector
t = transpose(t);

% [km, km/s] Transpose of the State Vector
S = transpose(S);

% [km, km, km] Projectile Position Relative to Ground Station in ENZ Coordinates
CG.relPos_ENZ = S(1:3, :);

% [km/s, km/s, km/s] Projectile Velocity Relative to Ground Station in ENZ Coordinates
CG.relVel_ENZ = S(4:6, :);

% [] Number of Time Steps
numPoints = size(CG.relPos_ENZ, 2);

% [km, km, km] Preallocate Array for Projectile Position in ECEF
CG.R_ECEF = zeros(3, numPoints);

% [km/s^2, km/s^2, km/s^2] Preallocate Array for Projectile Acceleration in ENZ
CG.A_ENZ = zeros(3, numPoints);

% [Pa] Preallocate Array for Dynamic Pressure
DynamicPressure = zeros(1, numPoints);

% [km, km, km] Ground Station Position Vector Relative to Earth's Center in ECEF
GS.R_ECEF = (C.Re + C.h_gs) * [cos(C.phi_gs) * cos(C.lambda_gs);...
                                cos(C.phi_gs) * sin(C.lambda_gs);...
                                sin(C.phi_gs)];

% [] Z-hat Unit Vector in ECEF Coordinates
Z_ECEF = GS.R_ECEF / norm(GS.R_ECEF);

% [] E-hat Unit Vector in ECEF Coordinates
E_ECEF = [-sin(C.lambda_gs);...
           cos(C.lambda_gs);...
           0];

% [] N-hat Unit Vector in ECEF Coordinates
N_ECEF = cross(Z_ECEF, E_ECEF);

% [] Transformation Matrix from ENZ to ECEF
ENZ2ECEF = [E_ECEF, N_ECEF, Z_ECEF];

% [] Loop Through Each Time Step to Calculate Derived Quantities
for i = 1:numPoints

    % [km, km, km] Projectile Position WRT Earth in ECEF
    CG.R_ECEF(:, i) = GS.R_ECEF + ENZ2ECEF * CG.relPos_ENZ(:, i);

    % [km] Magnitude of Projectile Position Vector from Earth's Center
    r_cg = norm(CG.R_ECEF(:, i));

    % [km] Altitude of Projectile Above Earth's Mean Radius
    h_cg = r_cg - C.Re;

    % [m] Convert Altitude to Meters for Standard Atmosphere Model
    h_cg_m = h_cg * 1000;

    % [kg/m^3] Atmospheric Density
    [~, ~, rho] = StandardAtmosphere(h_cg_m);

    % [kg/km^3] Convert Density to kg/km^3
    rho_km = rho * 1e9;

    % [km/s] Velocity Magnitude
    v_mag = norm(CG.relVel_ENZ(:, i));

    % [m/s] Convert to m/s for Dynamic Pressure Calculation
    v_mag_ms = v_mag * 1000;

    % [Pa] Dynamic Pressure
    DynamicPressure(i) = 0.5 * rho * v_mag_ms^2;

    % [km*kg/s^2] Aerodynamic Drag Force in ENZ
    if v_mag > 0
        Fd_ENZ = -0.5 * C.cd * rho_km * C.ar * v_mag * CG.relVel_ENZ(:, i);
    else
        Fd_ENZ = [0; 0; 0];
    end

    % [km/s^2] Gravitational Acceleration in ECEF (toward Earth center)
    g_ECEF = -C.Gm * CG.R_ECEF(:, i) / r_cg^3;

    % [] Transformation Matrix from ECEF to ENZ
    ECEF2ENZ = ENZ2ECEF';

    % [km/s^2] Gravitational Acceleration in ENZ
    g_ENZ = ECEF2ENZ * g_ECEF;

    % [km/s^2] Total Acceleration in ENZ
    CG.A_ENZ(:, i) = Fd_ENZ / C.mcg + g_ENZ;

end

%% Plot Results

PlotAltitude(t, CG.R_ECEF, C);
% [] Plots the altitude above mean equator as a function of time.

PlotDisplacements(t, CG.relPos_ENZ);
% [] Plots the ENZ displacements and linear range as functions of time.

PlotSpeeds(t, CG.relVel_ENZ);
% [] Plots the ENZ speeds and total speed as functions of time.

PlotAcceleration(t, CG.A_ENZ, C);
% [] Plots the inertial acceleration as a function of time.

PlotAngles(t, CG.relPos_ENZ, CG.relVel_ENZ);
% [] Plots the azimuth and elevation angles as functions of time.

PlotDynamicPressure(t, DynamicPressure);
% [] Plots the dynamic pressure as a function of time.

%% Print Simulation Time

SimulationTime = toc;

% [s] Stops the program timer.

SimulationTimeString = 'Simulation Complete! (%0.3f seconds)\n';
% [] Formatted String

fprintf(SimulationTimeString, SimulationTime);
% [] Prints the simulation time on the command window.
