%%
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

close all;
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

% [] Initial Quaternion from Euler Angles (Roll, Pitch, Yaw)
% Pitch angle from elevation, Yaw from azimuth, Roll from initial roll
phi0 = deg2rad(C.roll0);
theta0 = deg2rad(C.elevation0);
psi0 = deg2rad(C.azimuth0);

% [] Initial Rotation Matrix from NED to Body (1-2-3 sequence: Rx * Ry * Rz)
Rx_phi = [1, 0,          0;
    0, cos(phi0), -sin(phi0);
    0, sin(phi0),  cos(phi0)];

Ry_theta = [cos(theta0), 0, sin(theta0);
    0,           1, 0;
    -sin(theta0), 0, cos(theta0)];

Rz_psi = [cos(psi0), -sin(psi0), 0;
    sin(psi0),  cos(psi0), 0;
    0,          0,         1];

% [] Initial NED2B Matrix
NED2B0 = Rx_phi * Ry_theta * Rz_psi;

q0_0 = 0.5 * sqrt(trace(NED2B0) + 1);
q1_0 = (NED2B0(3,2) - NED2B0(2,3)) / (4 * q0_0);
q2_0 = (NED2B0(1,3) - NED2B0(3,1)) / (4 * q0_0);
q3_0 = (NED2B0(2,1) - NED2B0(1,2)) / (4 * q0_0);

% [km, km, km, km/s, km/s, km/s, rad/s, rad/s, rad/s, [], [], [], []] Initial State Vector
So = [0; 0; 0; VE0; VN0; VZ0; C.omega0_body; q0_0; q1_0; q2_0; q3_0];

% [s] Modeling Time Span
to = [0, 200];

% [] ODE Options with High Accuracy and Event Detection
% State vector: [pos(3), vel(3), omega(3), quat(4)]
% Use component-specific AbsTol with smaller values for omega to allow integration from zero
AbsTol_vec = [1e-8, 1e-8, 1e-8, ...     % position (km)
    1e-11, 1e-11, 1e-11, ...  % velocity (km/s)
    1e-12, 1e-12, 1e-12, ...  % omega (rad/s) - balance between accuracy and stability
    1e-12, 1e-12, 1e-12, 1e-12]; % quaternion
Options = odeset('RelTol', 1e-8, 'AbsTol', AbsTol_vec, 'Events', @(t,S)ProjectileEvent(t,S,C));

%% Numerical Integration

% [s, km, km/s, [], rad/s] Numerically Integrates the Projectile Equations of Motion
[t, S] = ode15s(@(t,S)ProjectileEOM(t,S,C), to, So, Options);

%% Post-Processing

% [s] Transpose of the Time Vector
t = transpose(t);

% [km, km/s, [], rad/s] Transpose of the State Vector
S = transpose(S);

% [km, km, km] Projectile Position Relative to Ground Station in ENZ Coordinates
CG.relPos_ENZ = S(1:3, :);

% [km/s, km/s, km/s] Projectile Velocity Relative to Ground Station in ENZ Coordinates
CG.relVel_ENZ = S(4:6, :);

% [rad/s] Angular Velocity in Body Frame
CG.omega_body = S(7:9, :);

% [] Quaternion Components
CG.quaternion = S(10:13, :);

% [] Number of Time Steps
numPoints = size(CG.relPos_ENZ, 2);

% [km, km, km] Preallocate Arrays
CG.R_ECEF = zeros(3, numPoints);
CG.A_ENZ = zeros(3, numPoints);
CG.A_body = zeros(3, numPoints);
CG.euler_angles = zeros(3, numPoints);
CG.euler_rates = zeros(3, numPoints);

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

    % [km/s] Calculate air-relative velocity for dynamic pressure
    % (Need to calculate this before the full transformation below)
    v_rel_ECEF_temp = ENZ2ECEF * CG.relVel_ENZ(:, i);
    v_atmosphere_ECEF_temp = cross(C.We, CG.R_ECEF(:, i));
    v_air_rel_ECEF_temp = v_rel_ECEF_temp - v_atmosphere_ECEF_temp;

    % [km/s] Air-relative velocity magnitude
    v_mag_air = norm(v_air_rel_ECEF_temp);

    % [m/s] Convert to m/s for Force and Dynamic Pressure Calculations
    v_mag_air_m = v_mag_air * 1000;

    % [Pa] Dynamic Pressure (using air-relative velocity)
    DynamicPressure(i) = 0.5 * rho * v_mag_air_m^2;

    % [] Local NED Frame at Projectile's Current Position
    % [] Z-hat Unit Vector at Projectile Position (radial outward)
    Z_ECEF_CG = CG.R_ECEF(:, i) / norm(CG.R_ECEF(:, i));

    % [rad] Longitude of Projectile
    lambda_CG = atan2(CG.R_ECEF(2, i), CG.R_ECEF(1, i));

    % [] E-hat Unit Vector at Projectile Position
    E_ECEF_CG = [-sin(lambda_CG); cos(lambda_CG); 0];

    % [] N-hat Unit Vector at Projectile Position
    N_ECEF_CG = cross(Z_ECEF_CG, E_ECEF_CG);

    % [] D-hat Unit Vector at Projectile Position (down = -zenith)
    D_ECEF_CG = -Z_ECEF_CG;

    % [] Transformation Matrix from ECEF to NED at Projectile Position
    ECEF2NED = [N_ECEF_CG, E_ECEF_CG, D_ECEF_CG]';

    % [] Transformation Matrix from NED to ECEF at Projectile Position
    NED2ECEF = ECEF2NED';

    % [] Extract Quaternion and normalize it
    q = CG.quaternion(:, i);
    q = q / norm(q);  % Normalize quaternion to maintain unit length
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);

    % [] Rotation Matrix from NED to Body
    NED2Body = [q1^2 + q2^2 - q3^2 - q4^2,  2*(q2*q3 - q1*q4),          2*(q2*q4 + q1*q3);
        2*(q2*q3 + q1*q4),          q1^2 - q2^2 + q3^2 - q4^2,  2*(q3*q4 - q1*q2);
        2*(q2*q4 - q1*q3),          2*(q3*q4 + q1*q2),          q1^2 - q2^2 - q3^2 + q4^2];

    % [] Rotation Matrix from Body to NED
    Body2NED = NED2Body';

    % [] Transformation Matrix from ECEF to ENZ
    ECEF2ENZ = ENZ2ECEF';

    % [km/s] Calculate air-relative velocity for aerodynamic forces
    % Velocity relative to Earth in ECEF
    v_rel_ECEF = ENZ2ECEF * CG.relVel_ENZ(:, i);

    % [km/s] Atmospheric velocity due to Earth's rotation at projectile position
    v_atmosphere_ECEF = cross(C.We, CG.R_ECEF(:, i));

    % [km/s] Velocity relative to air mass
    v_air_rel_ECEF = v_rel_ECEF - v_atmosphere_ECEF;

    % [m/s] Convert to body frame for aerodynamic calculations
    v_air_rel_ECEF_m = v_air_rel_ECEF * 1000;
    v_body_m = NED2Body * ECEF2NED * v_air_rel_ECEF_m;

    % [rad/s] Angular Velocity
    omega_body = CG.omega_body(:, i);

    % [m/s] Air-relative velocity magnitude in body frame
    v_mag_body_m = norm(v_body_m);

    % [N] Aerodynamic Forces (in Newtons, using meters)
    if v_mag_body_m > 0
        Fd_body = -0.5 * C.cd * rho * C.ar * v_mag_body_m * v_body_m;
    else
        Fd_body = [0; 0; 0];
    end

    if v_mag_body_m > 0 && norm(omega_body) > 0
        cross_prod = cross(omega_body, v_body_m);
        cross_mag = norm(cross_prod);
        if cross_mag > 1e-12
            Fm_body = 0.5 * C.cl * rho * C.ar * v_mag_body_m^2 * (cross_prod / cross_mag);
        else
            Fm_body = [0; 0; 0];
        end
    else
        Fm_body = [0; 0; 0];
    end

    % [N] Total Aerodynamic Force
    F_aero_body = Fd_body + Fm_body;
    F_aero_ENZ = ECEF2ENZ * NED2ECEF * Body2NED * F_aero_body;

    % [m/s^2] Aerodynamic Acceleration in ENZ
    a_aero_ENZ_m = F_aero_ENZ / C.mcg;

    % [km/s^2] Convert to km/s^2
    a_aero_ENZ = a_aero_ENZ_m / 1000;

    % [km/s^2] Gravitational Acceleration in ECEF
    g_ECEF = -C.Gm * CG.R_ECEF(:, i) / r_cg^3;

    % [km/s^2] Gravitational Acceleration in ENZ
    g_ENZ = ECEF2ENZ * g_ECEF;

    % [km/s^2] Total Acceleration in ENZ
    % [km/s] Velocity Relative to Earth in ECEF (already calculated above)
    % v_rel_ECEF was already computed for aerodynamic calculations

    % [km/s^2] Coriolis Acceleration in ECEF
    a_cor_ECEF = 2 * cross(C.We, v_rel_ECEF);

    % [km/s^2] Centrifugal Acceleration in ECEF
    a_cen_ECEF = cross(C.We, cross(C.We, CG.R_ECEF(:, i)));

    % [km/s^2] Total Kinematic Acceleration Correction in ENZ
    a_kinematic_ENZ = ECEF2ENZ * (a_cor_ECEF + a_cen_ECEF);

    % [km/s^2] Total Acceleration in ENZ
    CG.A_ENZ(:, i) = a_aero_ENZ + g_ENZ - a_kinematic_ENZ;

    % [km/s^2] Total Acceleration in Body Frame
    CG.A_body(:, i) = NED2Body * ECEF2NED * ENZ2ECEF * CG.A_ENZ(:, i);

    % [rad] Extract Euler Angles from Quaternion (1-2-3 sequence: roll-pitch-yaw)
    % q1=q0(scalar), q2=qx, q3=qy, q4=qz
    % Pitch (theta) - rotation about Y (extracted first in 1-2-3)
    CG.euler_angles(2, i) = asin(2*(q2*q4 + q1*q3));
    % Roll (phi) - rotation about X
    CG.euler_angles(1, i) = atan2(-2*(q3*q4 - q1*q2), 1 - 2*(q2^2 + q3^2));
    % Yaw (psi) - rotation about Z
    CG.euler_angles(3, i) = atan2(-2*(q2*q3 - q1*q4), 1 - 2*(q3^2 + q4^2));

    % [rad/s] Calculate Euler Angle Rates from Body Angular Velocities
    phi = CG.euler_angles(1, i);
    theta = CG.euler_angles(2, i);

    % Transformation matrix from body rates to Euler rates
    euler_rate_matrix = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi),            -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    % [rad/s] Euler angle rates
    CG.euler_rates(:, i) = euler_rate_matrix * omega_body;

end

%% Plot Results

PlotDisplacements(t, CG.relPos_ENZ, GS.R_ECEF, CG.R_ECEF, C);
% [] Plot 1: Displacement - Altitude, Linear Range, Haversine Range

PlotSpeeds(t, CG.relVel_ENZ, DynamicPressure);
% [] Plot 2: Speed - Relative Speed, Ground Speed, Dynamic Pressure

PlotPosition(t, CG.relPos_ENZ);
% [] Plot 3: Position - East, North, Zenith Displacements

PlotVelocity(t, CG.relVel_ENZ);
% [] Plot 4: Velocity - East, North, Zenith Speeds

PlotAcceleration(t, CG.A_body, CG.A_ENZ, C);
% [] Plot 5: Acceleration - Body X, Y, Z, and Total Inertial

PlotAzimuthElevation(t, CG.relPos_ENZ);
% [] Plot 6: AzimuthElevation - Azimuth and Elevation Angles

PlotRollPitchYaw(t, CG.euler_angles);
% [] Plot 7: RollPitchYaw - Roll, Pitch, Yaw Angles

PlotAngularRates(t, CG.omega_body);
% [] Plot 8: AngularRates - Body-X, Body-Y, Body-Z, Total

PlotQuaternion(t, CG.quaternion);
% [] Plot 9: Quaternion - Q1, Q2, Q3, Q4

%% Print Simulation Time

SimulationTime = toc;

% [s] Stops the program timer.

SimulationTimeString = 'Simulation Complete! (%0.3f seconds)\n';
% [] Formatted String

fprintf(SimulationTimeString, SimulationTime);
% [] Prints the simulation time on the command window.
