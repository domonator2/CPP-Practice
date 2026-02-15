function dSdt = ProjectileEOM(~, S, C)

% [km, km, km] Projectile Position Relative to Ground Station in ENZ
CG.relPos_ENZ = S(1:3);

% [km/s, km/s, km/s] Projectile Velocity Relative to Ground Station in ENZ
CG.relVel_ENZ = S(4:6);

% [rad/s] Angular Velocity in Body Frame
omega_body = S(7:9);

% [] Quaternion Components (q1, q2, q3, q4)
q = S(10:13);

% [km, km, km] Ground Station Position WRT Earth in ECEF
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

% [] Transformation Matrix from ECEF to ENZ
ECEF2ENZ = ENZ2ECEF';

% [km, km, km] Projectile Position WRT Earth in ECEF
CG.R_ECEF = GS.R_ECEF + ENZ2ECEF * CG.relPos_ENZ;

% [km] Magnitude of Projectile Position Vector from Earth's Center
r_cg = norm(CG.R_ECEF);

% [km] Altitude of Projectile Above Earth's Mean Radius
h_cg = r_cg - C.Re;

% [m] Convert Altitude to Meters for Standard Atmosphere Model
h_cg_m = h_cg * 1000;

% [kg/m^3] Atmospheric Density from Standard Atmosphere Model
[~, ~, rho] = StandardAtmosphere(h_cg_m);

% [] Local NED Frame at Projectile's Current Position
% [] Z-hat Unit Vector at Projectile Position (radial outward)
Z_ECEF_CG = CG.R_ECEF / norm(CG.R_ECEF);

% [rad] Longitude of Projectile
lambda_CG = atan2(CG.R_ECEF(2), CG.R_ECEF(1));

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

% [] Rotation Matrix from NED to Body using Quaternions
% DCM from quaternion
q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);

NED2Body = [q1^2 + q2^2 - q3^2 - q4^2,  2*(q2*q3 - q1*q4),          2*(q2*q4 + q1*q3);
    2*(q2*q3 + q1*q4),          q1^2 - q2^2 + q3^2 - q4^2,  2*(q3*q4 - q1*q2);
    2*(q2*q4 - q1*q3),          2*(q3*q4 + q1*q2),          q1^2 - q2^2 - q3^2 + q4^2];

% [] Rotation Matrix from Body to NED
Body2NED = NED2Body';

% [km/s] Velocity in ENZ Frame (relative to ground)
v_ENZ = CG.relVel_ENZ;

% [km/s] Velocity relative to Earth in ECEF
v_rel_ECEF = ENZ2ECEF * v_ENZ;

% [km/s] Atmospheric velocity due to Earth's rotation at projectile position
% Atmosphere rotates with Earth: v_atm = omega x R
v_atmosphere_ECEF = cross(C.We, CG.R_ECEF);

% [km/s] Velocity relative to air mass (for aerodynamic calculations)
v_air_rel_ECEF = v_rel_ECEF - v_atmosphere_ECEF;

% [m/s] Convert air-relative velocity to meters and transform to body frame
v_air_rel_ECEF_m = v_air_rel_ECEF * 1000;
v_body_m = NED2Body * ECEF2NED * v_air_rel_ECEF_m;

% [m/s] Velocity Magnitude (relative to air)
v_mag_m = norm(v_body_m);

% Forces Calculation (in Newtons)
% [N] Drag Force in Body Frame (opposes velocity)
if v_mag_m > 0
    Fd_body = -0.5 * C.cd * rho * C.ar * v_mag_m * v_body_m;
else
    Fd_body = [0; 0; 0];
end

% [N] Magnus Force in Body Frame
if v_mag_m > 0 && norm(omega_body) > 0
    cross_prod = cross(omega_body, v_body_m);
    cross_mag = norm(cross_prod);
    if cross_mag > 1e-12
        Fm_body = 0.5 * C.cl * rho * C.ar * v_mag_m^2 * (cross_prod / cross_mag);
    else
        Fm_body = [0; 0; 0];
    end
else
    Fm_body = [0; 0; 0];
end

% [N] Total Aerodynamic Force in Body Frame
F_aero_body = Fd_body + Fm_body;

% [N] Transform to ENZ Frame (Full Transformation: Body -> NED -> ECEF -> ENZ)
F_aero_ENZ = ECEF2ENZ * NED2ECEF * Body2NED * F_aero_body;

% [m/s^2] Aerodynamic Acceleration in ENZ
a_aero_ENZ_m = F_aero_ENZ / C.mcg;

% [km/s^2] Convert to km/s^2
a_aero_ENZ = a_aero_ENZ_m / 1000;

% [km/s^2] Gravitational Acceleration in ECEF (toward Earth center)
g_ECEF = -C.Gm * CG.R_ECEF / r_cg^3;

% [km/s^2] Gravitational Acceleration in ENZ
g_ENZ = ECEF2ENZ * g_ECEF;

% [km/s^2] Total Acceleration in ENZ
% [km/s] v_rel_ECEF already calculated above for aerodynamic forces

% [km/s^2] Coriolis Acceleration in ECEF
a_cor_ECEF = 2 * cross(C.We, v_rel_ECEF);

% [km/s^2] Centrifugal Acceleration in ECEF
a_cen_ECEF = cross(C.We, cross(C.We, CG.R_ECEF));

% [km/s^2] Total Kinematic Acceleration Correction in ENZ
a_kinematic_ENZ = ECEF2ENZ * (a_cor_ECEF + a_cen_ECEF);

% [km/s^2] Total Acceleration in ENZ
a_ENZ = a_aero_ENZ + g_ENZ - a_kinematic_ENZ;

% Moments Calculation
% [N*m] Moment due to Magnus Force
M_magnus_body = cross(C.R_CP_CG_B, Fm_body);

% [N*m] Moment due to Drag Force
M_drag_body = cross(C.R_CP_CG_B, Fd_body);

% [N*m] Total Moment in Body Frame
M_total_body = M_magnus_body + M_drag_body;

% [rad/s^2] Angular Acceleration using Euler's Equations
% I * omega_dot + omega x (I * omega) = M
I_omega = C.I_body * omega_body;
omega_cross_I_omega = cross(omega_body, I_omega);
omega_dot = C.I_body \ (M_total_body - omega_cross_I_omega);

% [] Quaternion Derivative
% q_dot = 0.5 * Omega * q
Omega = [    0,        -omega_body(1), -omega_body(2), -omega_body(3);
    omega_body(1),      0,          omega_body(3), -omega_body(2);
    omega_body(2), -omega_body(3),      0,          omega_body(1);
    omega_body(3),  omega_body(2), -omega_body(1),      0        ];

q_dot = 0.5 * Omega * q;

% [] Initialize State Derivative Vector
dSdt = zeros(13, 1);

% [km/s] The Derivative of Position is Velocity
dSdt(1:3) = CG.relVel_ENZ;

% [km/s^2] The Derivative of Velocity is Acceleration
dSdt(4:6) = a_ENZ;

% [rad/s^2] The Derivative of Angular Velocity
dSdt(7:9) = omega_dot;

% [] The Derivative of Quaternion
dSdt(10:13) = q_dot;

end
