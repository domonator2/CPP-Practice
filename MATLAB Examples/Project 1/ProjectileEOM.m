function dSdt = ProjectileEOM(~, S, C)

    % [km, km, km] Projectile Position Relative to Ground Station in ENZ
    CG.relPos_ENZ = S(1:3);

    % [km/s, km/s, km/s] Projectile Velocity Relative to Ground Station in ENZ
    CG.relVel_ENZ = S(4:6);

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

    % [kg/km^3] Convert Density to kg/km^3 for Consistent Unit System
    rho = rho * 1e9;

    % [km/s] Velocity Magnitude
    v_mag = norm(CG.relVel_ENZ);

    % [km*kg/s^2] Aerodynamic Drag Force in ENZ (opposes velocity)
    if v_mag > 0
        Fd_ENZ = -0.5 * C.cd * rho * C.ar * v_mag * CG.relVel_ENZ;
    else
        Fd_ENZ = [0; 0; 0];
    end

    % [km/s^2] Gravitational Acceleration in ECEF (toward Earth center)
    g_ECEF = -C.Gm * CG.R_ECEF / r_cg^3;

    % [km/s^2] Gravitational Acceleration in ENZ
    g_ENZ = ECEF2ENZ * g_ECEF;

    % [km/s^2] Total Acceleration in ENZ
    a_ENZ = Fd_ENZ / C.mcg + g_ENZ;

    % [] Initialize State Derivative Vector
    dSdt = zeros(6, 1);

    % [km/s] The Derivative of Position is Velocity
    dSdt(1:3) = CG.relVel_ENZ;

    % [km/s^2] The Derivative of Velocity is Acceleration
    dSdt(4:6) = a_ENZ;

end
