function [value, isterminal, direction] = ProjectileEvent(~, S, C)

    % [km, km, km] Projectile Position Relative to Ground Station in ENZ
    CG.relPos_ENZ = S(1:3);

    % [km, km, km] Ground Station Position WRT Earth in ECEF
    GS.R_ECEF = (C.Re + C.h_gs) * [cos(C.phi_gs) * cos(C.lambda_gs);
                                    cos(C.phi_gs) * sin(C.lambda_gs);
                                    sin(C.phi_gs)];

    % [] Z-hat Unit Vector in ECEF Coordinates
    Z_ECEF = GS.R_ECEF / norm(GS.R_ECEF);

    % [] E-hat Unit Vector in ECEF Coordinates
    E_ECEF = [-sin(C.lambda_gs);
               cos(C.lambda_gs);
               0];

    % [] N-hat Unit Vector in ECEF Coordinates
    N_ECEF = cross(Z_ECEF, E_ECEF);

    % [] Transformation Matrix from ENZ to ECEF
    ENZ2ECEF = [E_ECEF, N_ECEF, Z_ECEF];

    % [km, km, km] Projectile Position WRT Earth in ECEF
    CG.R_ECEF = GS.R_ECEF + ENZ2ECEF * CG.relPos_ENZ;

    % [km] Magnitude of Projectile Position Vector from Earth's Center
    r_cg = norm(CG.R_ECEF);

    % [km] Altitude of Projectile Above Earth's Mean Radius
    h_cg = r_cg - C.Re;

    % [km] Event Value: Stop when projectile hits ground (altitude = 0)
    value = h_cg;

    % [] Flag to Make This a Terminal Event (1 = Stop Integration When Event Occurs)
    isterminal = 1;

    % [] Direction of Zero Crossing to Detect (-1 = Decreasing, Projectile Returning)
    direction = -1;

end
