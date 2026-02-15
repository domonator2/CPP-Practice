%Parametric Hypersonic Convergence Model for X-29 Reverse Engineering

% Complete MATLAB Implementation

%matlab
%% MAIN SCRIPT: X29_Hypersonic_Convergence_Platform.m
% Parametric Hypersonic Convergence Analysis for X-29 Reverse Engineering
% Author: Aerospace Systems Analysis
% Date: 2024

clear; clc; close all;

%% ========================================================================
% SECTION 1: INPUT PARAMETERS AND CONFIGURATION
% =========================================================================

% Define parameter ranges
tau_range = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1];
sweep_range = [-45, -30, -15, 0, 15, 30, 45]; % degrees (negative = forward sweep)
cruise_duration_range = [1, 5, 10]; % minutes

% X-29 Reference Configuration (for validation)
X29_Reference = struct(...
    'wing_sweep', -33.73, ... % Forward sweep angle (degrees)
    'wing_area', 17.54, ... % m^2
    'wing_span', 8.29, ... % m
    'length', 16.44, ... % m
    'height', 4.36, ... % m
    'empty_weight', 6260, ... % kg
    'max_weight', 8074, ... % kg
    'max_speed', 1.8, ... % Mach
    'service_ceiling', 16764, ... % m
    'thrust', 71200, ... % N (with afterburner)
    'tau', 0.04 ... % thickness ratio estimate
);

% Flight conditions for analysis
FlightConditions = struct(...
    'altitude', 15000, ... % m
    'mach_cruise', [1.2, 1.5, 1.8, 2.5, 3.0, 5.0], ... % Mach numbers
    'mach_hypersonic', [5.0, 6.0, 7.0, 8.0], ... % Hypersonic regime
    'ambient_temp', 216.65, ... % K (standard atmosphere at altitude)
    'ambient_pressure', 12112, ... % Pa
    'ambient_density', 0.1948 ... % kg/m^3
);

fprintf('========================================\n');
fprintf('X-29 HYPERSONIC CONVERGENCE PLATFORM\n');
fprintf('========================================\n\n');

%% ========================================================================
% SECTION 2: GEOMETRY MODULE
% =========================================================================

function [Geometry] = GeometryModule(tau, sweep_deg, scale_factor)
    % Generates complete aircraft geometry based on X-29 topology
    % Inputs:
    %   tau - thickness ratio (0.01-0.1)
    %   sweep_deg - wing sweep angle (degrees, negative = forward)
    %   scale_factor - scaling relative to X-29 baseline
    
    % Base dimensions (X-29 scaled)
    Geometry.fuselage.length = 16.44 * scale_factor;
    Geometry.fuselage.diameter = 1.45 * scale_factor;
    Geometry.fuselage.fineness = Geometry.fuselage.length / Geometry.fuselage.diameter;
    
    % Wing geometry
    Geometry.wing.sweep = sweep_deg;
    Geometry.wing.sweep_rad = deg2rad(sweep_deg);
    Geometry.wing.span = 8.29 * scale_factor;
    Geometry.wing.root_chord = 3.0 * scale_factor;
    Geometry.wing.tip_chord = 1.2 * scale_factor;
    Geometry.wing.taper_ratio = Geometry.wing.tip_chord / Geometry.wing.root_chord;
    Geometry.wing.area = 17.54 * scale_factor^2;
    Geometry.wing.AR = Geometry.wing.span^2 / Geometry.wing.area;
    Geometry.wing.MAC = (2/3) * Geometry.wing.root_chord * ...
        (1 + Geometry.wing.taper_ratio + Geometry.wing.taper_ratio^2) / ...
        (1 + Geometry.wing.taper_ratio);
    Geometry.wing.tau = tau;
    Geometry.wing.max_thickness = tau * Geometry.wing.MAC;
    
    % Sweep corrections for forward swept wing
    if sweep_deg < 0
        Geometry.wing.effective_AR = Geometry.wing.AR * cos(Geometry.wing.sweep_rad);
        Geometry.wing.structural_factor = 1 + 0.5 * abs(sweep_deg) / 45;
    else
        Geometry.wing.effective_AR = Geometry.wing.AR * cos(Geometry.wing.sweep_rad);
        Geometry.wing.structural_factor = 1 + 0.2 * abs(sweep_deg) / 45;
    end
    
    % Canard geometry (X-29 feature)
    Geometry.canard.area = 2.8 * scale_factor^2;
    Geometry.canard.span = 3.5 * scale_factor;
    Geometry.canard.sweep = 30; % degrees
    Geometry.canard.position = 0.15; % fraction of fuselage length from nose
    
    % Vertical tail
    Geometry.vtail.area = 4.2 * scale_factor^2;
    Geometry.vtail.height = 2.1 * scale_factor;
    Geometry.vtail.sweep = 45; % degrees
    
    % Strake geometry
    Geometry.strake.area = 1.5 * scale_factor^2;
    Geometry.strake.length = 2.5 * scale_factor;
    
    % Wetted areas
    Geometry.wetted.fuselage = pi * Geometry.fuselage.diameter * ...
        Geometry.fuselage.length * 0.85;
    Geometry.wetted.wing = 2.1 * Geometry.wing.area;
    Geometry.wetted.canard = 2.1 * Geometry.canard.area;
    Geometry.wetted.vtail = 2.1 * Geometry.vtail.area;
    Geometry.wetted.total = Geometry.wetted.fuselage + Geometry.wetted.wing + ...
        Geometry.wetted.canard + Geometry.wetted.vtail;
    
    % Reference areas
    Geometry.Sref = Geometry.wing.area;
    Geometry.Swet = Geometry.wetted.total;
    
    % Volume calculations
    Geometry.volume.fuselage = (pi/4) * Geometry.fuselage.diameter^2 * ...
        Geometry.fuselage.length * 0.7;
    Geometry.volume.wing = Geometry.wing.area * Geometry.wing.max_thickness * 0.5;
    Geometry.volume.total = Geometry.volume.fuselage + Geometry.volume.wing;
    
    % Inlet geometry for propulsion
    Geometry.inlet.area = 0.35 * scale_factor^2;
    Geometry.inlet.position = [3.5, 0, -0.3] * scale_factor;
    Geometry.nozzle.area = 0.40 * scale_factor^2;
end

%% ========================================================================
% SECTION 3: AERODYNAMICS MODULE
% =========================================================================

function [Aero] = AerodynamicsModule(Geometry, Mach, altitude, tau)
    % Comprehensive aerodynamic analysis including hypersonic regime
    
    % Atmospheric properties
    [T, P, rho] = StandardAtmosphere(altitude);
    a = sqrt(1.4 * 287 * T); % Speed of sound
    V = Mach * a;
    q = 0.5 * rho * V^2; % Dynamic pressure
    
    Aero.flight.Mach = Mach;
    Aero.flight.altitude = altitude;
    Aero.flight.velocity = V;
    Aero.flight.q = q;
    Aero.flight.Re = rho * V * Geometry.wing.MAC / (1.458e-6 * T^1.5 / (T + 110.4));
    
    % Determine flow regime
    if Mach < 0.8
        regime = 'subsonic';
    elseif Mach < 1.2
        regime = 'transonic';
    elseif Mach < 5.0
        regime = 'supersonic';
    else
        regime = 'hypersonic';
    end
    Aero.regime = regime;
    
    % Prandtl-Glauert / Ackeret corrections
    if Mach < 1.0
        beta = sqrt(1 - Mach^2);
        Aero.compressibility_factor = 1 / beta;
    else
        beta = sqrt(Mach^2 - 1);
        Aero.compressibility_factor = 1 / beta;
    end
    
    % Skin friction coefficient
    Cf = SkinFriction(Aero.flight.Re, Mach);
    
    % Zero-lift drag
    FF_wing = (1 + 0.6/0.3 * tau + 100 * tau^4) * ...
        (1.34 * Mach^0.18 * cos(Geometry.wing.sweep_rad)^0.28);
    FF_fuse = 1 + 60/Geometry.fuselage.fineness^3 + ...
        Geometry.fuselage.fineness/400;
    
    CD0_wing = Cf * FF_wing * Geometry.wetted.wing / Geometry.Sref;
    CD0_fuse = Cf * FF_fuse * Geometry.wetted.fuselage / Geometry.Sref;
    CD0_canard = Cf * 1.1 * Geometry.wetted.canard / Geometry.Sref;
    CD0_vtail = Cf * 1.1 * Geometry.wetted.vtail / Geometry.Sref;
    
    Aero.CD0 = CD0_wing + CD0_fuse + CD0_canard + CD0_vtail;
    
    % Wave drag (supersonic/hypersonic)
    if Mach > 1.0
        Aero.CDwave = WaveDrag(Mach, tau, Geometry.wing.sweep_rad, ...
            Geometry.fuselage.fineness);
    else
        Aero.CDwave = 0;
    end
    
    % Lift curve slope
    Aero.CLa = LiftCurveSlope(Geometry.wing.AR, Mach, Geometry.wing.sweep_rad);
    
    % Induced drag factor
    e = OswaldEfficiency(Geometry.wing.AR, Geometry.wing.sweep_rad, ...
        Geometry.wing.taper_ratio, Mach);
    Aero.K = 1 / (pi * Geometry.wing.AR * e);
    
    % Forward sweep effects
    if Geometry.wing.sweep < 0
        % Enhanced lift at root, potential for tip stall prevention
        Aero.FSW_lift_enhancement = 1 + 0.1 * abs(Geometry.wing.sweep) / 45;
        Aero.CLa = Aero.CLa * Aero.FSW_lift_enhancement;
        % Increased induced drag due to span loading
        Aero.K = Aero.K * (1 + 0.15 * abs(Geometry.wing.sweep) / 45);
    else
        Aero.FSW_lift_enhancement = 1.0;
    end
    
    % Maximum lift coefficient
    Aero.CLmax = MaxLiftCoefficient(Mach, Geometry.wing.sweep_rad, tau);
    
    % Drag polar
    Aero.CD = @(CL) Aero.CD0 + Aero.CDwave + Aero.K * CL^2;
    
    % L/D calculations
    CL_opt = sqrt((Aero.CD0 + Aero.CDwave) / Aero.K);
    Aero.CL_maxLD = CL_opt;
    Aero.LD_max = CL_opt / Aero.CD(CL_opt);
    
    % Hypersonic specific parameters
    if Mach >= 5.0
        [Aero.hypersonic] = HypersonicAero(Mach, tau, Geometry);
    end
    
    % Store coefficients at key conditions
    Aero.CL_cruise = 0.3;
    Aero.CD_cruise = Aero.CD(Aero.CL_cruise);
    Aero.LD_cruise = Aero.CL_cruise / Aero.CD_cruise;
end

function Cf = SkinFriction(Re, Mach)
    % Turbulent flat plate skin friction with compressibility
    Cf_incomp = 0.455 / (log10(Re))^2.58;
    % Compressibility correction (Van Driest II)
    Tw_Taw = 1 + 0.035 * Mach^2;
    Cf = Cf_incomp / Tw_Taw;
end

function CDw = WaveDrag(Mach, tau, sweep_rad, fineness)
    % Wave drag estimation for supersonic/hypersonic flight
    beta = sqrt(Mach^2 - 1);
    
    % Wing wave drag (linearized theory)
    Mn = Mach * cos(sweep_rad); % Normal Mach number
    if Mn > 1
        CDw_wing = 4 * tau^2 / sqrt(Mn^2 - 1);
    else
        CDw_wing = 0;
    end
    
    % Fuselage wave drag (Sears-Haack body reference)
    CDw_fuse = 9 * pi / (2 * fineness^2);
    
    % Total wave drag
    CDw = CDw_wing * 0.7 + CDw_fuse * 0.3;
    
    % Hypersonic correction
    if Mach > 5
        CDw = CDw * (1 + 0.1 * (Mach - 5));
    end
end

function CLa = LiftCurveSlope(AR, Mach, sweep_rad)
    % Lift curve slope calculation
    if Mach < 1
        beta = sqrt(1 - Mach^2);
        kappa = 0.95; % Airfoil efficiency
        CLa = 2 * pi * AR / (2 + sqrt(4 + (AR * beta / kappa)^2 * ...
            (1 + tan(sweep_rad)^2 / beta^2)));
    elseif Mach < 5
        beta = sqrt(Mach^2 - 1);
        CLa = 4 / beta * cos(sweep_rad);
    else
        % Hypersonic Newtonian theory
        CLa = 2 / Mach;
    end
end

function e = OswaldEfficiency(AR, sweep_rad, taper, Mach)
    % Oswald efficiency factor
    e_basic = 1.78 * (1 - 0.045 * AR^0.68) - 0.64;
    
    % Sweep correction
    e_sweep = e_basic * cos(sweep_rad)^0.15;
    
    % Taper correction
    e_taper = e_sweep * (1 - 0.1 * (1 - taper)^2);
    
    % Compressibility correction
    if Mach > 0.7
        e = e_taper * (1 - 0.2 * (Mach - 0.7));
    else
        e = e_taper;
    end
    
    e = max(0.5, min(0.95, e));
end

function CLmax = MaxLiftCoefficient(Mach, sweep_rad, tau)
    % Maximum lift coefficient estimation
    CLmax_basic = 1.5 + 2.5 * tau;
    CLmax_sweep = CLmax_basic * cos(sweep_rad)^1.5;
    
    if Mach > 0.6
        CLmax = CLmax_sweep * (1 - 0.3 * (Mach - 0.6));
    else
        CLmax = CLmax_sweep;
    end
    
    CLmax = max(0.5, CLmax);
end

function [hyper] = HypersonicAero(Mach, tau, Geometry)
    % Hypersonic aerodynamic analysis
    
    % Newtonian pressure coefficient
    hyper.Cp_max = 2 / (1.4 * Mach^2) * ((2.4^2 * Mach^2 / ...
        (4 * 1.4 * Mach^2 - 2 * 0.4))^(1.4/0.4) * ...
        ((1 - 0.4 + 2 * 1.4 * Mach^2) / (1.4 + 1)) - 1);
    
    % Modified Newtonian theory
    hyper.Cp_stag = 1.839; % Stagnation point Cp
    
    % Viscous interaction parameter
    hyper.chi = Mach^3 * sqrt(1.458e-6 / (Geometry.flight.Re * 216.65));
    
    % Entropy layer effects
    hyper.entropy_swallowing = 1 + 0.1 * Mach;
    
    % Real gas effects indicator
    hyper.gamma_effective = 1.4 - 0.05 * (Mach - 5);
    
    % Aerodynamic heating parameter
    hyper.St = 0.332 / sqrt(Geometry.flight.Re) * ...
        (1 + 0.035 * Mach^2)^(-0.5);
end

%% ========================================================================
% SECTION 4: PROPULSION MODULE
% =========================================================================

function [Propulsion] = PropulsionModule(Geometry, Mach, altitude, cruise_duration)
    % Propulsion system analysis for turbofan/ramjet/scramjet
    
    [T, P, rho] = StandardAtmosphere(altitude);
    a = sqrt(1.4 * 287 * T);
    V = Mach * a;
    
    % Determine propulsion type based on Mach
    if Mach < 3.0
        prop_type = 'turbofan';
        [Propulsion] = TurbofanAnalysis(Mach, altitude, Geometry);
    elseif Mach < 5.0
        prop_type = 'ramjet';
        [Propulsion] = RamjetAnalysis(Mach, altitude, Geometry);
    else
        prop_type = 'scramjet';
        [Propulsion] = ScramjetAnalysis(Mach, altitude, Geometry);
    end
    
    Propulsion.type = prop_type;
    Propulsion.cruise_duration = cruise_duration;
    
    % Fuel consumption
    Propulsion.fuel_flow = Propulsion.thrust * Propulsion.TSFC; % kg/s
    Propulsion.fuel_cruise = Propulsion.fuel_flow * cruise_duration * 60; % kg
    
    % Inlet analysis
    Propulsion.inlet = InletAnalysis(Mach, Geometry.inlet.area);
    
    % Nozzle analysis
    Propulsion.nozzle = NozzleAnalysis(Mach, Geometry.nozzle.area, Propulsion);
end

function [Prop] = TurbofanAnalysis(Mach, altitude, Geometry)
    % GE F404 class turbofan analysis (X-29 engine)
    
    [T, P, rho] = StandardAtmosphere(altitude);
    
    % Sea level static thrust (F404-GE-400)
    T_SLS_dry = 48900; % N
    T_SLS_AB = 71200; % N
    
    % Altitude and Mach corrections
    theta = T / 288.15;
    delta = P / 101325;
    
    % Ram recovery
    if Mach < 1
        eta_ram = 1 - 0.075 * (Mach - 0.35)^1.35;
    else
        eta_ram = 1 - 0.075 * Mach^1.35;
    end
    eta_ram = max(0.7, eta_ram);
    
    % Thrust lapse
    if Mach < 0.9
        thrust_ratio = delta * (1 - 0.3 * (1 - Mach));
    else
        thrust_ratio = delta * (1 + 0.2 * (Mach - 0.9)) * eta_ram;
    end
    
    Prop.thrust_dry = T_SLS_dry * thrust_ratio;
    Prop.thrust_AB = T_SLS_AB * thrust_ratio;
    Prop.thrust = Prop.thrust_AB; % Assume AB for performance
    
    % TSFC
    TSFC_SLS = 0.84 / 3600; % 1/s (dry)
    TSFC_AB_SLS = 1.74 / 3600; % 1/s (AB)
    
    Prop.TSFC_dry = TSFC_SLS * sqrt(theta) / delta;
    Prop.TSFC_AB = TSFC_AB_SLS * sqrt(theta) / delta;
    Prop.TSFC = Prop.TSFC_AB;
    
    % Engine parameters
    Prop.BPR = 0.34; % Bypass ratio
    Prop.OPR = 25; % Overall pressure ratio
    Prop.TIT = 1650; % K, turbine inlet temperature
    Prop.mass_flow = 64.4; % kg/s
    Prop.weight = 1036; % kg
    Prop.length = 3.912; % m
    Prop.diameter = 0.889; % m
end

function [Prop] = RamjetAnalysis(Mach, altitude, Geometry)
    % Ramjet propulsion for Mach 3-5
    
    [T, P, rho] = StandardAtmosphere(altitude);
    a = sqrt(1.4 * 287 * T);
    V = Mach * a;
    
    % Inlet total conditions
    T0 = T * (1 + 0.2 * Mach^2);
    P0_ideal = P * (1 + 0.2 * Mach^2)^3.5;
    
    % Inlet efficiency (normal shock at Mach 3+)
    if Mach < 2.5
        eta_inlet = 0.95;
    else
        eta_inlet = 0.95 - 0.05 * (Mach - 2.5);
    end
    P02 = P0_ideal * eta_inlet;
    
    % Combustor
    T04 = 2200; % K max temperature
    eta_b = 0.95;
    Qf = 43e6; % J/kg fuel heating value
    cp = 1004;
    f = (T04 - T0) * cp / (eta_b * Qf - cp * T04);
    
    % Nozzle
    eta_n = 0.97;
    P9_P0 = 1; % Ideally expanded
    T9 = T04 * (P9_P0)^(0.4/1.4);
    V9 = sqrt(2 * cp * eta_n * (T04 - T9));
    
    % Thrust
    A_inlet = Geometry.inlet.area;
    m_dot = rho * V * A_inlet * eta_inlet;
    
    Prop.thrust = m_dot * ((1 + f) * V9 - V);
    Prop.TSFC = f / (Prop.thrust / m_dot);
    Prop.mass_flow = m_dot;
    Prop.weight = 300 * (A_inlet / 0.35)^0.8; % kg estimate
    Prop.ISP = Prop.thrust / (f * m_dot * 9.81);
end

function [Prop] = ScramjetAnalysis(Mach, altitude, Geometry)
    % Scramjet propulsion for Mach 5+
    
    [T, P, rho] = StandardAtmosphere(altitude);
    a = sqrt(1.4 * 287 * T);
    V = Mach * a;
    
    % Inlet with oblique shocks
    theta_ramp = 10 * pi / 180; % Inlet ramp angle
    [M2, P2_P1] = ObliqueShock(Mach, theta_ramp);
    
    % Isolator
    M3 = M2 * 0.95;
    P03 = P * P2_P1 * (1 + 0.2 * M3^2)^3.5;
    T03 = T * (1 + 0.2 * M3^2);
    
    % Combustor (supersonic combustion)
    T04 = min(2500, T03 + 1500); % K
    eta_b = 0.85;
    Qf = 43e6;
    cp = 1100; % Higher for high temps
    f = (T04 - T03) * cp / (eta_b * Qf - cp * T04);
    f = max(0.01, min(0.08, f));
    
    % Nozzle expansion
    eta_n = 0.95;
    V9 = V * sqrt(T04 / T03) * sqrt(eta_n);
    
    % Thrust
    A_inlet = Geometry.inlet.area;
    capture_ratio = 0.8;
    m_dot = rho * V * A_inlet * capture_ratio;
    
    Prop.thrust = m_dot * ((1 + f) * V9 - V);
    Prop.TSFC = f / (Prop.thrust / m_dot);
    Prop.mass_flow = m_dot;
    Prop.weight = 400 * (A_inlet / 0.35)^0.8;
    Prop.ISP = Prop.thrust / (f * m_dot * 9.81);
    
    % Ensure positive thrust
    if Prop.thrust < 0
        Prop.thrust = m_dot * 0.1 * V; % Minimum drag-level thrust
        Prop.TSFC = f / (Prop.thrust / m_dot);
    end
end

function [inlet] = InletAnalysis(Mach, A_inlet)
    % Inlet performance analysis
    
    if Mach < 1.5
        inlet.type = 'pitot';
        inlet.recovery = 1 - 0.01 * Mach^2;
    elseif Mach < 3
        inlet.type = 'external_compression';
        inlet.recovery = 1 - 0.075 * (Mach - 1)^1.35;
    else
        inlet.type = 'mixed_compression';
        inlet.recovery = 0.95 - 0.025 * (Mach - 3);
    end
    
    inlet.area = A_inlet;
    inlet.drag_coeff = 0.1 * (1 - inlet.recovery);
end

function [nozzle] = NozzleAnalysis(Mach, A_exit, Prop)
    % Nozzle performance
    
    nozzle.area_ratio = 1 + 0.5 * Mach;
    nozzle.exit_area = A_exit * nozzle.area_ratio;
    nozzle.efficiency = 0.98 - 0.01 * Mach;
    nozzle.type = 'convergent_divergent';
end

function [M2, P2_P1] = ObliqueShock(M1, theta)
    % Oblique shock relations
    gamma = 1.4;
    
    % Iterative solution for shock angle
    beta_min = asin(1/M1);
    beta_max = pi/2;
    
    for i = 1:50
        beta = (beta_min + beta_max) / 2;
        tan_theta = 2 * cot(beta) * (M1^2 * sin(beta)^2 - 1) / ...
            (M1^2 * (gamma + cos(2*beta)) + 2);
        if tan_theta > tan(theta)
            beta_max = beta;
        else
            beta_min = beta;
        end
    end
    
    % Post-shock Mach number
    Mn1 = M1 * sin(beta);
    Mn2 = sqrt((1 + 0.2 * Mn1^2) / (1.4 * Mn1^2 - 0.2));
    M2 = Mn2 / sin(beta - theta);
    
    % Pressure ratio
    P2_P1 = 1 + 2 * gamma / (gamma + 1) * (Mn1^2 - 1);
end

%% ========================================================================
% SECTION 5: STABILITY AND CONTROL MODULE
% =========================================================================

function [Stability] = StabilityModule(Geometry, Aero, Mach)
    % Stability and control analysis for forward-swept configuration
    
    % Reference lengths
    l_ref = Geometry.fuselage.length;
    c_ref = Geometry.wing.MAC;
    
    % CG position (fraction of fuselage length from nose)
    Stability.xcg = 0.35; % Nominal CG
    
    % Aerodynamic center
    if Mach < 1
        xac_wing = 0.25 * c_ref + 0.35 * l_ref;
    else
        xac_wing = 0.35 * c_ref + 0.35 * l_ref; % AC moves aft supersonically
    end
    
    % Forward swept wing: AC shifts forward
    if Geometry.wing.sweep < 0
        xac_shift = -0.05 * abs(Geometry.wing.sweep) / 45 * l_ref;
        xac_wing = xac_wing + xac_shift;
    end
    
    % Canard contribution
    xac_canard = Geometry.canard.position * l_ref;
    eta_canard = 0.9; % Canard efficiency
    
    % Overall aerodynamic center
    S_total = Geometry.wing.area + Geometry.canard.area;
    Stability.xac = (xac_wing * Geometry.wing.area + ...
        xac_canard * Geometry.canard.area * eta_canard) / ...
        (Geometry.wing.area + Geometry.canard.area * eta_canard);
    
    % Static margin
    Stability.xcg_frac = Stability.xcg;
    Stability.xac_frac = Stability.xac / l_ref;
    Stability.SM = (Stability.xac_frac - Stability.xcg_frac) * 100; % percent MAC
    
    % Forward swept wing typically has negative SM (relaxed stability)
    % X-29 operated at -35% SM
    if Geometry.wing.sweep < 0
        Stability.SM_adjustment = -0.2 * abs(Geometry.wing.sweep) / 45 * 100;
        Stability.SM = Stability.SM + Stability.SM_adjustment;
    end
    
    % Longitudinal stability derivatives
    Stability.Cma = -Aero.CLa * (Stability.xac_frac - Stability.xcg_frac);
    Stability.Cmq = -2 * Aero.CLa * (Stability.xac_frac - Stability.xcg_frac)^2;
    
    % Lateral-directional stability
    Stability.Cnb = VerticalTailSizing(Geometry, Mach);
    Stability.Clb = -0.1 * (1 + abs(sin(Geometry.wing.sweep_rad)));
    
    % Forward sweep divergence speed
    if Geometry.wing.sweep < 0
        % Aeroelastic divergence concern
        EI = 1e8; % Bending stiffness estimate (N-m^2)
        GJ = 5e7; % Torsional stiffness estimate (N-m^2)
        
        [T, P, rho] = StandardAtmosphere(Geometry.flight.altitude);
        
        Stability.q_divergence = 2 * GJ / (c_ref * Geometry.wing.span^2 * ...
            abs(tan(Geometry.wing.sweep_rad)) * Aero.CLa);
        Stability.V_divergence = sqrt(2 * Stability.q_divergence / rho);
        Stability.M_divergence = Stability.V_divergence / sqrt(1.4 * 287 * T);
        
        % Structural coupling requirement
        Stability.aeroelastic_coupling = 'Active aeroelastic tailoring required';
    else
        Stability.q_divergence = inf;
        Stability.M_divergence = inf;
        Stability.aeroelastic_coupling = 'Conventional flutter boundaries';
    end
    
    % Control effectiveness
    Stability.control.elevator_effectiveness = 0.035 * (1 - 0.1 * abs(Mach - 1));
    Stability.control.aileron_effectiveness = 0.02 * cos(Geometry.wing.sweep_rad);
    Stability.control.rudder_effectiveness = 0.015;
    
    % Trim analysis
    CL_trim = 0.3;
    Stability.trim.de = -CL_trim / (Stability.control.elevator_effectiveness * ...
        Aero.CLa);
    Stability.trim.alpha = CL_trim / Aero.CLa * 180 / pi;
    
    % Control power requirements for relaxed stability
    Stability.control_power_required = abs(Stability.SM) * 2; % Relative measure
end

function Cnb = VerticalTailSizing(Geometry, Mach)
    % Vertical tail contribution to directional stability
    
    Vv = Geometry.vtail.area * (0.9 * Geometry.fuselage.length) / ...
        (Geometry.wing.area * Geometry.wing.span);
    
    if Mach < 1
        CLa_vt = 4; % per radian
    else
        CLa_vt = 4 / sqrt(Mach^2 - 1);
    end
    
    eta_vt = 0.9;
    Cnb = eta_vt * Vv * CLa_vt;
end

%% ========================================================================
% SECTION 6: STRUCTURES AND MATERIALS MODULE
% =========================================================================

function [Structures] = StructuresModule(Geometry, Aero, Propulsion, Mach)
    % Structural analysis and material selection
    
    % Load factors
    Structures.loads.n_limit = 9; % X-29 limit load factor
    Structures.loads.n_ultimate = 13.5;
    
    % Dynamic pressure at max speed
    [T, P, rho] = StandardAtmosphere(15000);
    V_max = Mach * sqrt(1.4 * 287 * T);
    q_max = 0.5 * rho * V_max^2;
    
    % Wing structural loads
    W_estimate = 7500; % kg initial estimate
    Structures.loads.wing_root_moment = 0.4 * W_estimate * 9.81 * ...
        Structures.loads.n_limit * Geometry.wing.span / 2;
    
    % Forward swept wing considerations
    if Geometry.wing.sweep < 0
        % Divergence loads increase
        Structures.loads.divergence_factor = 1 + 0.3 * abs(Geometry.wing.sweep) / 45;
        Structures.loads.wing_root_moment = Structures.loads.wing_root_moment * ...
            Structures.loads.divergence_factor;
        
        % Aeroelastic tailoring requirement
        Structures.aeroelastic.tailoring_required = true;
        Structures.aeroelastic.ply_orientation = [-45, 45, 0, 90]; % Composite layup
        Structures.aeroelastic.coupling_parameter = 0.4; % Bend-twist coupling
    else
        Structures.loads.divergence_factor = 1.0;
        Structures.aeroelastic.tailoring_required = false;
    end
    
    % Material selection based on Mach number
    if Mach < 2.5
        Structures.materials.primary = 'Aluminum_7075_T6';
        Structures.materials.secondary = 'CFRP_AS4_3501-6';
        Structures.materials.TPS = 'None';
        
        mat.density = 2800; % kg/m^3
        mat.yield = 503e6; % Pa
        mat.ultimate = 572e6; % Pa
        mat.E = 71.7e9; % Pa
        mat.temp_limit = 450; % K
        
    elseif Mach < 5
        Structures.materials.primary = 'Titanium_6Al4V';
        Structures.materials.secondary = 'Inconel_718';
        Structures.materials.TPS = 'Ceramic_tiles';
        
        mat.density = 4430; % kg/m^3
        mat.yield = 880e6; % Pa
        mat.ultimate = 950e6; % Pa
        mat.E = 113.8e9; % Pa
        mat.temp_limit = 700; % K
        
    else
        Structures.materials.primary = 'CMC_SiC';
        Structures.materials.secondary = 'Inconel_X750';
        Structures.materials.TPS = 'UHTC_ablative';
        
        mat.density = 2700; % kg/m^3
        mat.yield = 350e6; % Pa
        mat.ultimate = 400e6; % Pa
        mat.E = 380e9; % Pa
        mat.temp_limit = 1900; % K
    end
    
    Structures.material_properties = mat;
    
    % Thermal analysis
    Structures.thermal = ThermalAnalysis(Mach, Geometry);
    
    % Sizing calculations
    Structures.wing.skin_thickness = sqrt(Structures.loads.wing_root_moment / ...
        (mat.yield * Geometry.wing.root_chord * 0.12 * Geometry.wing.tau));
    Structures.wing.spar_cap_area = Structures.loads.wing_root_moment / ...
        (mat.yield * Geometry.wing.root_chord * 0.85 * Geometry.wing.tau);
    
    % Composite weight fraction (X-29 had significant composite use)
    if Geometry.wing.sweep < 0
        Structures.composite_fraction = 0.35; % Higher for FSW aeroelastic tailoring
    else
        Structures.composite_fraction = 0.20;
    end
    
    % Fatigue considerations
    Structures.fatigue.life_cycles = 10000; % Design flight hours equivalent
    Structures.fatigue.factor = 4; % Fatigue factor of safety
end

function [thermal] = ThermalAnalysis(Mach, Geometry)
    % Aerodynamic heating analysis
    
    [T_inf, P, rho] = StandardAtmosphere(15000);
    
    % Stagnation temperature
    T0 = T_inf * (1 + 0.2 * Mach^2);
    
    % Recovery temperature
    r = 0.89; % Recovery factor (turbulent)
    T_recovery = T_inf * (1 + r * 0.2 * Mach^2);
    
    thermal.T_stagnation = T0;
    thermal.T_recovery = T_recovery;
    
    % Heat flux estimates (reference heating rate)
    V = Mach * sqrt(1.4 * 287 * T_inf);
    thermal.q_dot_stag = 1.83e-4 * sqrt(rho / 0.3) * V^3; % W/m^2
    
    % Leading edge heating
    R_LE = 0.01 * Geometry.wing.root_chord; % Leading edge radius
    thermal.q_dot_LE = thermal.q_dot_stag * sqrt(0.3 / R_LE);
    
    % Equilibrium wall temperature (radiation equilibrium)
    epsilon = 0.8; % Emissivity
    sigma = 5.67e-8; % Stefan-Boltzmann
    thermal.T_wall_eq = (thermal.q_dot_LE / (epsilon * sigma))^0.25;
    
    % TPS requirements
    if thermal.T_wall_eq > 1500
        thermal.TPS_type = 'Active cooling required';
        thermal.TPS_mass_fraction = 0.15;
    elseif thermal.T_wall_eq > 900
        thermal.TPS_type = 'Ceramic TPS';
        thermal.TPS_mass_fraction = 0.08;
    elseif thermal.T_wall_eq > 500
        thermal.TPS_type = 'High-temp metallic';
        thermal.TPS_mass_fraction = 0.03;
    else
        thermal.TPS_type = 'Standard materials';
        thermal.TPS_mass_fraction = 0;
    end
end

%% ========================================================================
% SECTION 7: WEIGHTS AND BALANCE MODULE
% =========================================================================

function [Weights] = WeightsModule(Geometry, Structures, Propulsion, cruise_duration)
    % Weight estimation and CG analysis
    
    % Wing weight (Raymer statistical method with FSW correction)
    Nz = 9; % Ultimate load factor
    W_dg = 8000; % Design gross weight estimate (kg)
    q = 30000; % Design dynamic pressure (Pa)
    
    W_wing = 0.0051 * (W_dg * Nz)^0.557 * Geometry.wing.area^0.649 * ...
        Geometry.wing.AR^0.5 * (Geometry.wing.tau)^(-0.4) * ...
        (1 + abs(Geometry.wing.taper_ratio))^0.1 * ...
        cos(Geometry.wing.sweep_rad)^(-1) * Geometry.wing.area^0.1;
    
    % Forward sweep weight penalty
    if Geometry.wing.sweep < 0
        FSW_factor = 1 + 0.2 * abs(Geometry.wing.sweep) / 45;
        W_wing = W_wing * FSW_factor;
        % Additional weight for aeroelastic tailoring
        W_wing = W_wing * (1 + 0.15 * Structures.composite_fraction);
    end
    
    % Fuselage weight
    W_fuse = 0.052 * Geometry.fuselage.length^1.086 * ...
        (Geometry.fuselage.diameter * 100)^0.896 * ...
        (1 + 0.05 * q / 10000);
    
    % Empennage weight
    W_htail = 0; % Canard configuration
    W_vtail = 0.073 * (1 + 0.2 * abs(Geometry.wing.sweep) / 45) * ...
        Nz^0.376 * q^0.122 * Geometry.vtail.area^0.873;
    W_canard = 0.053 * Nz^0.376 * q^0.122 * Geometry.canard.area^0.873 * ...
        Geometry.wing.AR^0.357;
    
    % Landing gear
    W_lg = 0.043 * W_dg;
    
    % Propulsion system
    W_engine = Propulsion.weight;
    W_fuel_system = 0.025 * W_dg;
    W_propulsion = W_engine + W_fuel_system;
    
    % Systems
    W_flight_controls = 0.053 * Geometry.wing.area^0.8 * ...
        (1 + 0.5 * abs(Geometry.wing.sweep) / 45); % Higher for FSW active control
    W_hydraulics = 0.015 * W_dg;
    W_electrical = 0.03 * W_dg;
    W_avionics = 450; % kg (X-29 class)
    W_furnishings = 100; % kg (fighter)
    
    % TPS weight
    W_TPS = Structures.thermal.TPS_mass_fraction * W_dg;
    
    % Empty weight
    Weights.wing = W_wing;
    Weights.fuselage = W_fuse;
    Weights.canard = W_canard;
    Weights.vtail = W_vtail;
    Weights.landing_gear = W_lg;
    Weights.propulsion = W_propulsion;
    Weights.flight_controls = W_flight_controls;
    Weights.hydraulics = W_hydraulics;
    Weights.electrical = W_electrical;
    Weights.avionics = W_avionics;
    Weights.furnishings = W_furnishings;
    Weights.TPS = W_TPS;
    
    Weights.empty = W_wing + W_fuse + W_canard + W_vtail + W_lg + ...
        W_propulsion + W_flight_controls + W_hydraulics + ...
        W_electrical + W_avionics + W_furnishings + W_TPS;
    
    % Fuel weight
    Weights.fuel_cruise = Propulsion.fuel_cruise;
    Weights.fuel_reserve = 0.15 * Weights.fuel_cruise;
    Weights.fuel_total = Weights.fuel_cruise + Weights.fuel_reserve;
    
    % Payload
    Weights.payload = 450; % kg (pilot + equipment)
    
    % Gross weights
    Weights.OEW = Weights.empty;
    Weights.MTOW = Weights.OEW + Weights.fuel_total + Weights.payload;
    Weights.landing = Weights.MTOW - Weights.fuel_cruise;
    
    % CG calculation
    CG_positions = struct(...
        'wing', 0.40, ...
        'fuselage', 0.45, ...
        'canard', 0.15, ...
        'vtail', 0.85, ...
        'propulsion', 0.70, ...
        'fuel', 0.40, ...
        'payload', 0.30);
    
    moment_sum = W_wing * CG_positions.wing + W_fuse * CG_positions.fuselage + ...
        W_canard * CG_positions.canard + W_vtail * CG_positions.vtail + ...
        W_propulsion * CG_positions.propulsion;
    
    Weights.CG.empty = moment_sum / Weights.empty;
    Weights.CG.full_fuel = (moment_sum + Weights.fuel_total * CG_positions.fuel + ...
        Weights.payload * CG_positions.payload) / Weights.MTOW;
    Weights.CG.min_fuel = (moment_sum + 0.1 * Weights.fuel_total * CG_positions.fuel + ...
        Weights.payload * CG_positions.payload) / ...
        (Weights.empty + 0.1 * Weights.fuel_total + Weights.payload);
    
    % CG travel
    Weights.CG.travel = abs(Weights.CG.full_fuel - Weights.CG.min_fuel) * ...
        Geometry.fuselage.length / Geometry.wing.MAC * 100; % % MAC
    
    % Weight fractions
    Weights.fractions.structure = (W_wing + W_fuse + W_canard + W_vtail) / Weights.MTOW;
    Weights.fractions.propulsion = W_propulsion / Weights.MTOW;
    Weights.fractions.systems = (W_flight_controls + W_hydraulics + W_electrical + ...
        W_avionics) / Weights.MTOW;
    Weights.fractions.fuel = Weights.fuel_total / Weights.MTOW;
    Weights.fractions.payload = Weights.payload / Weights.MTOW;
end

%% ========================================================================
% SECTION 8: COST MODULE
% =========================================================================

function [Cost] = CostModule(Weights, Structures, Propulsion, Mach, production_qty)
    % Cost estimation using DAPCA IV model and adjustments
    
    % Program parameters
    if nargin < 5
        production_qty = 2; % X-29 only had 2 aircraft
    end
    
    % Engineering hours
    H_E = 4.86 * Weights.empty^0.777 * 1.8^0.894 * production_qty^0.163;
    
    % Tooling hours
    H_T = 5.99 * Weights.empty^0.777 * 1.8^0.696 * production_qty^0.263;
    
    % Manufacturing hours
    H_M = 7.37 * Weights.empty^0.82 * 1.8^0.484 * production_qty^0.641;
    
    % Quality control
    H_Q = 0.133 * H_M;
    
    % Labor rates (2024 USD/hr)
    R_E = 150; % Engineering
    R_T = 120; % Tooling
    R_M = 100; % Manufacturing
    R_Q = 90; % QC
    
    % Development cost
    Cost.development.engineering = H_E * R_E;
    Cost.development.tooling = H_T * R_T;
    Cost.development.manufacturing = H_M * R_M;
    Cost.development.QC = H_Q * R_Q;
    Cost.development.total = Cost.development.engineering + Cost.development.tooling + ...
        Cost.development.manufacturing + Cost.development.QC;
    
    % Material costs
    if strcmp(Structures.materials.primary, 'Aluminum_7075_T6')
        mat_cost_factor = 1.0;
    elseif strcmp(Structures.materials.primary, 'Titanium_6Al4V')
        mat_cost_factor = 8.0;
    else
        mat_cost_factor = 25.0; % CMC and advanced materials
    end
    
    Cost.materials = Weights.empty * 50 * mat_cost_factor; % $/kg base
    
    % Composite premium
    Cost.composite_premium = Weights.empty * Structures.composite_fraction * 200;
    
    % Engine cost
    Cost.engine = Propulsion.weight * 5000; % $/kg for advanced engine
    
    % Avionics cost
    Cost.avionics = 15e6; % Advanced FCS for FSW
    
    % Flight test
    Cost.flight_test = 50e6 + 10e6 * (abs(Mach - 1.8) / 1.8);
    
    % Total flyaway cost per aircraft
    Cost.flyaway = (Cost.development.total + Cost.materials + ...
        Cost.composite_premium + Cost.engine + Cost.avionics) / production_qty;
    
    % Program cost
    Cost.program_total = Cost.flyaway * production_qty + Cost.flight_test;
    
    % Operating costs
    Cost.operating.fuel_per_hour = Propulsion.fuel_flow * 3600 * 2.5; % $/hr at $2.5/kg
    Cost.operating.maintenance_per_hour = Cost.flyaway * 1e-5;
    Cost.operating.total_per_hour = Cost.operating.fuel_per_hour + ...
        Cost.operating.maintenance_per_hour;
    
    % Cost indices
    Cost.cost_per_kg = Cost.flyaway / Weights.MTOW;
    Cost.development_per_aircraft = Cost.development.total / production_qty;
end

%% ========================================================================
% SECTION 9: SYNTHESIS AND CONVERGENCE MODULE
% =========================================================================

function [Synthesis] = SynthesisModule(tau, sweep, cruise_duration, mach_range)
    % Hypersonic convergence synthesis
    
    fprintf('\n--- Running Synthesis for tau=%.2f, sweep=%d°, duration=%d min ---\n', ...
        tau, sweep, cruise_duration);
    
    % Initialize geometry
    scale_factor = 1.0;
    Geometry = GeometryModule(tau, sweep, scale_factor);
    Geometry.flight.altitude = 15000;
    
    % Iterate for weight convergence
    W_guess = 7500;
    tol = 0.01;
    max_iter = 20;
    
    for iter = 1:max_iter
        % Run analysis at design Mach
        Mach_design = min(mach_range);
        
        % Aerodynamics
        Aero = AerodynamicsModule(Geometry, Mach_design, Geometry.flight.altitude, tau);
        
        % Propulsion
        Propulsion = PropulsionModule(Geometry, Mach_design, ...
            Geometry.flight.altitude, cruise_duration);
        
        % Stability
        Stability = StabilityModule(Geometry, Aero, Mach_design);
        
        % Structures
        Structures = StructuresModule(Geometry, Aero, Propulsion, Mach_design);
        
        % Weights
        Weights = WeightsModule(Geometry, Structures, Propulsion, cruise_duration);
        
        % Check convergence
        W_new = Weights.MTOW;
        if abs(W_new - W_guess) / W_guess < tol
            break;
        end
        W_guess = 0.5 * W_guess + 0.5 * W_new;
    end
    
    % Cost analysis
    Cost = CostModule(Weights, Structures, Propulsion, Mach_design, 2);
    
    % Store converged results
    Synthesis.Geometry = Geometry;
    Synthesis.Aerodynamics = Aero;
    Synthesis.Propulsion = Propulsion;
    Synthesis.Stability = Stability;
    Synthesis.Structures = Structures;
    Synthesis.Weights = Weights;
    Synthesis.Cost = Cost;
    Synthesis.convergence_iterations = iter;
    
    % Multi-Mach analysis
    Synthesis.Mach_sweep = struct();
    for i = 1:length(mach_range)
        M = mach_range(i);
        Synthesis.Mach_sweep(i).Mach = M;
        Synthesis.Mach_sweep(i).Aero = AerodynamicsModule(Geometry, M, ...
            Geometry.flight.altitude, tau);
        Synthesis.Mach_sweep(i).Prop = PropulsionModule(Geometry, M, ...
            Geometry.flight.altitude, cruise_duration);
    end
    
    % Performance metrics
    Synthesis.Performance = PerformanceMetrics(Synthesis);
    
    % Figure of Merit
    Synthesis.FOM = FigureOfMerit(Synthesis);
end

function [Perf] = PerformanceMetrics(Synthesis)
    % Calculate key performance parameters
    
    W = Synthesis.Weights.MTOW * 9.81;
    S = Synthesis.Geometry.wing.area;
    
    % Wing loading
    Perf.wing_loading = W / S; % N/m^2
    
    % Thrust to weight
    Perf.TW_ratio = Synthesis.Propulsion.thrust / W;
    
    % Maximum speed capability
    Perf.Mach_max = 1.8; % Design estimate
    
    % Range (Breguet)
    [T, P, rho] = StandardAtmosphere(15000);
    V = Synthesis.Aerodynamics.flight.Mach * sqrt(1.4 * 287 * T);
    LD = Synthesis.Aerodynamics.LD_cruise;
    Wi_Wf = Synthesis.Weights.MTOW / (Synthesis.Weights.MTOW - Synthesis.Weights.fuel_cruise);
    Perf.range = V / (Synthesis.Propulsion.TSFC * 9.81) * LD * log(Wi_Wf) / 1000; % km
    
    % Rate of climb (energy method)
    Ps = (Synthesis.Propulsion.thrust - Synthesis.Aerodynamics.CD_cruise * ...
        0.5 * rho * V^2 * S) * V / W;
    Perf.ROC_max = Ps; % m/s
    
    % Turn performance
    Perf.n_sustained = Synthesis.Propulsion.thrust / ...
        (Synthesis.Aerodynamics.CD_cruise * 0.5 * rho * V^2 * S);
    Perf.turn_rate = 9.81 * sqrt(Perf.n_sustained^2 - 1) / V * 180 / pi; % deg/s
    
    % Takeoff/Landing
    Perf.stall_speed = sqrt(2 * W / (1.225 * S * Synthesis.Aerodynamics.CLmax));
    Perf.approach_speed = 1.3 * Perf.stall_speed;
end

function FOM = FigureOfMerit(Synthesis)
    % Composite figure of merit for design comparison
    
    Perf = Synthesis.Performance;
    
    % Normalize to X-29 reference
    FOM.maneuverability = Perf.turn_rate / 20; % Relative to 20 deg/s
    FOM.speed = Synthesis.Aerodynamics.flight.Mach / 1.8;
    FOM.efficiency = Synthesis.Aerodynamics.LD_cruise / 8;
    FOM.weight = 8000 / Synthesis.Weights.MTOW;
    FOM.cost = 100e6 / Synthesis.Cost.flyaway;
    FOM.stability_margin = abs(Synthesis.Stability.SM + 35) / 35; % X-29 had -35%
    
    % Weighted composite
    weights = [0.20, 0.15, 0.15, 0.15, 0.15, 0.20];
    values = [FOM.maneuverability, FOM.speed, FOM.efficiency, ...
        FOM.weight, FOM.cost, FOM.stability_margin];
    
    FOM.composite = sum(weights .* min(values, 1.5)); % Cap at 1.5
end

%% ========================================================================
% SECTION 10: UTILITY FUNCTIONS
% =========================================================================

function [T, P, rho] = StandardAtmosphere(h)
    % Standard atmosphere model
    % Input: h - altitude in meters
    % Output: T (K), P (Pa), rho (kg/m^3)
    
    if h < 11000
        T = 288.15 - 0.0065 * h;
        P = 101325 * (T / 288.15)^5.2561;
    elseif h < 20000
        T = 216.65;
        P = 22632 * exp(-9.81 * (h - 11000) / (287 * 216.65));
    elseif h < 32000
        T = 216.65 + 0.001 * (h - 20000);
        P = 5474.9 * (T / 216.65)^(-34.163);
    else
        T = 228.65 + 0.0028 * (h - 32000);
        P = 868.02 * (T / 228.65)^(-12.201);
    end
    
    rho = P / (287 * T);
end

%% ========================================================================
% SECTION 11: MAIN EXECUTION AND PARAMETRIC STUDY
% =========================================================================

% Run parametric study
fprintf('\n========================================\n');
fprintf('PARAMETRIC STUDY EXECUTION\n');
fprintf('========================================\n');

% Store all results
Results = struct();
result_idx = 1;

mach_range = [0.8, 1.2, 1.5, 1.8, 2.5, 3.0, 5.0];

% Selected parameter combinations
tau_selected = [0.03, 0.04, 0.05, 0.07];
sweep_selected = [-45, -30, 0, 30];
duration_selected = [1, 5, 10];

for t = 1:length(tau_selected)
    for s = 1:length(sweep_selected)
        for d = 1:length(duration_selected)
            tau = tau_selected(t);
            sweep = sweep_selected(s);
            duration = duration_selected(d);
            
            try
                Results(result_idx).tau = tau;
                Results(result_idx).sweep = sweep;
                Results(result_idx).duration = duration;
                Results(result_idx).Synthesis = SynthesisModule(tau, sweep, ...
                    duration, mach_range);
                Results(result_idx).valid = true;
            catch ME
                Results(result_idx).valid = false;
                Results(result_idx).error = ME.message;
                fprintf('Error at tau=%.2f, sweep=%d: %s\n', tau, sweep, ME.message);
            end
            
            result_idx = result_idx + 1;
        end
    end
end

%% ========================================================================
% SECTION 12: VISUALIZATION AND OUTPUT
% =========================================================================

fprintf('\n========================================\n');
fprintf('GENERATING VISUALIZATIONS\n');
fprintf('========================================\n');

% Figure 1: Parametric Trade Study - Weight vs Sweep
figure('Name', 'Weight vs Wing Sweep', 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
for t = 1:length(tau_selected)
    tau_val = tau_selected(t);
    sweep_data = [];
    weight_data = [];
    
    for r = 1:length(Results)
        if Results(r).valid && Results(r).tau == tau_val && Results(r).duration == 5
            sweep_data = [sweep_data, Results(r).sweep];
            weight_data = [weight_data, Results(r).Synthesis.Weights.MTOW];
        end
    end
    
    [sweep_sorted, idx] = sort(sweep_data);
    weight_sorted = weight_data(idx);
    plot(sweep_sorted, weight_sorted, '-o', 'LineWidth', 2, ...
        'DisplayName', sprintf('\\tau = %.2f', tau_val));
    hold on;
end
xlabel('Wing Sweep (degrees)');
ylabel('MTOW (kg)');
title('MTOW vs Wing Sweep Angle');
legend('Location', 'best');
grid on;

% X-29 reference point
plot(-33.73, 8074, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r', ...
    'DisplayName', 'X-29 Reference');

subplot(2,2,2);
for t = 1:length(tau_selected)
    tau_val = tau_selected(t);
    sweep_data = [];
    ld_data = [];
    
    for r = 1:length(Results)
        if Results(r).valid && Results(r).tau == tau_val && Results(r).duration == 5
            sweep_data = [sweep_data, Results(r).sweep];
            ld_data = [ld_data, Results(r).Synthesis.Aerodynamics.LD_cruise];
        end
    end
    
    [sweep_sorted, idx] = sort(sweep_data);
    ld_sorted = ld_data(idx);
    plot(sweep_sorted, ld_sorted, '-s', 'LineWidth', 2, ...
        'DisplayName', sprintf('\\tau = %.2f', tau_val));
    hold on;
end
xlabel('Wing Sweep (degrees)');
ylabel('L/D at Cruise');
title('Cruise L/D vs Wing Sweep');
legend('Location', 'best');
grid on;

subplot(2,2,3);
for t = 1:length(tau_selected)
    tau_val = tau_selected(t);
    sweep_data = [];
    sm_data = [];
    
    for r = 1:length(Results)
        if Results(r).valid && Results(r).tau == tau_val && Results(r).duration == 5
            sweep_data = [sweep_data, Results(r).sweep];
            sm_data = [sm_data, Results(r).Synthesis.Stability.SM];
        end
    end
    
    [sweep_sorted, idx] = sort(sweep_data);
    sm_sorted = sm_data(idx);
    plot(sweep_sorted, sm_sorted, '-d', 'LineWidth', 2, ...
        'DisplayName', sprintf('\\tau = %.2f', tau_val));
    hold on;
end
xlabel('Wing Sweep (degrees)');
ylabel('Static Margin (%)');
title('Static Margin vs Wing Sweep');
legend('Location', 'best');
grid on;
plot([-50, 50], [-35, -35], 'r--', 'LineWidth', 1.5, 'DisplayName', 'X-29 SM');
ylim([-60, 20]);

subplot(2,2,4);
for t = 1:length(tau_selected)
    tau_val = tau_selected(t);
    sweep_data = [];
    fom_data = [];
    
    for r = 1:length(Results)
        if Results(r).valid && Results(r).tau == tau_val && Results(r).duration == 5
            sweep_data = [sweep_data, Results(r).sweep];
            fom_data = [fom_data, Results(r).Synthesis.FOM.composite];
        end
    end
    
    [sweep_sorted, idx] = sort(sweep_data);
    fom_sorted = fom_data(idx);
    plot(sweep_sorted, fom_sorted, '-^', 'LineWidth', 2, ...
        'DisplayName', sprintf('\\tau = %.2f', tau_val));
    hold on;
end
xlabel('Wing Sweep (degrees)');
ylabel('Figure of Merit');
title('Composite FOM vs Wing Sweep');
legend('Location', 'best');
grid on;

saveas(gcf, 'X29_Parametric_Sweep.png');

% Figure 2: Mach Number Effects
figure('Name', 'Mach Effects', 'Position', [100, 100, 1200, 600]);

% Select X-29 like configuration
x29_config_idx = 0;
for r = 1:length(Results)
    if Results(r).valid && Results(r).sweep == -30 && ...
            Results(r).tau == 0.04 && Results(r).duration == 5
        x29_config_idx = r;
        break;
    end
end

if x29_config_idx > 0
    Mach_data = zeros(1, length(Results(x29_config_idx).Synthesis.Mach_sweep));
    CD0_data = zeros(1, length(Results(x29_config_idx).Synthesis.Mach_sweep));
    LD_data = zeros(1, length(Results(x29_config_idx).Synthesis.Mach_sweep));
    Thrust_data = zeros(1, length(Results(x29_config_idx).Synthesis.Mach_sweep));
    
    for i = 1:length(Results(x29_config_idx).Synthesis.Mach_sweep)
        Mach_data(i) = Results(x29_config_idx).Synthesis.Mach_sweep(i).Mach;
        CD0_data(i) = Results(x29_config_idx).Synthesis.Mach_sweep(i).Aero.CD0 + ...
            Results(x29_config_idx).Synthesis.Mach_sweep(i).Aero.CDwave;
        LD_data(i) = Results(x29_config_idx).Synthesis.Mach_sweep(i).Aero.LD_max;
        Thrust_data(i) = Results(x29_config_idx).Synthesis.Mach_sweep(i).Prop.thrust;
    end
    
    subplot(1,3,1);
    plot(Mach_data, CD0_data, 'b-o', 'LineWidth', 2);
    xlabel('Mach Number');
    ylabel('CD_0 + CD_{wave}');
    title('Zero-Lift Drag vs Mach');
    grid on;
    
    subplot(1,3,2);
    plot(Mach_data, LD_data, 'r-s', 'LineWidth', 2);
    xlabel('Mach Number');
    ylabel('(L/D)_{max}');
    title('Maximum L/D vs Mach');
    grid on;
    
    subplot(1,3,3);
    plot(Mach_data, Thrust_data/1000, 'g-d', 'LineWidth', 2);
    xlabel('Mach Number');
    ylabel('Thrust (kN)');
    title('Available Thrust vs Mach');
    grid on;
end

saveas(gcf, 'X29_Mach_Effects.png');

% Figure 3: Cost Trade Study
figure('Name', 'Cost Analysis', 'Position', [100, 100, 1000, 600]);

sweep_cost = [];
flyaway_cost = [];
weight_cost = [];

for r = 1:length(Results)
    if Results(r).valid && Results(r).tau == 0.04 && Results(r).duration == 5
        sweep_cost = [sweep_cost, Results(r).sweep];
        flyaway_cost = [flyaway_cost, Results(r).Synthesis.Cost.flyaway / 1e6];
        weight_cost = [weight_cost, Results(r).Synthesis.Weights.MTOW];
    end
end

[sweep_sorted, idx] = sort(sweep_cost);
flyaway_sorted = flyaway_cost(idx);
weight_sorted = weight_cost(idx);

yyaxis left;
plot(sweep_sorted, flyaway_sorted, 'b-o', 'LineWidth', 2);
ylabel('Flyaway Cost ($M)');

yyaxis right;
plot(sweep_sorted, weight_sorted, 'r-s', 'LineWidth', 2);
ylabel('MTOW (kg)');

xlabel('Wing Sweep (degrees)');
title('Cost and Weight vs Wing Sweep (τ = 0.04)');
grid on;
legend('Flyaway Cost', 'MTOW', 'Location', 'best');

saveas(gcf, 'X29_Cost_Analysis.png');

% Figure 4: 3D Trade Space
figure('Name', '3D Trade Space', 'Position', [100, 100, 1000, 800]);

tau_3d = [];
sweep_3d = [];
fom_3d = [];

for r = 1:length(Results)
    if Results(r).valid && Results(r).duration == 5
        tau_3d = [tau_3d, Results(r).tau];
        sweep_3d = [sweep_3d, Results(r).sweep];
        fom_3d = [fom_3d, Results(r).Synthesis.FOM.composite];
    end
end

scatter3(tau_3d, sweep_3d, fom_3d, 100, fom_3d, 'filled');
xlabel('Thickness Ratio (τ)');
ylabel('Wing Sweep (degrees)');
zlabel('Figure of Merit');
title('Design Space Exploration');
colorbar;
colormap(jet);
view(45, 30);
grid on;

% Mark X-29 design point
hold on;
plot3(0.04, -33.73, 0.9, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r');
text(0.04, -33.73, 0.95, 'X-29', 'FontSize', 12);

saveas(gcf, 'X29_3D_Trade_Space.png');

%% ========================================================================
% SECTION 13: RESULTS SUMMARY
% =========================================================================

fprintf('\n========================================\n');
fprintf('RESULTS SUMMARY\n');
fprintf('========================================\n\n');

% Find optimal configuration
best_fom = 0;
best_idx = 1;
for r = 1:length(Results)
    if Results(r).valid && Results(r).Synthesis.FOM.composite > best_fom
        best_fom = Results(r).Synthesis.FOM.composite;
        best_idx = r;
    end
end

fprintf('OPTIMAL CONFIGURATION:\n');
fprintf('  Thickness Ratio (tau): %.3f\n', Results(best_idx).tau);
fprintf('  Wing Sweep: %d degrees\n', Results(best_idx).sweep);
fprintf('  Cruise Duration: %d minutes\n', Results(best_idx).duration);
fprintf('  Figure of Merit: %.3f\n\n', best_fom);

fprintf('OPTIMAL CONFIGURATION DETAILS:\n');
fprintf('  MTOW: %.1f kg\n', Results(best_idx).Synthesis.Weights.MTOW);
fprintf('  Empty Weight: %.1f kg\n', Results(best_idx).Synthesis.Weights.OEW);
fprintf('  Cruise L/D: %.2f\n', Results(best_idx).Synthesis.Aerodynamics.LD_cruise);
fprintf('  Static Margin: %.1f%%\n', Results(best_idx).Synthesis.Stability.SM);
fprintf('  Flyaway Cost: $%.1f M\n\n', Results(best_idx).Synthesis.Cost.flyaway / 1e6);

fprintf('X-29 REFERENCE COMPARISON:\n');
fprintf('  X-29 MTOW: 8074 kg\n');
fprintf('  X-29 Empty Weight: 6260 kg\n');
fprintf('  X-29 Wing Sweep: -33.73 degrees\n');
fprintf('  X-29 Max Mach: 1.8\n');

% Find X-29-like configuration
x29_like_idx = 0;
for r = 1:length(Results)
    if Results(r).valid && Results(r).sweep == -30 && ...
            Results(r).tau == 0.04 && Results(r).duration == 5
        x29_like_idx = r;
        break;
    end
end

if x29_like_idx > 0
    fprintf('\nX-29 APPROXIMATION (sweep=-30°, tau=0.04):\n');
    fprintf('  Model MTOW: %.1f kg (%.1f%% diff)\n', ...
        Results(x29_like_idx).Synthesis.Weights.MTOW, ...
        (Results(x29_like_idx).Synthesis.Weights.MTOW - 8074) / 8074 * 100);
    fprintf('  Model Empty: %.1f kg (%.1f%% diff)\n', ...
        Results(x29_like_idx).Synthesis.Weights.OEW, ...
        (Results(x29_like_idx).Synthesis.Weights.OEW - 6260) / 6260 * 100);
end

%% ========================================================================
% SECTION 14: EXPORT RESULTS
% =========================================================================

% Save results to file
save('X29_Hypersonic_Convergence_Results.mat', 'Results', 'X29_Reference', ...
    'FlightConditions', 'tau_range', 'sweep_range', 'cruise_duration_range');

% Create summary table
fprintf('\n========================================\n');
fprintf('COMPLETE PARAMETRIC RESULTS TABLE\n');
fprintf('========================================\n');
fprintf('%-6s %-8s %-6s %-10s %-10s %-8s %-10s %-8s\n', ...
    'tau', 'Sweep', 'Dur', 'MTOW(kg)', 'OEW(kg)', 'L/D', 'SM(%)', 'FOM');
fprintf('%s\n', repmat('-', 1, 75));

for r = 1:length(Results)
    if Results(r).valid
        fprintf('%-6.2f %-8d %-6d %-10.1f %-10.1f %-8.2f %-10.1f %-8.3f\n', ...
            Results(r).tau, ...
            Results(r).sweep, ...
            Results(r).duration, ...
            Results(r).Synthesis.Weights.MTOW, ...
            Results(r).Synthesis.Weights.OEW, ...
            Results(r).Synthesis.Aerodynamics.LD_cruise, ...
            Results(r).Synthesis.Stability.SM, ...
            Results(r).Synthesis.FOM.composite);
    end
end

fprintf('\n========================================\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('==========================================\n');