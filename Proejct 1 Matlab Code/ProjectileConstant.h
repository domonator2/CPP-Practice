#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <array>

struct ProjectileConstant{
    // [rad] Ground station Latitude
    double phi_gs = (32 + 44.0/60.0 + 52.0/3600.0) * M_PI / 180.0;

    // [rad] Ground station Longitude
    double lambda_gs = -(97 + 5.0/60.0 + 34.0/3600.0) * M_PI / 180.0;

    // [km] Ground Station altitude above mean equator
    double h_gs = 0.185;

    // [km/s] Initial Speed
    double v0 = 0.827;

    // [deg] Initial Azimuth
    double az0 = 107.0;

    // [deg] Initial Elevation
    double el0 = 60;

    // [kg] Projectile Mass
    double mcg = 35.0;

    // [] Drag Coefficient
    double CD = 0.82;

    // [km^2] profile Area
    double ar = 6.0225 * 1e-3 * M_PI * 1e-6;

    // [km/s^2] Acceleration due to gravity at sea level
    double g0 = 0.00980665;

    // [km] Mean Equatorial Radius of the Earth
    double Re = 6378.137;

    // [km^3/s^2] Gravitational Parameter of the Earth
    double Gm = 3.986004418 * 1e5;

    // [rad/s] Rotational Speed of the Earth
    double we = 2 * M_PI / 86164.1;

    // [rad/s] Rotational Velocity of the Earth
    std::array<double, 3> We = {0, 0, 2 * M_PI / 86164.1};
};
