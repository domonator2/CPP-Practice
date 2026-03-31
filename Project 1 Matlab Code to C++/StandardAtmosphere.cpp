// This function Calculates atmospheric properties using the 1976 standard atmosphere model.

// Authored by: Dominic Larin
// Date: 2/9/2026

#include "StandardAtmosphere.h"
#include <cmath>

AtmosphereOutput StandardAtmosphere(double hg){
    
    // [m/s^2] Acceleration due to gravity
    double g0 = 9.80665;

    // [J/(kg*K)] Specific Gas Constant
    double R = 287.0;

    // [m] Mean equatorial radius of the earth
    double Re = 6378137.0;

    // [m] Geopotential Altitude
    double h = Re * hg / (Re + hg);

    if (h<= 11000){
        
        // [K/m] Temperature Gradient
        double a = (216.66 - 288.16) / 11000;

        // [K] Temperature
        double temperature = 288.16 + a * h;

        // [Pa] Pressure.
        double pressure = 101325 * pow((temperature / 288.16), (-g0 / (a * R)));

        // [kg/m^3] Density
        double density = pressure / (R * temperature);

        return {temperature, pressure, density};
    }
    else if(h>11000 && h <= 25000){

        // [K] Temperature
        double temperature = 216.66;

        // [Pa] Pressure
        double pressure = 22650.1684742737 * std::exp((-g0 / (R * temperature) * (h - 11000)));

        // [kg/m^3] Density
        double density = pressure / (R * temperature);

        return {temperature, pressure, density};
    }
    else if (h > 25000 && h <= 47000){

        // [K/m] Temperature gradient
        double a = (282.66 - 216.66) / (47000 - 25000);

        // [K] Temperature
        double temperature = 216.66 + a * (h - 25000);

        // [Pa] Pressure
        double pressure = 2493.58245271879 * pow((temperature / 216.66), (-g0 / (a * R)));

        // [kg/m^3] Density
        double density = pressure / (R * temperature);

        return{temperature, pressure, density};
    }
    else {
        return{0.0, 0.0, 0.0};
    }
}