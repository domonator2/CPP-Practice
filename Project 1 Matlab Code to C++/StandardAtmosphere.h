// Header File for the standard Atmosphere Model
// Authored by: Dominic Larin
// Date: 2/9/2026

#pragma once

// Declare the struct and the function Signature

struct AtmosphereOutput{
    // [K] Absolute Temperature
    double temperature;

    // [Pa] Ambient Pressure
    double pressure;

    // [kg/m^3] Air Density
    double density;
};

AtmosphereOutput StandardAtmosphere(double hg);