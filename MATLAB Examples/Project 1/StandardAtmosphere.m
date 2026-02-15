%{
====================================================================================================
FUNCTION NAME: StandardAtmosphere.m
AUTHOR: Julio César Benavides, Ph.D.
CREATED: 07/17/2012
REVISED: 04/10/2023
====================================================================================================
FUNCTION DESCRIPTION:
This function calculates the local atmospheric temperature, pressure, and density based on the ARDC
1959 Standard Atmosphere.
====================================================================================================
INPUT VARIABLES:
(hg)|Geometric altitude.
====================================================================================================
OUTPUT VARIABLES:
(T)|Local atmospheric temperature.
----------------------------------------------------------------------------------------------------
(P)|Local atmospheric pressure.
----------------------------------------------------------------------------------------------------
(D)|Local atmospheric density.
====================================================================================================
VARIABLE FORMAT:
(hg)|Scalar {1x1} (m).
----------------------------------------------------------------------------------------------------
(T)|Scalar {1x1} (K).
----------------------------------------------------------------------------------------------------
(P)|Scalar {1x1} (Pa).
----------------------------------------------------------------------------------------------------
(D)|Scalar {1x1} (kg/m^3).
====================================================================================================
AUXILIARY FUNCTIONS:
None.
====================================================================================================
ABBREVIATIONS:
None.
====================================================================================================
COMMENTS:
Any use of this code either in part or in full must first be approved by Dr. Julio César Benavides,
Professor of Instruction in the Department of Mechanical and Aerospace Engineering at The University
of Texas at Arlington.  For permission to use this code, Dr. Benavides may be contacted at
julio.benavides@uta.edu.
====================================================================================================
%}

function [T,P,D] = StandardAtmosphere(hg)

    g = 9.80665;
    %[m/s^2]Mean acceleration due to gravity.

    R = 287;
    %[J/kg/K]Specific gas constant.

    Re = 6378137;
    %[m]Mean equatorial radius of the Earth.

    h = Re * hg / (Re + hg);
    %[m]Converts the gemometric alitude to geopotential altitude.

    if (h <= 11000)

        a = (216.66 - 288.16) / 11000;
        %[K/m]Temperature gradient.

        T = 288.16 + a * h;
        %[K]Temperature

        P = 101325 * (T / 288.16)^(-g / (a * R));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 11000) && (h <= 25000)

        T = 216.66;
        %[K]Temperature

        P = 22650.1684742737 * exp(-g / (R * T) * (h - 11000));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 25000) && (h <= 47000)

        a = (282.66 - 216.66) / (47000 - 25000);
        %[K/m]Temperature gradient.

        T = 216.66 + a * (h - 25000);
        %[K]Temperature

        P = 2493.58245271879 * (T / 216.66)^(-g / (a * R));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 47000) && (h <= 53000)

        T = 282.66;
        %[K]Temperature

        P = 120.879682128688 * exp(-g / (R * T) * (h - 47000));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 53000) && (h <= 79000)

        a = (165.66 - 282.66) / (79e3 - 53000);
        %[K/m]Temperature gradient.

        T = 282.66 + a * (h - 53000);
        %[K]Temperature

        P = 58.5554504138705 * (T / 282.66)^(-g / (a * R));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 79000) && (h <= 90000)

        T = 165.66;
        %[K]Temperature

        P = 1.01573256565262 * exp(-g / (R * T) * (h - 79000));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 90000) && (h <= 105000)

        a = (225.66 - 165.66) / (105000 - 90000);
        %[K/m]Temperature gradient.

        T = 165.66 + a * (h - 90e3);
        %[K]Temperature

        P = 0.105215646463089 * (T / 165.66)^(-g / (a * R));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 105000) && (h <= 500000)

        a = (4 - 225.66) / (500000 - 105000);
        %[K/m]Temperature gradient.

        T = 225.66 + a * (h - 105000);
        %[K]Temperature

        P = 0.00751891790761519 * (T / 225.66)^(-g / (a * R));
        %[Pa]Pressure.

        D = P / (R * T);
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    elseif (h > 500000)

        T = 4;
        %[K]Temperature

        P = 0;
        %[Pa]Pressure.

        D = 0;
        %[kg/m^3]Density.

        return;
        %[]Closes the density function and returns the focus to the calling function or script.

    end

end
%===================================================================================================