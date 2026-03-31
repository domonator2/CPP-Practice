// This function serves to establish the EOMs for the simulation.

#include "ProjectileEOM.h"
#include "StandardAtmosphere.h"
#include <Eigen/Dense>
#include <cmath>

 Eigen::VectorXd ProjectileEOM(double t, const Eigen::VectorXd& S, const ProjectileConstant& C) {
    
    // [km, km, km] Projectile Position relative to Ground Station
    Eigen::Vector3d relPos_ENZ = S.segment<3>(0);

    // [km/s, km/s, km/s] Projectile Velocity Relative to Ground Station in ENZ
    Eigen::Vector3d relVel_ENZ = S.segment<3>(3);

   // [km, km, km] Ground Station Position WRT Earth in ECEF
    Eigen::Vector3d GS_R_ECEF = (C.Re + C.h_gs) * Eigen::Vector3d(
      std::cos(C.phi_gs)*std::cos(C.lambda_gs), 
      std::cos(C.phi_gs)*std::sin(C.lambda_gs), 
      std::sin(C.phi_gs));

   // [] Z-hat unit vector in ECEF Coordinates.
   Eigen::Vector3d Z_ECEF = GS_R_ECEF / GS_R_ECEF.norm();

   // [] E-hat unit vector in ECEF coordinates.
   Eigen::Vector3d E_ECEF = Eigen::Vector3d(
      -std::sin(C.lambda_gs),
      std::cos(C.lambda_gs),
      0);

   // [] N-hat Unit Vector in ECEF Coordinates
   Eigen::Vector3d N_ECEF = Z_ECEF.cross(E_ECEF);

   // [] Transformation Matrix from ENZ to ECEF
   Eigen::Matrix3d ENZ2ECEF;
   ENZ2ECEF << E_ECEF, N_ECEF, Z_ECEF;

   // [] Transformation Matrix from ECEF to ENZ.
   Eigen::Matrix3d ECEF2ENZ = ENZ2ECEF.transpose();

   // [km, km, km] Projectile Position WRT Earth in ECEF
   Eigen::Vector3d CG_R_ECEF = GS_R_ECEF + ENZ2ECEF * relPos_ENZ;

   // [km] Magnitude of projectile position vector from Earth's Center
   double r_cg = CG_R_ECEF.norm();

   // [km] Altitude of Projectile above Earth's Mean Radius
   double h_cg = r_cg - C.Re;

   // [m] Convert altitude to meters for standard atmosphere model.
   double h_cg_m = h_cg * 1000;

   // [kg/m^3] Atmospheric Density from Standard atmosphere model
   AtmosphereOutput atm = StandardAtmosphere(h_cg_m);

   // [kg/km^3] Convert Denstiy to kg/km^3 for consistent unit system
   atm.density = atm.density * 1e9;

   // [km/s] Velocity Magnitude
   double v_mag = relVel_ENZ.norm();

   // [km*kg/s^2] Aerodynamic Drag Force in ENZ (Opposite Direction of velocity)
   Eigen::Vector3d Fd_ENZ = Eigen::Vector3d::Zero();
   if (v_mag > 0){
      Fd_ENZ = (-0.5 * C.CD * atm.density * C.ar * v_mag) * relVel_ENZ;
   }

   // [km/s^2] Gravitational Acceleration in ECEF (Towards Earth Center)
   Eigen::Vector3d g_ECEF = -C.Gm * CG_R_ECEF / (pow(r_cg, 3));

   // [km/s^2] Gravitational acceleration in ENZ.
   Eigen::Vector3d g_ENZ = ECEF2ENZ * g_ECEF;

   // [km/s^2] Total Acceleration in ENZ
   Eigen::Vector3d a_ENZ = Fd_ENZ / C.mcg + g_ENZ;

   // [km/s, km/s, km/s, km/s^2, km/s^2, km/s^2] State Vector Derivative
   Eigen::VectorXd dSdt(6);
   dSdt << relVel_ENZ, a_ENZ;

   return dSdt;

 }                                            
           