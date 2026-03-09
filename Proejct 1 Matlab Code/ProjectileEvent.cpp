#include <Eigen/Dense>
#include <cmath>
#include "ProjectileConstant.h"
#include "ProjectileEvent.h"


 EventOutput ProjectileEvent(double t, const Eigen::VectorXd& S, const ProjectileConstant& C){
    
    // [km, km, km] Projectile Position Relative to the Ground Station in ENZ
    Eigen::Vector3d relPos_ENZ = S.segment<3>(0);

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

   // [km, km, km] Projectile Position WRT Earth in ECEF
   Eigen::Vector3d CG_R_ECEF = GS_R_ECEF + ENZ2ECEF * relPos_ENZ;

   // [km] Magnitude of projectile position vector from Earth's Center
   double r_cg = CG_R_ECEF.norm();

   // [km] Altitude of Projectile above Earth's Mean Radius
   double h_cg = r_cg - C.Re;

   // [km] Event Value: Altitude relative to initial altitude
   double value = h_cg - C.h_gs;

   // [] Flag to make this a terminal event (1 = Stop Integration when this event occurs)
   int isterminal = 1;

   // [] Direction of zero crossing to Detect (-1 = Decreasing, Projectile Returning)
   int direction = -1;

   return {value, isterminal, direction};
}