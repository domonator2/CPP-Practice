// This function serves to establish the EOMs for the simulation.

#include "ProjectileEOM.h"
#include <Eigen/Dense>
#include <cmath>

 Eigen::VectorXd ProjectileEOM(double t, const Eigen::VectorXd& S, const ProjectileConstant& C) {
    
    // [km, km, km] Projectile Position relative to Ground Station
    Eigen::Vector3d relPos_ENZ = S.segment<3>(0);

    // [km/s, km/s, km/s] Projectile Velocity Relative to Ground Station in ENZ
    Eigen::Vector3d relVel_ENZ = S.segment<3>(3);

    

 }                                            
           