#pragma once
#include <Eigen/Dense>
#include "ProjectileConstant.h"

Eigen::VectorXd ProjectileEOM(double t, const Eigen::VectorXd& S, const ProjectileConstant& C);