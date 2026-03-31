#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "ProjectileConstant.h"

struct EventOutput{
    double value;
    int isterminal;
    int direction;
};

EventOutput ProjectileEvent(double t, const Eigen::VectorXd& S, const ProjectileConstant& C);