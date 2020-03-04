#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "../utility.h"
#include <vector>

class Pose2DLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double* x, const double* delta,double* x_plus_delta) const
    {
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = Utility::normalizeAngle(x[2] + delta[2]);     
        return true;   
    }
    virtual bool ComputeJacobian(const double* x, double* jacobian) const
    {
        Eigen::Map<Eigen::Matrix3d> j(jacobian);
        j.setIdentity();
        return true;
    }
    virtual int GlobalSize() const {return 3; }
    virtual int LocalSize() const {return 3; }
};