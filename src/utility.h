#pragma once

#include <eigen3/Eigen/Core>
using namespace Eigen;

class Utility
{
public:
    static Matrix2d  toR(double phi);
    static double    fromR(const Matrix2d& mat);
    static double    normalizeAngle(double angle);
    static Matrix2d  skewSymmetric(double phi);
};