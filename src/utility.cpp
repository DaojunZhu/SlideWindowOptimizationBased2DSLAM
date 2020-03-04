#include "utility.h"

Matrix2d Utility::toR(double phi)
{
    Matrix2d R;
    R << cos(phi), -sin(phi),
        sin(phi), cos(phi);
    return R;
}

double Utility::fromR(const Matrix2d& mat)
{
    return atan2(mat(1,0),mat(0,0));
}

double Utility::normalizeAngle(double theta)
{
    if (theta >= -M_PI && theta < M_PI)
            return theta;

    double multiplier = floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;

    return theta;
    //return std::atan2(sin(theta),cos(theta));
}

Matrix2d Utility::skewSymmetric(double phi)
{
    Eigen::Matrix2d mat;
    mat << 0, -phi,
            phi, 0;
    return mat;
}