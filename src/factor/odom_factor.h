#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "odom_integration.h"

class OdomFactor : public ceres::SizedCostFunction<3,3,3>
{
public:
    OdomFactor(OdomIntegration* _odom_integration)
      : odom_integration(_odom_integration){}
    virtual bool Evaluate(double const* const* parameters, double* residuals,double** jacobians) const
    {
        Eigen::Vector2d Pi(parameters[0][0],parameters[0][1]);
        Eigen::Matrix2d Ri = Utility::toR(parameters[0][2]);
        Eigen::Vector2d Pj(parameters[1][0],parameters[1][1]);
        Eigen::Matrix2d Rj = Utility::toR(parameters[1][2]);
        
        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.head(2) = Ri.transpose()*(Pj-Pi) - odom_integration->delta_odom.head(2);
        residual(2) = Utility::normalizeAngle( parameters[1][2] - parameters[0][2] - odom_integration->delta_odom(2));
        Eigen::Matrix3d sqrt_info = Eigen::LLT<Eigen::Matrix3d>(odom_integration->covariance.inverse()).matrixL().transpose();
        residual = sqrt_info * residual;

        if(jacobians)
        {
            double xi = Pi(0);
            double yi = Pi(1);
            double phii = parameters[0][2];
            double c = cos(phii);
            double s = sin(phii);
            double xj = Pj(0);
            double yj = Pj(1);
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,3,3,RowMajor>> Ji(jacobians[0]);
                Ji.setZero();
                Ji << -c, -s, (xi-xj)*s + (yj-yi)*c,
                        s, -c, (xi-xj)*c + (yi-yj)*s,
                        0,  0,   -1;
                Ji = sqrt_info * Ji;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double,3,3,RowMajor>>  Jj(jacobians[1]);
                Jj << c, s, 0,
                      -s, c, 0,
                      0, 0, 1;  
                Jj = sqrt_info * Jj;
            }
        }
        return true;
    }
private:
    OdomIntegration* odom_integration;
};