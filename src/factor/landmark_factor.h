#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "../utility.h"
#include "../parameters.h"

class LandMarkFactor : public ceres::SizedCostFunction<2,3,2>
{
public:
    LandMarkFactor(Eigen::Vector2d _obs)
     :obs(_obs) 
     {
         noise.setIdentity();
         noise(0,0) = OBS_R_N * OBS_R_N;
         noise(1,1) = OBS_TH_N * OBS_TH_N;
         sqrt_info = Eigen::LLT<Eigen::Matrix2d>(noise.inverse()).matrixL().transpose();
     }

    virtual bool Evaluate(double const* const* parameters,double* residuals, double** jacobians) const
    {
        Eigen::Vector2d Pr(parameters[0][0],parameters[0][1]);
        Eigen::Matrix2d Rr = Utility::toR(parameters[0][2]);
        Eigen::Vector2d Pf(parameters[1][0],parameters[1][1]);
        
        Eigen::Vector2d PfInr = Rr.transpose()*(Pf - Pr);
        Eigen::Map<Eigen::Vector2d> residual(residuals);
        residual[0] = sqrt(PfInr[0]*PfInr[0]+PfInr[1]*PfInr[1]) - obs[0];
        residual[1] = Utility::normalizeAngle(atan2(PfInr[1],PfInr[0]) - obs[1]);    
        residual = sqrt_info * residual;    

        if(jacobians)
        {
            Matrix2d Jtmp;
            double sqrt_tmp = sqrt(PfInr[0]*PfInr[0]+PfInr[1]*PfInr[1]);
            Jtmp << PfInr[0]/sqrt_tmp, PfInr[1]/sqrt_tmp,
                    -PfInr[1]/(sqrt_tmp*sqrt_tmp),PfInr[0]/(sqrt_tmp*sqrt_tmp);
            double c = cos(parameters[0][2]);
            double s = sin(parameters[0][2]);
            double xr = parameters[0][0];
            double yr = parameters[0][1];
            double xf = parameters[1][0];
            double yf = parameters[1][1];
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,2,3,Eigen::RowMajor>> Jr(jacobians[0]);
                Matrix<double,2,3> Jtr;
                Jtr << -c, -s, (xr-xf)*s+(yf-yr)*c,
                        s, -c, (xr-xf)*c+(yr-yf)*s;
                Jr = Jtmp * Jtr;
                Jr = sqrt_info * Jr;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor>> Jf(jacobians[1]);
                Matrix2d Jtf;
                Jtf << c, s,
                      -s, c;
                Jf = Jtmp * Jtf;
                Jf = sqrt_info * Jf;
            }
        }
        return true;
    }

private:
    Eigen::Vector2d obs;  
    Eigen::Matrix2d noise;  
    Eigen::Matrix2d sqrt_info;
};