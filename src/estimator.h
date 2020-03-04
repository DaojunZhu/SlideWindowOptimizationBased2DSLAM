#pragma once

#include <ros/ros.h>
#include <ros/time.h>

#include "parameters.h"
#include "utility.h"
#include "factor/odom_integration.h"
#include "feature_manager.h"
#include "factor/pose2d_local_parameterization.h"
#include "factor/odom_factor.h"
#include "factor/landmark_factor.h"
#include "factor/marginalization_factor.h"
#include <eigen3/Eigen/Dense>

#include <thread>
#include <mutex>
#include <queue>
#include <ceres/ceres.h>
using namespace Eigen;
using namespace std;

class Estimator
{
public:
    Estimator();
    ~Estimator();
    void initFirstPose(Eigen::Vector2d p,Eigen::Matrix2d r);
    void inputFeature(double t, const std::vector<std::pair<int,Eigen::Vector2d>>& features);
    void inputOdom(double t,const Eigen::Vector3d& d_odom_meas); 
    void processMeasurements();
    bool odomAvailable(double t);
    bool getOdomInterval(double t0,double t1, vector<pair<double,Vector3d>>& odomVector);
    void processOdom(double t, const Eigen::Vector3d& delta_odom);
    void processLandMarks(double t, const vector<pair<int,Vector2d>>& observations);
    void updateLatestStates();
    void fastPredictOdom(const Eigen::Vector3d& delta_odom);

    void clearState();
    void optimization();
    void slideWindow();
    void vector2double();
    void double2vector();

    std::thread processThread;
    std::mutex mBuf;
    std::mutex mPropagate;
    std::queue<pair<double, Vector3d>> odomBuf;
    std::queue<pair<double, vector<pair<int,Vector2d>>>> featureBuf;
    double preTime,curTime; //上一帧特征点测量和当前帧特征点测量的时间戳
 
    //range传感器与里程计的外参
    Matrix2d rbc;
    Vector2d tbc;

    Vector2d Ps[(WINDOW_SIZE + 1)];
    Matrix2d Rs[(WINDOW_SIZE + 1)];
    OdomIntegration* pre_odom_integrations[(WINDOW_SIZE + 1)];

    double para_pose[WINDOW_SIZE + 1][3];
    double para_feature[NUM_OF_F][2];

    double latest_time;
    Eigen::Vector2d latest_P;
    Eigen::Matrix2d latest_R;

    vector<Eigen::Vector2d> key_poses;

    int frame_count;

    FeatureManager f_manager;

    MarginalizationInfo* last_marginalization_info;
    vector<double*> last_marginalization_parameter_blocks;

    ros::Time time_0;
    ros::Time time_1;
    double elapsed_time;
    
};