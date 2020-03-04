#pragma once

#include <eigen3/Eigen/Core>
#include <list>
#include <vector>
using namespace std;
using namespace Eigen;

class FeaturePerFrame
{
public:
    FeaturePerFrame(const Vector2d& feature_obs)
     : range{feature_obs(0)},theta{feature_obs(1)} {}

    double range;
    double theta;
};

class FeaturePerId
{
public:
    FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id),start_frame(_start_frame),
        pose(0,0){}

    int endFrame();

    const int feature_id;
    int start_frame;
    std::vector<FeaturePerFrame> feature_per_frame;
    Vector2d pose;  
};

class FeatureManager
{
public:
    int getFeatureCount();
    MatrixXd getFeaturePose();
    void setFeaturePose(const MatrixXd& f_pose);
    void addFeature(int frame_count ,const std::vector<pair<int,Vector2d> > obs);
    void initializeNewFeaturePose(int frame_count, const Matrix2d& R, const Vector2d& P);
    
    list<FeaturePerId> features;
    int last_track_num;
    int new_feature_num;
};