#include <ros/ros.h>
#include <slam_data_sim/RangeBearingObsData.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "parameters.h"
#include "estimator.h"
#include <eigen3/Eigen/Core>
#include "visualization.h"
using namespace Eigen;
using namespace std;

Estimator estimator;

nav_msgs::Odometry last_odom;
bool first_odom_flag = false;

void feature_callback(const slam_data_sim::RangeBearingObsDataPtr& features_msg)
{
    vector<pair<int, Vector2d>> observations;
    int feature_num = features_msg->ids.size();
    for(int i=0; i<feature_num; i++)
    {
        int id = features_msg->ids[i];
        Vector2d feature_obs(features_msg->ranges[i],features_msg->angles[i]);
        observations.push_back(make_pair(id,feature_obs));
    }
    estimator.inputFeature(features_msg->header.stamp.toSec(),observations);
}

void odom_callback(const nav_msgs::OdometryPtr& odom_msg)
{
    if(!first_odom_flag)
    {
        first_odom_flag = true;
        last_odom = *odom_msg;
        return;
    }
    double t = odom_msg->header.stamp.toSec();
    double dx = odom_msg->pose.pose.position.x - last_odom.pose.pose.position.x;
    double dy = odom_msg->pose.pose.position.y - last_odom.pose.pose.position.y;
    Vector2d dT (dx,dy);
    dT = Utility::toR(tf::getYaw(last_odom.pose.pose.orientation)).transpose() * dT;
    double dth = Utility::normalizeAngle(tf::getYaw(odom_msg->pose.pose.orientation) - tf::getYaw(last_odom.pose.pose.orientation));
    Vector3d delta_odom(dT(0),dT(1),dth);
    estimator.inputOdom(t,delta_odom);
    last_odom = *odom_msg;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"slam_2d_sim");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

    nh.param("odom_topic",ODOM_TOPIC,std::string{"odom"});
    nh.param("feature_topic",FEATURE_TOPIC,std::string{"observations"});
    nh.param("odom_x_n",ODOM_X_N,0.0001);
    nh.param("odom_y_n",ODOM_Y_N,0.0001);
    nh.param("odom_th_n",ODOM_TH_N,0.01);
    nh.param("obs_r_n",OBS_R_N,0.01);
    nh.param("obs_th_r",OBS_TH_N,0.01);
    nh.param("num_iterations",NUM_ITERATIONS,8);

    registerPub(nh);

    ros::Subscriber sub_feature = nh.subscribe(FEATURE_TOPIC,100,feature_callback);
    ros::Subscriber sub_odom = nh.subscribe(ODOM_TOPIC,2000,odom_callback);

    ros::spin();
    return 0;
}