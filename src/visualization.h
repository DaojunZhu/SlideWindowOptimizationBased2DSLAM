#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include "utility.h"
#include "estimator.h"
#include "parameters.h"

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_latest_odometry;
extern ros::Publisher pub_key_poses;


void registerPub(ros::NodeHandle &n);
void pubLatestOdometry(const Eigen::Vector2d &P, const Eigen::Matrix2d &R, double t);
void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);
void pubKeyPoses(const Estimator& estimator, const std_msgs::Header &header);
void pubPointCloud(const Estimator& estimator, const std_msgs::Header& header);
void pubTF(const Estimator &estimator, const std_msgs::Header &header);
