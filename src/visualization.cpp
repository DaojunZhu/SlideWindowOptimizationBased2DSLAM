#include "visualization.h"

ros::Publisher pub_odometry;
ros::Publisher pub_latest_odometry;
ros::Publisher pub_key_poses;
ros::Publisher pub_point_cloud;

void registerPub(ros::NodeHandle& n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("odom_propagate",1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry",1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
}

void pubLatestOdometry(const Eigen::Vector2d &P, const Eigen::Matrix2d &R, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(Utility::fromR(R));
    pub_latest_odometry.publish(odometry);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if(estimator.frame_count == WINDOW_SIZE)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                Utility::fromR(estimator.Rs[WINDOW_SIZE]));
        pub_odometry.publish(odometry);
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if(estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();
    key_poses.id = 0;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;
    for(int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector2d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = 0.0;
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubPointCloud(const Estimator& estimator, const std_msgs::Header& header)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = header;    

    for(auto& it_per_id : estimator.f_manager.features)
    {
        geometry_msgs::Point32 p;
        p.x = it_per_id.pose.x();
        p.y = it_per_id.pose.y();
        p.z = 0.0;
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if(estimator.frame_count != WINDOW_SIZE)
        return;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    Vector2d correct_t;
    Matrix2d correct_R;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_R = estimator.Rs[WINDOW_SIZE];
    
    transform.setOrigin(tf::Vector3(correct_t(0),correct_t(1),0.0));
    transform.setRotation( tf::createQuaternionFromYaw(Utility::fromR(correct_R)));
    br.sendTransform(tf::StampedTransform(transform,header.stamp,"world","robot"));
}