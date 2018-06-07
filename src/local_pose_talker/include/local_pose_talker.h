#ifndef LOCAL_POSE_TALKER_H_
#define LOCAL_POSE_TALKER_H_

#include <iostream>
#include <cmath>
#include <memory>
#include <mutex>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


using namespace std;

class local_pose_talker{
public:
    local_pose_talker(string topic_name);
    ~local_pose_talker();
    bool local_pose_pub();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    string pose_topicname_;
    ros::Publisher pose_pub_;
    tf::TransformListener tf_listener_;
    Eigen::Isometry3d local2robot, map2local, map2robot, local2map;
    geometry_msgs::PoseStamped global_pose_, local_pose_, marker_pose_;
    double local_marker_x_,local_marker_y_;
};

#endif