#include "local_pose_talker.h"

int main(int argc,char** argv){
    ros::init(argc,argv,"local_pose_talker");
    ros::NodeHandle pnh("~");
    string topicname;
    pnh.param<string>("topicname",topicname,"target_mark");
    local_pose_talker pp(topicname);
    ros::spin();
    return 0;
}

local_pose_talker::local_pose_talker(string topic_name): nh_(""), pnh_("~")
{
    pose_topicname_ = topic_name;
    pnh_.param<double>("local_marker_x",local_marker_x_,1.0);
    pnh_.param<double>("local_marker_y",local_marker_y_,1.0);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topicname_,1,true);

    marker_pose_.header.frame_id = "/map";
    marker_pose_.header.stamp = ros::Time::now();
    marker_pose_.pose.position.x = local_marker_x_;
    marker_pose_.pose.position.y = local_marker_y_;
    marker_pose_.pose.orientation.w = 1;

    tf::poseMsgToEigen(marker_pose_.pose,map2local);
    local2map = map2local.inverse();

}

local_pose_talker::~local_pose_talker()
{}

bool local_pose_talker::local_pose_pub()
{   
    // tf::StampedTransform robot_pose;
    // geometry_msgs::TransformStamped robot_pose_;
    // try{
    //     tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0.0), robot_pose);
    // }
    // catch (tf::TransformException& ex){
    //     ROS_ERROR("%s",ex.what());
    //     return false;
    // }
    // tf::transformStampedTFToMsg(robot_pose,robot_pose_);
    // tf::transformMsgToEigen(robot_pose_.transform, map2robot);
    // local2robot = local2map * map2robot;

    // local_pose_.header.frame_id = "/triangle_pose_stamped";
    // local_pose_.header.stamp = ros::Time::now();
    // tf::poseEigenToMsg(local2robot, local_pose_.pose);
    pose_pub_.publish(marker_pose_);

    return true;
}