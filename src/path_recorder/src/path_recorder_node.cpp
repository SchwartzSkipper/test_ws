/* Path Recorder
 * Project: Scheduling System
 * Author: Lilan Pan
 * Date:
 * * Created on: 3/8/2017
 * Copyright (c) 2017 HRG
*/
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>

using namespace std;
class path_recorder{
public:
    path_recorder(string record_file);
    ~path_recorder();
    void callback(geometry_msgs::PoseWithCovarianceStamped pose);
    void init(string pose_topic="robot_pose");
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber pose_sub_;
    double min_record_dist_;
    double min_record_angle_;
    vector<geometry_msgs::Pose> vPath_;
    ofstream ofs_;
};

int main(int argc,char** argv){
    ros::init(argc,argv,"path_recorder");
    ros::NodeHandle pnh("~");
    string pathfilename;
    string posetopic;
    pnh.param<string>("pathfilename",pathfilename,"/home/hitrobot/catkin_ws/path.txt");
    pnh.param<string>("posetopic",posetopic,"robot_pose");
    path_recorder pr(pathfilename);
    pr.init(posetopic);
    ros::spin();
}

path_recorder::path_recorder(string record_file):nh_(""),pnh_("~"),ofs_(record_file.c_str(),ofstream::out){
    pnh_.param("min_record_dist",min_record_dist_,0.05);
    pnh_.param("min_record_angle",min_record_angle_,0.1);
}

path_recorder::~path_recorder(){
    ofs_.close();
}

void path_recorder::callback(geometry_msgs::PoseWithCovarianceStamped pose){
    if(vPath_.empty()){
        vPath_.push_back(pose.pose.pose);
        ofs_<<pose.pose.pose.position.x<<" "<<pose.pose.pose.position.y<<" "
           <<pose.pose.pose.orientation.z<<" "<<pose.pose.pose.orientation.w<<endl;
    }else{
        geometry_msgs::Pose prev_pose=vPath_[vPath_.size()-1];
        if(sqrt((pose.pose.pose.position.x-prev_pose.position.x)*(pose.pose.pose.position.x-prev_pose.position.x)+(pose.pose.pose.position.y-prev_pose.position.y)*(pose.pose.pose.position.y-prev_pose.position.y))>min_record_dist_
          || fabs(atan2(pose.pose.pose.orientation.z,pose.pose.pose.orientation.w)-atan2(prev_pose.orientation.z,prev_pose.orientation.w))>0.5*min_record_angle_){
            cout<<"Recorded a pose at ("<<pose.pose.pose.position.x<<", "<<pose.pose.pose.position.y<<", "<<2*atan2(pose.pose.pose.orientation.z,pose.pose.pose.orientation.w)<<")"<<endl;
            vPath_.push_back(pose.pose.pose);
            ofs_<<pose.pose.pose.position.x<<" "<<pose.pose.pose.position.y<<" "
               <<pose.pose.pose.orientation.z<<" "<<pose.pose.pose.orientation.w<<endl;
        }
    }
}

void path_recorder::init(string pose_topic){
    pose_sub_=nh_.subscribe(pose_topic,1,&path_recorder::callback,this);
}
