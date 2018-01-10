/* Path player
 * Project: Scheduling System
 * Author: Lilan Pan
 * Date:
 * * Created on: 3/8/2017
 * * Add a function for reversing the path: ?
 * Copyright (c) 2017 HRG
*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <scheduling_msgs/PathStampWithID.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <PathPlayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <fstream>

using namespace std;
class path_player{
public:
    path_player(string record_file);
    //~path_player();
    void play_callback(const ros::TimerEvent& event);
    void is_path_reverse_callback(const std_msgs::String rev);
    void reconfigureCB(::path_recorder::PathPlayerConfig &config, uint32_t level);
    void play();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    string path_topicname_;
    ros::Publisher path_pub_;
    ros::Publisher pathwithid_pub_;

    nav_msgs::Path path_;
    scheduling_msgs::PathStampWithID pathwithid_;

    bool is_path_reverse_;
    nav_msgs::Path path_rev_;
    scheduling_msgs::PathStampWithID pathwithid_rev_;
    ros::Subscriber is_path_reverse_sub_;

    geometry_msgs::PoseStamped pose_;
    double pub_freq_;
    ifstream ifs_;
    ros::Timer timer_;

    bool publish_pathID_;
    int pathID_;

    dynamic_reconfigure::Server<path_recorder::PathPlayerConfig> *dsrv_;
};

int main(int argc,char** argv){
    ros::init(argc,argv,"path_recorder");
    ros::NodeHandle pnh("~");
    string pathfilename;
    pnh.param<string>("pathfilename",pathfilename,"/home/hitrobot/catkin_ws/path0.txt");
    path_player pp(pathfilename);
    pp.play();
    ros::spin();
}

path_player::path_player(string record_file):nh_(""),pnh_("~"),ifs_(record_file.c_str()){
    pnh_.param<string>("path_topicname",path_topicname_,"specified_path");
    pnh_.param<bool>("reverse_path",is_path_reverse_,false);
    pnh_.param<bool>("publish_path_ID",publish_pathID_,true);
    pnh_.param<int>("pathID",pathID_,1);
    pnh_.param("publish_frequency",pub_freq_,1.0);
    if(!ifs_.is_open())
        ROS_ERROR("Cannot open record file");
    double pose_value;

    pose_.header.frame_id="map";
    path_.header.frame_id="map";
    pathwithid_.header.frame_id="map";
    pathwithid_.pathID=pathID_;

    while(!ifs_.eof()){
        ifs_>>pose_value;
        if(ifs_.eof()) //for empty line
            break;
        pose_.pose.position.x=pose_value;
        ifs_>>pose_value;
        pose_.pose.position.y=pose_value;
        ifs_>>pose_value;
        pose_.pose.orientation.z=pose_value;
        ifs_>>pose_value;
        pose_.pose.orientation.w=pose_value;
        path_.poses.push_back(pose_);
        pathwithid_.poses.push_back(pose_);
    }
    //Create the reverse path
    path_rev_.header.frame_id="map";
    pathwithid_rev_.header.frame_id="map";
    pathwithid_rev_.pathID=-pathID_;

    int num_poses=path_.poses.size();
    for(int i=0;i<num_poses;i++){
        pose_=path_.poses[num_poses-1-i];
        pose_.pose.orientation.z=-pose_.pose.orientation.z;
        path_rev_.poses.push_back(pose_);
        pathwithid_rev_.poses.push_back(pose_);
    }

    bool latched=false;
    if(pub_freq_==0.0)
        latched=true;
    is_path_reverse_sub_=nh_.subscribe("play_reverse_path",1,&path_player::is_path_reverse_callback,this);
    path_pub_=nh_.advertise<nav_msgs::Path>(path_topicname_+"_gui",1,latched);
    if(publish_pathID_)
        pathwithid_pub_=nh_.advertise<scheduling_msgs::PathStampWithID>(path_topicname_,1,latched);

    //dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<path_recorder::PathPlayerConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<path_recorder::PathPlayerConfig>::CallbackType cb = boost::bind(&path_player::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void path_player::play_callback(const ros::TimerEvent &event){
    if(!is_path_reverse_){
        if(publish_pathID_)
            pathwithid_pub_.publish(pathwithid_);
        path_pub_.publish(path_);
    }else{
        if(publish_pathID_)
            pathwithid_pub_.publish(pathwithid_rev_);
        path_pub_.publish(path_rev_);
    }
}

void path_player::is_path_reverse_callback(const std_msgs::String rev){
    if(rev.data=="True" || rev.data=="true" || rev.data=="TRUE"){
        is_path_reverse_=true;
    }else if(rev.data=="False" || rev.data=="false" || rev.data=="FALSE"){
        is_path_reverse_=false;
    }
}

void path_player::reconfigureCB(path_recorder::PathPlayerConfig &config, uint32_t level){
    is_path_reverse_=config.path_reversed;
    if(is_path_reverse_==true)
        ROS_INFO("Inverse Path");
    else
        ROS_INFO("Forward Path");
}

void path_player::play(){
    ROS_INFO("Start playing the path from the record file");
    if(pub_freq_==0.0){
        if(!is_path_reverse_){
            if(publish_pathID_)
                pathwithid_pub_.publish(pathwithid_);
            path_pub_.publish(path_);
        }else{
            if(publish_pathID_)
                pathwithid_pub_.publish(pathwithid_rev_);
            path_pub_.publish(path_rev_);
        }
    }else
        timer_=nh_.createTimer(ros::Duration(1.0/pub_freq_),&path_player::play_callback,this);
}
