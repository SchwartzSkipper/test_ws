/* Global planner plugin receiving a specified path published by other node
 * Project: Scheduling System
 * Author: Lilan Pan
 * Date:
 * * Created on: 4/8/2017
 * * Introducing pathID: 9/8/2017
 * * Add dynamic configuration: 29/8/2017
 * * Allow to follow the direction of goal: 31/8/2017
 * * Add set and clear planner path service: 19/12/2017
 * Copyright (c) 2017 HRG
*/

#include <global_planner/fixed_planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <fixed_global_planner/FixedGlobalPlannerConfig.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::FixedGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {
void FixedGlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

FixedGlobalPlanner::FixedGlobalPlanner():costmap_(NULL), initialized_(false), allow_unknown_(true) {}

FixedGlobalPlanner::FixedGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true),nh_(""){
    //initialize the planner
    initialize(name, costmap, frame_id);
}

FixedGlobalPlanner::~FixedGlobalPlanner() {}

void FixedGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void FixedGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle pnh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        pnh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        pnh.param("allow_unknown", allow_unknown_, true);
        pnh.param("planner_window_x", planner_window_x_, 0.0);
        pnh.param("planner_window_y", planner_window_y_, 0.0);
        pnh.param<std::string>("subscribe_path_topic",sub_path_topic_,"specified_path");
        pnh.param<std::string>("set_planner_path_service",set_planner_path_service_name_,"set_planner_path");
        pnh.param("endpoint_tolerance",endpoint_tolerance_,1.0);
        pnh.param("error_retry_time",error_retry_time_,0.2);
        pnh.param<bool>("retrace_path",retrace_path_,false);
        pnh.param<bool>("include_start_pose",include_start_pose_,false);
        pnh.param<bool>("use_goal_direction",use_goal_direction_,true);
        pnh.param<bool>("dynamic_reconfigure_subscribed_path_topic",dynamic_reconfigure_subscribed_path_topic_,true);

        pnh.param<bool>("always_update_path",always_update_path_,true);

        path_sub_=nh_.subscribe(sub_path_topic_,2,&FixedGlobalPlanner::pathCallback,this);
        set_planner_path_service_=nh_.advertiseService(set_planner_path_service_name_,&FixedGlobalPlanner::setPlannerPathServiceCallback,this);
        clear_planner_path_service_=nh_.advertiseService("clear_planner_path",&FixedGlobalPlanner::clearPlannerPathServiceCallback,this);

        plan_pub_ = pnh.advertise<nav_msgs::Path>("plan", 1);
        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        dsrv_ = new dynamic_reconfigure::Server<::fixed_global_planner::FixedGlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<::fixed_global_planner::FixedGlobalPlannerConfig>::CallbackType cb = boost::bind(&FixedGlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void FixedGlobalPlanner::reconfigureCB(fixed_global_planner::FixedGlobalPlannerConfig& config, uint32_t level) {
    allow_unknown_=config.allow_unknown;
    endpoint_tolerance_=config.endpoint_tolerance;
    retrace_path_=config.retrace_path;
    use_goal_direction_=config.use_goal_direction;
    if(sub_path_topic_!=config.subscribe_path_topic){
        if(dynamic_reconfigure_subscribed_path_topic_){
            boost::mutex::scoped_lock lock(mutex_);
            sub_path_topic_=config.subscribe_path_topic;
            ROS_WARN("Switch to subscribe new path topic: %s",sub_path_topic_.c_str());
            path_sub_.shutdown();
            path_sub_=nh_.subscribe(sub_path_topic_,2,&FixedGlobalPlanner::pathCallback,this);
        }else{
            ROS_WARN("You have set the dynamic_reconfigure_subscribed_path_topic option off, the default subscribed topic is %s",sub_path_topic_.c_str());
        }
    }
}

void FixedGlobalPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void FixedGlobalPlanner::pathCallback(scheduling_msgs::PathStampWithID path){
    ROS_DEBUG("Received a path");
    //ROS_WARN("specified_path_.pathID: %d, path.pathID: %d",specified_path_.pathID,path.pathID);
    if(always_update_path_ || specified_path_.pathID!=path.pathID){
        boost::mutex::scoped_lock lock(mutex_);
        specified_path_=path;
        //for reducing the data communication, the pose frame_id can be same to path frame_id
        for(int i=0;i<specified_path_.poses.size();i++){
            if(specified_path_.poses[i].header.frame_id.empty())
                specified_path_.poses[i].header.frame_id=specified_path_.header.frame_id;
        }
        ROS_INFO("Received a new path with ID %d",specified_path_.pathID);
    }
}

bool FixedGlobalPlanner::setPlannerPathServiceCallback(scheduling_msgs::SetPlannerPath::Request& req, scheduling_msgs::SetPlannerPath::Response& resp){
    if(req.poses.size()==0){
        ROS_WARN("Receive a setPlannerPath request without any planner pose data");
        resp.feedback=0;
    }else{
        specified_path_.priority=req.priority;
        specified_path_.pathID=req.pathID;
        specified_path_.header.frame_id=req.header.frame_id;
        specified_path_.header.stamp=ros::Time::now();
        specified_path_.poses.clear();

        geometry_msgs::PoseStamped pose_stamped;
        boost::mutex::scoped_lock lock(mutex_);
        pose_stamped.header.frame_id=req.header.frame_id;
        for(geometry_msgs::Pose2D pose:req.poses){
            pose_stamped.pose.position.x=pose.x;
            pose_stamped.pose.position.y=pose.y;
            pose_stamped.pose.orientation.z=sin(pose.theta/2);
            pose_stamped.pose.orientation.w=cos(pose.theta/2);
            specified_path_.poses.push_back(pose_stamped);
        }
        ROS_INFO("Received a new set_planner_path request with ID %d",specified_path_.pathID);
        resp.feedback=1;
    }
    return true;
}

bool FixedGlobalPlanner::clearPlannerPathServiceCallback(scheduling_msgs::ClearPlannerPath::Request& req, scheduling_msgs::ClearPlannerPath::Response& resp){
    if(req.pathID==specified_path_.pathID){
        specified_path_.poses.clear();
        specified_path_.pathID=-1;
        specified_path_.priority=0;
        specified_path_.flags=0;
        specified_path_.header.frame_id="";
        resp.feedback=1.0;
        ROS_INFO("Clear the path with pathID %d",req.pathID);
    }else{
        ROS_ERROR("The pathID of clear_planner_path service does not match to the ID of planner path");
        resp.feedback=0;
    }
    return true;
}

void FixedGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool FixedGlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool FixedGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, endpoint_tolerance_, plan);
}

bool FixedGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    int pathlen=specified_path_.poses.size();
    if(pathlen==0){
        ROS_ERROR("Specified path not given");
        ros::Duration(error_retry_time_).sleep();
        return false;
    }
    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        ros::Duration(error_retry_time_).sleep();
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        ros::Duration(error_retry_time_).sleep();
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //As that we may edit the path, a copy of original path should be made

    scheduling_msgs::PathStampWithID execute_path=specified_path_;

    if(!retrace_path_){
        if(poseDist(start,execute_path.poses[0])>endpoint_tolerance_){
            ROS_ERROR("Path %d: The position of robot is too far from the start point of specified path",specified_path_.pathID);
            ros::Duration(error_retry_time_).sleep();
            return false;
        }
        if(poseDist(goal,execute_path.poses[pathlen-1])>endpoint_tolerance_){
            ROS_ERROR("Path %d: The goal is too far from the end point of specified path",specified_path_.pathID);
            ros::Duration(error_retry_time_).sleep();
            return false;
        }
    }else{
        double curr_dist=0.0;
        double min_dist=1000.0;
        int start_min_index=0;
        int end_min_index=0;
        std::vector<geometry_msgs::PoseStamped>::iterator pit;
        for(int i=0;i<pathlen-1;i++){
            curr_dist=poseDist(start,execute_path.poses[i]);
            if(curr_dist<min_dist){
                min_dist=curr_dist;
                start_min_index=i;
            }
        }
        if(min_dist>endpoint_tolerance_){
            ROS_ERROR("Path %d: Cannot find any pose close to the start point in the specified path",specified_path_.pathID);
            ROS_ERROR("The cloest point on path to the start point (%f, %f) is (%f, %f), distance: %f",start.pose.position.x,start.pose.position.y,execute_path.poses[start_min_index].pose.position.x,execute_path.poses[start_min_index].pose.position.y,min_dist);
            ros::Duration(error_retry_time_).sleep();
            return false;
        }
        min_dist=1000.0;
        for(int i=0;i<pathlen-1-start_min_index;i++){
            curr_dist=poseDist(goal,execute_path.poses[pathlen-1-i]);
            if(curr_dist<min_dist){
                min_dist=curr_dist;
                end_min_index=i;//for convenience, we set the end_min_index from end to goal
            }
        }
        if(min_dist>endpoint_tolerance_){
            ROS_ERROR("Path %d: Cannot find any pose close to the goal point in the specified path",specified_path_.pathID);
            ROS_ERROR("The cloest point on path to the goal point (%f, %f) is (%f, %f), distance: %f",goal.pose.position.x,goal.pose.position.y,execute_path.poses[pathlen-1-end_min_index].pose.position.x,execute_path.poses[pathlen-1-end_min_index].pose.position.y,min_dist);
            ros::Duration(error_retry_time_).sleep();
            return false;
        }
        //crop path
        for(int i=0;i<end_min_index;i++)
            execute_path.poses.pop_back();

        pit=execute_path.poses.begin();
        for(int i=0;i<start_min_index;i++)
            pit=execute_path.poses.erase(pit);

        if(include_start_pose_){
            execute_path.poses.insert(execute_path.poses.begin(),start);
        }
        if(use_goal_direction_){
            geometry_msgs::PoseStamped goal_pose=execute_path.poses[execute_path.poses.size()-1];
            goal_pose.pose.orientation.z=goal.pose.orientation.z;
            goal_pose.pose.orientation.w=goal.pose.orientation.w;
            execute_path.poses.push_back(goal_pose);
        }
        pathlen=execute_path.poses.size();
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    double pose_wx,pose_wy;//path coordinate frame is world but costmap is in map coordinate frame
    double pose_mx,pose_my;
    for(int i=0;i<pathlen;i++){
        pose_wx=execute_path.poses[i].pose.position.x;
        pose_wy=execute_path.poses[i].pose.position.y;
        worldToMap(pose_wx,pose_wy,pose_mx,pose_my);
        if(costmap_->getCost(pose_mx,pose_my)==costmap_2d::LETHAL_OBSTACLE){
            ROS_INFO("There are lethal obstacles on the specified path");
            plan.clear();
            return false;
        }
        plan.push_back(execute_path.poses[i]);
    }

    //publish the plan for visualization purposes
    publishPlan(plan);

    return !plan.empty();
}

void FixedGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }
    plan_pub_.publish(gui_path);
}

double FixedGlobalPlanner::poseDist(const geometry_msgs::PoseStamped pose0,const geometry_msgs::PoseStamped pose1){
    return sqrt((pose0.pose.position.x-pose1.pose.position.x)*(pose0.pose.position.x-pose1.pose.position.x)+
                (pose0.pose.position.y-pose1.pose.position.y)*(pose0.pose.position.y-pose1.pose.position.y));
}
} //end namespace global_planner

