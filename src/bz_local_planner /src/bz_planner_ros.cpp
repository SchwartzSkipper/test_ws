/*********************************************************************
* Author: GrayLoo
*********************************************************************/

#include <bz_local_planner/bz_planner_ros.h>

PLUGINLIB_EXPORT_CLASS(bz_local_planner::BZPlannerROS, nav_core::BaseLocalPlanner)

namespace bz_local_planner{

BZPlannerROS::BZPlannerROS() : odom_helper_("odom"),
                               initialized_(false),
                               sim_time_(1.7),
                               sim_granularity_(0.025),
                               stop_time_buffer_(0.2),
                               bz_length_tolerance_(0.02),
                               x_tolerance_(0.05),
                               y_tolerance_(0.05),
                               xy_tolerance_(0.05),
                               yaw_tolerance_(0.05),
                               goal_reach_level_(3),
                               x_offset_pos_(1.0),
                               x_offset_neg_(1.0),
                               source_u_(0.3),
                               target_v_(0.5),
                               current_tolerance_(1024.0)                   
{}

BZPlannerROS::~BZPlannerROS()
{}

void BZPlannerROS::initialize(std::string name, 
                              tf::TransformListener* tf, 
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        ros::NodeHandle pnh("~/" + name);
        bz_plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
        bz_ctrl_points_pub_ = pnh.advertise<nav_msgs::Path>("ctrl_points", 1);
        local_plan_pub_ = pnh.advertise<nav_msgs::Path>("local_plan", 1);
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ros_->getRobotPose(current_pose_);
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        planner_util_.initialize(tf_, costmap_, costmap_ros_->getGlobalFrameID());
        obstacle_cost_ = std::make_shared<base_local_planner::ObstacleCostFunction>(costmap_);
        obstacle_cost_->prepare();
        if (pnh.getParam("odom_topic", odom_topic_))
        {
            odom_helper_.setOdomTopic(odom_topic_);
        }
        dyn_server_ = std::make_shared<dynamic_reconfigure::Server<bz_local_planner::BZPlannerConfig>>(pnh);
        dynamic_reconfigure::Server<bz_local_planner::BZPlannerConfig>::CallbackType cb;
        cb = boost::bind(&BZPlannerROS::reconfigureCB, this, _1, _2);
        dyn_server_->setCallback(cb);

        std::function<bool(void)> f;
        f = std::bind(&BZPlannerROS::goalReachBZLen, this);
        is_goal_reach_funcs_.push_back(f);
        f = std::bind(&BZPlannerROS::goalReachXY, this);
        is_goal_reach_funcs_.push_back(f);
        f = std::bind(&BZPlannerROS::goalReachEucDis, this);
        is_goal_reach_funcs_.push_back(f);
        f = std::bind(&BZPlannerROS::goalReachXYYaw, this);
        is_goal_reach_funcs_.push_back(f);
        f = std::bind(&BZPlannerROS::goalReachEucDisYaw, this);
        is_goal_reach_funcs_.push_back(f);

        initialized_ = true;
        ROS_INFO("Initialized bezier local planner.");
    }
    else
    {
        ROS_WARN("Bezier planner has already been initialized, doing nothing.");
    }   
}

bool BZPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)
    {
        ROS_ERROR("Bezier planner has not been initialized");
        return false;
    }
    if (planner_util_.setPlan(plan))
    {
    	global_plan_.clear();
    	global_plan_ = plan;
        // reset current tolerance
        current_tolerance_ = 1024.0;
        tf::Stamped<tf::Pose> goal_pose;
        planner_util_.getGoal(goal_pose);
        geometry_msgs::PoseStamped pose;
        tf::poseStampedTFToMsg(goal_pose, pose);
        try
        {
            tf_->waitForTransform("/map", goal_pose.frame_id_, ros::Time(0), ros::Duration(5.0));
            tf_->transformPose("/map", pose, goal_); 
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            exit(1);
        }
        return true;
    }
    return false;
}

bool BZPlannerROS::getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
      *tf_,
      global_plan_,
      global_pose,
      *costmap_,
      global_frame_,
      transformed_plan)) 
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }
  return true;
}

bool BZPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if (!costmap_ros_->getRobotPose(current_pose_))
    {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!getLocalPlan(current_pose_,transformed_plan))  
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("bz_local_planner", "Received an empty transformed plan.");
      return false;
    }

    goal_ = transformed_plan.back();

    std::lock_guard<std::mutex> lk(dyn_params_mutex_);
    std::vector<geometry_msgs::Pose2D> bz_ctrl_points;
    calcBezierVel(cmd_vel, bz_ctrl_points);
    publishBezierPlan(bz_ctrl_points);
    publishBezierCtrlPoints(bz_ctrl_points);
    base_local_planner::Trajectory result_traj;
    generateSimTraj(cmd_vel, result_traj);
    double cost = calcObstacleCost(result_traj, costmap_ros_->getRobotFootprint());
    result_traj.cost_ = cost;
    publishLocalPlan(result_traj);
    ROS_INFO("Vel x: %.2lf, y: %.2lf, yaw: %.2lf, cost: %.2lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cost);
    if (cost < 0)
    {
        // still return true but command to stop, 
        // since we do not care about the global plan, 
        // neither do we want global planner to make a new plan
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    return true;
}

bool BZPlannerROS::isGoalReached()
{   
	std::lock_guard<std::mutex> lk(dyn_params_mutex_);
    if (!costmap_ros_->getRobotPose(current_pose_))
    {
        ROS_ERROR("Could not get robot pose, will stop");
        return true;
    }
	auto goal_reach_func = is_goal_reach_funcs_[goal_reach_level_];
	if (goal_reach_func())
	{
		ROS_INFO("Goal reached, level: %d", goal_reach_level_);
		return true;
	}
	return false;
}

void BZPlannerROS::reconfigureCB(BZPlannerConfig &config, uint32_t level)
{
	sim_time_ = config.sim_time;
	sim_granularity_ = config.sim_granularity;
	stop_time_buffer_ = config.stop_time_buffer;
	bz_length_tolerance_ = config.bz_length_tolerance;
	x_tolerance_ = config.x_tolerance;
	y_tolerance_ = config.y_tolerance;
	xy_tolerance_ = config.xy_tolerance;
	yaw_tolerance_ = config.yaw_tolerance;
	goal_reach_level_ = config.goal_reach_level;
	ROS_INFO("Updated bz_local_planner dynamic parameters");
}

void BZPlannerROS::publishBezierPlan(const std::vector<geometry_msgs::Pose2D>& points)
{
    int num = 50;
    double step = 1.0 / num;
    double t, t_squ, t_cub;
    double t_1, t_1_squ, t_1_cub;
    nav_msgs::Path gui_bz_path;
    gui_bz_path.header.frame_id = "map";
    gui_bz_path.header.stamp = ros::Time::now();
    gui_bz_path.poses.resize(num+1);
    for (int i = 0; i < num+1; i++)
    {
        t = step * i;
        t_squ = pow(t, 2);
        t_cub = pow(t, 3);
        t_1 = 1 - t;
        t_1_squ = pow(t_1, 2);
        t_1_cub = pow(t_1, 3);
        gui_bz_path.poses[i].header.frame_id = "map";
        gui_bz_path.poses[i].header.stamp = ros::Time::now();
        gui_bz_path.poses[i].pose.position.x = t_1_cub*points[0].x+3*t_1_squ*t*points[1].x+3*t_1*t_squ*points[2].x+t_cub*points[3].x;
        gui_bz_path.poses[i].pose.position.y = t_1_cub*points[0].y+3*t_1_squ*t*points[1].y+3*t_1*t_squ*points[2].y+t_cub*points[3].y;
        gui_bz_path.poses[i].pose.orientation.w = 1.0;
    }
    bz_plan_pub_.publish(gui_bz_path);
}

void BZPlannerROS::publishBezierCtrlPoints(const std::vector<geometry_msgs::Pose2D>& points)
{
    nav_msgs::Path gui_bz_points;
    gui_bz_points.header.frame_id = "map";
    gui_bz_points.header.stamp = ros::Time::now();
    gui_bz_points.poses.resize(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        gui_bz_points.poses[i].pose.position.x = points[i].x;
        gui_bz_points.poses[i].pose.position.y = points[i].y;
        gui_bz_points.poses[i].pose.orientation.w = 1.0;
    }
    bz_ctrl_points_pub_.publish(gui_bz_points);
}   

void BZPlannerROS::publishLocalPlan(base_local_planner::Trajectory& path)
{
    if (path.cost_ < 0)
    {
        return;
    }
    nav_msgs::Path gui_local_path;
    gui_local_path.poses.resize(path.getPointsSize());
    gui_local_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_local_path.header.stamp = ros::Time::now();
    for(int i = 0; i < path.getPointsSize(); ++i)
    {
        double p_x, p_y, p_th;
        path.getPoint(i, p_x, p_y, p_th);
        tf::Stamped<tf::Pose> p =
            tf::Stamped<tf::Pose>(tf::Pose(
              tf::createQuaternionFromYaw(p_th),
              tf::Point(p_x, p_y, 0.0)),
            ros::Time::now(),
            costmap_ros_->getGlobalFrameID());
        geometry_msgs::PoseStamped pose;
        tf::poseStampedTFToMsg(p, pose);
        gui_local_path.poses[i] = pose;
    }
    local_plan_pub_.publish(gui_local_path);
}

double BZPlannerROS::calcObstacleCost(base_local_planner::Trajectory& traj, 
                                      std::vector<geometry_msgs::Point> footprint_spec)
{
    obstacle_cost_->setFootprint(footprint_spec);
    return obstacle_cost_->scoreTrajectory(traj);
}

void BZPlannerROS::generateSimTraj(geometry_msgs::Twist& cmd_vel, 
                                   base_local_planner::Trajectory& traj)
{
    int num_steps = ceil((sim_time_ + stop_time_buffer_) / sim_granularity_);
    double dt = sim_time_ / num_steps;
    // current pose
    Eigen::Vector3f init_pose = Eigen::Vector3f::Zero();
    init_pose[0] = current_pose_.getOrigin().getX();
    init_pose[1] = current_pose_.getOrigin().getY();
    init_pose[2] = tf::getYaw(current_pose_.getRotation());
    // simulation vel
    Eigen::Vector3f sim_vel = Eigen::Vector3f::Zero();
    sim_vel[0] = cmd_vel.linear.x;
    sim_vel[1] = cmd_vel.linear.y;
    sim_vel[2] = cmd_vel.angular.z;
    // trajectory
    traj.time_delta_ = dt;
    traj.xv_ = cmd_vel.linear.x;
    traj.yv_ = cmd_vel.linear.y;
    traj.thetav_ = cmd_vel.angular.z;
    for (int i = 0; i < num_steps; ++i)
    {
        Eigen::Vector3f new_pos = calcNewPosition(init_pose, sim_vel, dt * i);
        traj.addPoint(new_pos[0], new_pos[1], new_pos[2]);  
    }
    // ROS_INFO("simulation trajectory points: %d", traj.getPointsSize());
}   

Eigen::Vector3f BZPlannerROS::calcNewPosition(const Eigen::Vector3f& pos, 
                                              const Eigen::Vector3f& vel, 
                                              double dt)
{
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
    new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
    new_pos[2] = pos[2] + vel[2] * dt;
    return new_pos;
}                                              

void BZPlannerROS::calcBezierVel(geometry_msgs::Twist& cmd_vel, std::vector<geometry_msgs::Pose2D>& points)
{
    geometry_msgs::PoseStamped pose, global_pose;
    tf::poseStampedTFToMsg(current_pose_, pose);
    try
    {
        tf_->waitForTransform("/map", costmap_ros_->getGlobalFrameID(), ros::Time(0), ros::Duration(5.0));
        tf_->transformPose("/map", pose, global_pose); 
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        exit(1);
    }
    geometry_msgs::Pose2D pr, ps, pt, pu, pv;
    ps.x = global_pose.pose.position.x;
    ps.y = global_pose.pose.position.y;
    ps.theta = tf::getYaw(global_pose.pose.orientation);
    double ps_tan = tan(ps.theta);
    pt.x = goal_.pose.position.x;
    pt.y = goal_.pose.position.y;
    pt.theta = tf::getYaw(goal_.pose.orientation);
    double pt_tan = tan(pt.theta);
    pr.x = (ps_tan * ps.x - pt_tan * pt.x - ps.y + pt.y) / (ps_tan - pt_tan);
    pr.y = ps_tan * (pr.x - ps.x) + ps.y;
    pr.theta = atan2(ps.y - pt.y, ps.x - pt.x);
    double pr_len = sqrt(pow(ps.x - pt.x, 2) + pow(ps.y - pt.y, 2));
    double px_len = pr_len * cos(pr.theta -pt.theta);
    // for isGoalReached use
    current_tolerance_ = px_len;
    double px_offset = (pr_len > (x_offset_pos_ + x_offset_neg_)) ? x_offset_pos_ : (pr_len - x_offset_neg_);
    double pr_dir = (fabs(pt.theta - pr.theta) > M_PI) ? (2*M_PI - fabs(pt.theta - pr.theta)) : (fabs(pt.theta - pr.theta));
    if (pr_dir < M_PI_2) 
    {
        pr_len = - pr_len;
        px_offset = - px_offset;
    }
    pt.x = pt.x - px_offset * cos(pt.theta);
    pt.y = pt.y - px_offset * sin(pt.theta);
    double ps_offset = (pr_len - px_offset) * source_u_;
    pu.x = ps.x + ps_offset * cos(ps.theta);
    pu.y = ps.y + ps_offset * sin(ps.theta);
    double pt_offset = (pr_len - px_offset) * target_v_;
    pv.x = pt.x - pt_offset * cos(pt.theta);
    pv.y = pt.y - pt_offset * sin(pt.theta);
    double dx_d = ps.x;
    double dx_c = 3.0*(pu.x-ps.x);
    double dx_b = 3.0*(pv.x-pu.x)-dx_c;
    double dx_a = pt.x-ps.x-dx_c-dx_b;
    double dy_d = ps.y;
    double dy_c = 3.0*(pu.y-ps.y);
    double dy_b = 3.0*(pv.y-pu.y)-dy_c;
    double dy_a = pt.y-ps.y-dy_c-dy_b;
    double t = 0.1;
    double tSquared = pow(t,2);
    double tCubed = tSquared*t;
    double dx = dx_a*tCubed + dx_b*tSquared + dx_c*t + dx_d;
    double dy = dy_a*tCubed + dy_b*tSquared + dy_c*t + dy_d; 
    double dxy = sqrt(pow(dx - ps.x, 2) + pow(dy - ps.y, 2));
    double dyaw_temp = ps.theta - (pr_len < 0 ? M_PI : 0) * (ps.theta > 0 ? 1 : -1);
    double dyaw2 = atan2(dy-ps.y, dx-ps.x);
    double dyaw = fmod(dyaw2 - dyaw_temp, M_PI * 2);
    if (dyaw > M_PI) 
        dyaw -= M_PI*2;
    if (dyaw < -M_PI) 
        dyaw += M_PI*2;
    cmd_vel.linear.x = (pr_len > 0 ? 1 : -1);
    cmd_vel.angular.z = fabs(cmd_vel.linear.x / dxy) * dyaw * 3;
    double cmd_vel_factor = (1 / sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.angular.z, 2)));
    cmd_vel_factor *= fmin(0.1 * px_len + 0.15, 0.3);
    cmd_vel.linear.x *= cmd_vel_factor;
    cmd_vel.angular.z *= cmd_vel_factor;
    points.push_back(ps);
    points.push_back(pu);
    points.push_back(pv);
    points.push_back(pt);
}

bool BZPlannerROS::goalReachBZLen()
{
	if (fabs(current_tolerance_) < bz_length_tolerance_)
    {
        return true;
    }
    return  false;
}

bool BZPlannerROS::goalReachXY()
{
	tf::Stamped<tf::Pose> goal_pose;
	planner_util_.getGoal(goal_pose);
	double goal_x = goal_pose.getOrigin().getX();
	double goal_y = goal_pose.getOrigin().getY();
	double goal_th = tf::getYaw(goal_pose.getRotation());
	if (fabs(current_pose_.getOrigin().getX() - goal_x) <= x_tolerance_
		&& fabs(current_pose_.getOrigin().getY() - goal_y) <= y_tolerance_)
	{
		return true;
	}
	return false;
}

bool BZPlannerROS::goalReachEucDis()
{
	tf::Stamped<tf::Pose> goal_pose;
	planner_util_.getGoal(goal_pose);
	double goal_x = goal_pose.getOrigin().getX();
	double goal_y = goal_pose.getOrigin().getY();
	if (base_local_planner::getGoalPositionDistance(current_pose_, goal_x, goal_y) <= xy_tolerance_)
	{
		return true;
	}
	return false;
}

bool BZPlannerROS::goalReachXYYaw()
{
	tf::Stamped<tf::Pose> goal_pose;
	planner_util_.getGoal(goal_pose);
	double goal_th = tf::getYaw(goal_pose.getRotation());
	if (goalReachXY()
		&& base_local_planner::getGoalOrientationAngleDifference(current_pose_, goal_th) <= yaw_tolerance_)
	{
		return true;
	}
	return false;
}

bool BZPlannerROS::goalReachEucDisYaw()
{
	tf::Stamped<tf::Pose> goal_pose;
	planner_util_.getGoal(goal_pose);
	double goal_th = tf::getYaw(goal_pose.getRotation());
	if (goalReachEucDis()
		&& base_local_planner::getGoalOrientationAngleDifference(current_pose_, goal_th) <= yaw_tolerance_)
	{
		return true;
	}
	return false;
}

}; // namespace
