/*********************************************************************
* Author: GrayLoo
********************************************************************/

#include <bz_local_planner/bz_planner_ros.h> 

PLUGINLIB_EXPORT_CLASS(bz_local_planner::BZPlannerROS, nav_core::BaseLocalPlanner)

namespace bz_local_planner{

BZPlannerROS::BZPlannerROS() : odom_helper_("odom"),
                               pose_helper_("target_mark"),
                               goal_helper_("relocation_goal"),
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
                               current_tolerance_(1024.0),
                               max_vel_x_(0.3),
                               min_vel_x_(0.15),
                               angular_ratio_(3.0),
                               vel_ratio_(0.1),
                               wheel_base_(1.55),
                               minimum_dist_(0.1),
                               convert_global_(false),
                               final_goal_lock_(false),
                               relocation_pose_topic_("triangle_pose"),
                               relocation_mode_(false),
                               local_frame_id_("base_local"),
                               relocation_frame_("map"),
                               motion_status_(NAVIGATION),
                               test_vel_(true)
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
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        planner_util_.initialize(tf_, costmap, costmap_ros_->getGlobalFrameID());
        obstacle_cost_ = std::make_shared<base_local_planner::ObstacleCostFunction>(costmap);
        obstacle_cost_->prepare();
        if (pnh.getParam("odom_topic", odom_topic_))
        {
            odom_helper_.setOdomTopic(odom_topic_);
        }
        if (pnh.getParam("relocation_pose_topic", relocation_pose_topic_))
        {
            pose_helper_.setPoseTopic(relocation_pose_topic_);
        }
        goal_helper_.setPoseTopic("relocation_goal");
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
        check_quad_num_ = 0;

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
        final_goal_lock_ = false;
        // reset current tolerance
        current_tolerance_ = 1024.0;
        tf::Stamped<tf::Pose> goal_pose;
        planner_util_.getGoal(goal_pose);
        geometry_msgs::PoseStamped pose;
        tf::poseStampedTFToMsg(goal_pose, pose);
        try
        {
            tf_->waitForTransform("/map", goal_pose.frame_id_, ros::Time(0), ros::Duration(5.0));
            tf_->transformPose("/map", ros::Time(0),  pose, global_frame_, final_goal_); 
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return false; //exit(1);
        }

        costmap_ros_->getRobotPose(current_pose_);
        geometry_msgs::PoseStamped start_pose, start_global_pose;
        tf::poseStampedTFToMsg(current_pose_, start_pose);
        try
        {
            tf_->waitForTransform("/map", global_frame_, ros::Time(0), ros::Duration(5.0));
            tf_->transformPose("/map", ros::Time(0), start_pose, global_frame_, start_global_pose); 
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return false; //exit(1);
        }
        bezierParams(select, start_global_pose, final_goal_);
        
        // pose helper provides  a goal in local frame, which is converted into map.
        if(pose_helper_.getPoseStatus() && motion_status_ == RELOCATION)
        {
            ROS_INFO("Controlling in relocation mode");
            if(relocation_frame_ == "base_footprint")
            {
                if(getLocalGoal())
                {
                    //ROS_ERROR("HA2");
                    goal_ = relocation_goal_;
                }
            }
            
            if(relocation_frame_ == "map")
            {
                if(getLocalPose())
                {
                    goal_ = relocation_pose_;
                }
            }
        }
        return true;
    }
    return false;
}

bool BZPlannerROS::globalPlanConversion(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, Bzstruct& a)
{
    geometry_msgs::PoseStamped temp_goal1,temp_goal2,temp_goal3;
    double dist,temp_yaw;
    temp_goal1 = plan.back();
    temp_goal2 = plan[plan.size() - 3];
    try
    {
        tf_->waitForTransform("/map", global_frame_, ros::Time(0), ros::Duration(5.0));
        tf_->transformPose("/map", ros::Time(0), temp_goal1, global_frame_, temp_goal1); 
        tf_->transformPose("/map", ros::Time(0), temp_goal2, global_frame_, temp_goal2);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;//exit(1);
    }
    dist = sqrt(pow(final_goal_.pose.position.x - temp_goal1.pose.position.x , 2)
                + pow(final_goal_.pose.position.y - temp_goal1.pose.position.y, 2));
    temp_yaw = atan2((temp_goal1.pose.position.y - temp_goal2.pose.position.y),(temp_goal1.pose.position.x - temp_goal2.pose.position.x));
    temp_yaw = (a.pr_len > 0 ? temp_yaw : temp_yaw + M_PI); 
    if(dist < minimum_dist_)
    {
        goal = final_goal_;
        final_goal_lock_ = true;
    }
    else
    {
        goal = plan.back();
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(temp_yaw);
    }
    return true;
}

bool BZPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    std::vector<geometry_msgs::Pose2D> bz_ctrl_points;
    geometry_msgs::PoseStamped pose, global_pose;
    tf::Stamped<tf::Pose> t1,t2;
    if (!costmap_ros_->getRobotPose(current_pose_))
    {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    // if(!planner_util_.getLocalPlan(current_pose_, transformed_plan)) //transform goal based on local plan
    // {
    //     ROS_ERROR("Could not transform local plan");
    //     return false;
    // }

    tf::poseStampedTFToMsg(current_pose_, pose);

    if(motion_status_ == RELOCATION){
        if(getLocalPose())
        {
            if(relocation_frame_ == "map")
            {
                try
                {
                    tf_->waitForTransform("/map", global_frame_, ros::Time(0), ros::Duration(5.0));
                    tf_->transformPose("/map", ros::Time(0), pose, global_frame_, global_pose); 
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("%s", ex.what());
                    return false; //exit(1);
                }
                goal_ = relocation_pose_;
                pose_helper_.pose_gained_ =false;       
            }

            if(relocation_frame_ == "base_footprint" && getLocalGoal())
            {
                if(getLocalPose())
                {   //ROS_ERROR("HA3");
                    goal_ = relocation_goal_;
                    global_pose = relocation_pose_;
                    pose_helper_.pose_gained_ = false;
                    // goal_helper_.pose_gained_ = false;
                }
                else{
                    ROS_ERROR("Can't get relocation pose while relocation goal received");
                    return false;  //return or not
                }
            }
        }
    }
    if(motion_status_ == NAVIGATION)
    {
        try
        {
            tf_->waitForTransform("/map", global_frame_, ros::Time(0), ros::Duration(5.0));
            tf_->transformPose("/map", ros::Time(0), pose, global_frame_, global_pose); 
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return false; //exit(1);
        }
        // if(!final_goal_lock_ && convert_global_)
        // {
        //     if(!globalPlanConversion(goal_, transformed_plan, select)){
        //         ROS_ERROR("Could not convert the global path into the goal used for calculating bezier curves");
        //         return false;
        //     }
        // }
        // else if(!convert_global_)
        // {
            goal_ = final_goal_;
        //}
    }
    std::lock_guard<std::mutex> lk(dyn_params_mutex_);
    tf::Pose gpose,ppose;
    tf::poseMsgToTF(global_pose.pose, ppose);
    tf::poseMsgToTF(goal_.pose, gpose);
    double pyaw_angle = tf::getYaw(ppose.getRotation());
    double gyaw_angle = tf::getYaw(gpose.getRotation());
    ROS_ERROR("BEZpose: x: %.2lf, y: %.2lf, yaw: %.2lf", global_pose.pose.position.x, global_pose.pose.position.y, pyaw_angle);
    ROS_ERROR("BEZgoal: x: %.2lf, y: %.2lf, yaw: %.2lf", goal_.pose.position.x, goal_.pose.position.y, gyaw_angle);
    if (!calcBezierVel(cmd_vel, bz_ctrl_points, global_pose, goal_)) {
        return false;
    }
    
    publishBezierPlan(bz_ctrl_points);
    publishBezierCtrlPoints(bz_ctrl_points);
    base_local_planner::Trajectory result_traj;
    generateSimTraj(cmd_vel, result_traj);
    double cost = calcObstacleCost(result_traj, costmap_ros_->getRobotFootprint());
    result_traj.cost_ = cost;
    publishLocalPlan(result_traj);
    ROS_ERROR("Vel x: %.2lf, y: %.2lf, yaw: %.2lf, cost: %.2lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cost);
    if(test_vel_)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
    }
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
    tf::Stamped<tf::Pose> t1,t2;
	std::lock_guard<std::mutex> lk(dyn_params_mutex_);

    if (!costmap_ros_->getRobotPose(current_pose_))
    {
    ROS_ERROR("Could not get robot pose, will stop");
    return true;
    }

    if(goalReachSum(goal_reach_level_))
    {
        return true;
    }

	return false;
}

void BZPlannerROS::reconfigureCB(BZPlannerConfig &config, uint32_t level)
{   int temp;
	sim_time_ = config.sim_time;
	sim_granularity_ = config.sim_granularity;
	stop_time_buffer_ = config.stop_time_buffer;
	bz_length_tolerance_ = config.bz_length_tolerance;
	x_tolerance_ = config.x_tolerance;
	y_tolerance_ = config.y_tolerance;
	xy_tolerance_ = config.xy_tolerance;
	yaw_tolerance_ = config.yaw_tolerance;
	goal_reach_level_ = config.goal_reach_level;
    max_vel_x_ = config.max_vel_x;
    min_vel_x_ = config.min_vel_x;
    vel_ratio_ = config.vel_ratio;
    angular_ratio_ = config.angular_ratio;
    wheel_base_ = config.wheel_base;
    x_offset_neg_ = config.x_offset_neg;
    x_offset_pos_ = config.x_offset_pos;
    minimum_dist_ = config.minimum_dist;
    convert_global_ = config.convert_global;
    source_u_ = config.current_ctrl;
    target_v_ = config.goal_ctrl;
    relocation_pose_topic_ = config.relocation_pose_topic;
    relocation_frame_ = config.relocation_frame;
    local_frame_id_ = config.local_frame_id;
    motion_status_ = static_cast<BZPlannerROS::ud_enum>(config.motion_status);
    pose_helper_.setPoseTopic(relocation_pose_topic_);
    test_vel_ = config.test_vel;
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

void BZPlannerROS::bezierParams(Bzstruct& bzstr, geometry_msgs::PoseStamped& current_p, geometry_msgs::PoseStamped& goal)
{
    bzstr.ps.x = current_p.pose.position.x;
    bzstr.ps.y = current_p.pose.position.y;
    bzstr.ps.theta = tf::getYaw(current_p.pose.orientation);
    bzstr.ps_tan = tan(bzstr.ps.theta);
    bzstr.pt.x = goal.pose.position.x;
    bzstr.pt.y = goal.pose.position.y;
    bzstr.pt.theta = tf::getYaw(goal.pose.orientation);
    bzstr.pt_tan = tan(bzstr.pt.theta);
    bzstr.pr.x = (bzstr.ps_tan * bzstr.ps.x - bzstr.pt_tan * bzstr.pt.x - bzstr.ps.y + bzstr.pt.y) / (bzstr.ps_tan - bzstr.pt_tan);
    bzstr.pr.y = bzstr.ps_tan * (bzstr.pr.x - bzstr.ps.x) + bzstr.ps.y;
    bzstr.pr.theta = atan2(bzstr.ps.y - bzstr.pt.y, bzstr.ps.x - bzstr.pt.x);
    bzstr.pr_len = sqrt(pow(bzstr.ps.x - bzstr.pt.x, 2) + pow(bzstr.ps.y - bzstr.pt.y, 2));
    bzstr.px_len = bzstr.pr_len * cos(bzstr.pr.theta -bzstr.pt.theta);
    // for isGoalReached use
    current_tolerance_ = bzstr.px_len;
    bzstr.px_offset = (bzstr.pr_len > (x_offset_pos_ + x_offset_neg_)) ? x_offset_pos_ : (bzstr.pr_len - x_offset_neg_);
    bzstr.pr_dir = (fabs(bzstr.pt.theta - bzstr.pr.theta) > M_PI) ? (2*M_PI - fabs(bzstr.pt.theta - bzstr.pr.theta)) : (fabs(bzstr.pt.theta - bzstr.pr.theta));
    if (bzstr.pr_dir < M_PI_2) 
    {
        bzstr.pr_len = - bzstr.pr_len;
        bzstr.px_offset = - bzstr.px_offset;
    }
    bzstr.pt.x = bzstr.pt.x - bzstr.px_offset * cos(bzstr.pt.theta);
    bzstr.pt.y = bzstr.pt.y - bzstr.px_offset * sin(bzstr.pt.theta);
    bzstr.ps_offset = (bzstr.pr_len - bzstr.px_offset) * source_u_;
    bzstr.pu.x = bzstr.ps.x + bzstr.ps_offset * cos(bzstr.ps.theta);
    bzstr.pu.y = bzstr.ps.y + bzstr.ps_offset * sin(bzstr.ps.theta);
    bzstr.pt_offset = (bzstr.pr_len - bzstr.px_offset) * target_v_;
    bzstr.pv.x = bzstr.pt.x - bzstr.pt_offset * cos(bzstr.pt.theta);
    bzstr.pv.y = bzstr.pt.y - bzstr.pt_offset * sin(bzstr.pt.theta);
}

bool BZPlannerROS::calcBezierVel(geometry_msgs::Twist& cmd_vel, std::vector<geometry_msgs::Pose2D>& points, geometry_msgs::PoseStamped& start ,geometry_msgs::PoseStamped& goal)
{
    Bzstruct b;
    bezierParams(b, start, goal);
    double dx_d = b.ps.x;
    double dx_c = 3.0*(b.pu.x-b.ps.x);
    double dx_b = 3.0*(b.pv.x-b.pu.x)-dx_c;
    double dx_a = b.pt.x-b.ps.x-dx_c-dx_b;
    double dy_d = b.ps.y;
    double dy_c = 3.0*(b.pu.y-b.ps.y);
    double dy_b = 3.0*(b.pv.y-b.pu.y)-dy_c;
    double dy_a = b.pt.y-b.ps.y-dy_c-dy_b;
    double t = 0.1;
    double tSquared = pow(t,2);
    double tCubed = tSquared*t;
    double dx = dx_a*tCubed + dx_b*tSquared + dx_c*t + dx_d;
    double dy = dy_a*tCubed + dy_b*tSquared + dy_c*t + dy_d; 
    double dxy = sqrt(pow(dx - b.ps.x, 2) + pow(dy - b.ps.y, 2));
    double dyaw_temp = b.ps.theta - (b.pr_len < 0 ? M_PI : 0) * (b.ps.theta > 0 ? 1 : -1);
    double dyaw2 = atan2(dy-b.ps.y, dx-b.ps.x);
    double dyaw = fmod(dyaw2 - dyaw_temp, M_PI * 2);
    if (dyaw > M_PI) 
        dyaw -= M_PI*2;
    if (dyaw < -M_PI) 
        dyaw += M_PI*2;
    cmd_vel.linear.x = (b.pr_len > 0 ? 1 : -1);
    cmd_vel.angular.z = fabs(cmd_vel.linear.x / dxy) * dyaw * angular_ratio_;
    double cmd_vel_factor = (1 / sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.angular.z * wheel_base_, 2)));
    cmd_vel_factor *= fmin(vel_ratio_ * fmax(fabs(b.px_len) - x_offset_pos_, 0) + min_vel_x_, max_vel_x_);
    cmd_vel.linear.x *= cmd_vel_factor;
    cmd_vel.angular.z *= cmd_vel_factor;
    points.push_back(b.ps);
    points.push_back(b.pu);
    points.push_back(b.pv);
    points.push_back(b.pt);

    return true;
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

bool BZPlannerROS::goalReachLocal()
{   
    tf::Stamped<tf::Pose> goal_pose, present_pose;
    if(getLocalGoal(goal_pose) && getLocalPose(present_pose)){
        double goal_x = goal_pose.getOrigin().getX();  
        double goal_y = goal_pose.getOrigin().getY();
        double goal_th = tf::getYaw(goal_pose.getRotation());
        if (base_local_planner::getGoalPositionDistance(present_pose,goal_x,goal_y) <= xy_tolerance_
            && base_local_planner::getGoalOrientationAngleDifference(present_pose, goal_th) <= yaw_tolerance_)
        {
            return true;
        }
    }
	return false;
}

bool BZPlannerROS::goalReachLocalinMap()
{   
    tf::Stamped<tf::Pose> goal_pose, present_pose;
        tf::poseStampedMsgToTF(goal_,goal_pose);
        double goal_x = goal_pose.getOrigin().getX();  
        double goal_y = goal_pose.getOrigin().getY();
        double goal_th = tf::getYaw(goal_pose.getRotation());
        if (base_local_planner::getGoalPositionDistance(present_pose,goal_x,goal_y) <= xy_tolerance_
            && base_local_planner::getGoalOrientationAngleDifference(present_pose, goal_th) <= yaw_tolerance_)
        {
            return true;
        }
    
	return false;
}

bool BZPlannerROS::goalReachSum(int reach_level)
{
    switch(motion_status_)
    {
        case RELOCATION :{
            if(relocation_frame_ == "map")
            {
                if(goalReachLocalinMap()){
                    ROS_INFO("Goal reached in relocation mode, with goal and current pose converted into frame map");
                    return true;
                }
            }
            if(relocation_frame_ == "base_footprint")
            {   //ROS_ERROR("ha1");
                if(goalReachLocal()){
                    ROS_INFO("Goal reached in relocation mode");
                    return true;
                }
            }
            break;
        }
        case NAVIGATION :{
            auto goal_reach_func = is_goal_reach_funcs_[reach_level];
            if(goal_reach_func())
            {
                ROS_INFO("Goal reached in navigation mode, level: %d",reach_level);
                return true;
            }
            break;
        }
        default :{
            ROS_ERROR("Motion status not available, exit now");
            return false;
            break;
        }
    }
    return false;
}

bool BZPlannerROS::getLocalGoal(tf::Stamped<tf::Pose>& tf_goal)
{
    if(goal_helper_.getPoseStatus())
    {
        goal_helper_.getPose(relocation_goal_);
        if(checkQuaternionOnce(relocation_goal_))
        {
            tf::poseStampedMsgToTF(relocation_goal_,tf_goal);
            relocation_mode_ = true;
            return true;
            ROS_INFO("Received relocation goal, executing relocation mode now");
        }
        
        return false;
    }
    else{
        return false;
    }
}

bool BZPlannerROS::getLocalGoal()
{   tf::Stamped<tf::Pose> t1;
    if(getLocalGoal(t1))
    {
        return true;
        ROS_INFO("Received relocation goal, executing relocation mode now");
    }
    else{
        return false;
    }
}

bool BZPlannerROS::getLocalPose(tf::Stamped<tf::Pose>& tf_pose)
{   
    if(getLocalPose())
    {
        tf::poseStampedMsgToTF(relocation_pose_, tf_pose);
        return true;
    }
    return false;
}

bool BZPlannerROS::getLocalPose()
{   
    geometry_msgs::PoseStamped temp1,temp2, pose_in_, pose_out_;
    Eigen::Isometry3d TransformL2R, TransformR2L;
    std::vector<geometry_msgs::PoseStamped> detect_invalid_pose_(5);
    if(!(pose_helper_.getPoseStatus() || getLocalPosebyTf(local_frame_id_, pose_out_)))
    {
    	ROS_ERROR("Didn't receive any valid local poses");
        return false;
    }

    if(pose_helper_.getPoseStatus())
    {   

        pose_helper_.getPose(pose_in_);
        //invalid pose detection
        if(checkQuaternion(pose_in_) >= 4)
        {   
            ROS_ERROR("Received too many invalid poses, failured to update local pose ");
            return false;
        }
        else
        {
            // ROS_ERROR("POSE RECEIVED");
            motion_status_ = RELOCATION;
            if(relocation_frame_ == "map")
            {
                try
                {
                    tf_->waitForTransform(relocation_frame_, pose_in_.header.frame_id, ros::Time(0), ros::Duration(5.0));
                    tf_->transformPose(relocation_frame_, ros::Time(0), pose_in_, pose_in_.header.frame_id, pose_out_); 
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("%s", ex.what());
                    return false; //exit(1);
                }
                relocation_pose_ = pose_out_;
                return true;
            }

            if(relocation_frame_ == "base_footprint")   
            {   
                if(!checkQuaternionOnce(pose_in_))
                {
                    pose_in_ = last_valid_pose_;
                    //ignore the initial errors.
                }
                if(pose_in_.header.frame_id != "base_footprint")
                {
                    try
                    {
                        tf_->waitForTransform(relocation_frame_, pose_in_.header.frame_id, ros::Time(0), ros::Duration(5.0));
                        tf_->transformPose(relocation_frame_, ros::Time(0), pose_in_, pose_in_.header.frame_id, pose_out_); 
                    }
                    catch(tf::TransformException& ex)
                    {
                        ROS_ERROR("%s", ex.what());
                        return false; //exit(1);
                    }
                }
                else{
                    pose_out_ = pose_in_;
                }
                last_valid_pose_ = pose_out_;
                tf::Pose pose;
                tf::Quaternion trans_quad, inv_quad, origin_quad;
                tf::Transform footprint2local_trans, temp_inv,origin_trans;
                origin_quad.setRPY(0.0,0.0,0.0);
                origin_trans.setOrigin(tf::Vector3(0.0,0.0,0.0));
                origin_trans.setRotation(origin_quad);
                footprint2local_trans.setOrigin(tf::Vector3(pose_out_.pose.position.x, pose_out_.pose.position.y, 0.0));
                tf::poseMsgToTF(pose_out_.pose, pose);
                double yaw = tf::getYaw(pose.getRotation());
                trans_quad = pose.getRotation();
                footprint2local_trans.setRotation(trans_quad);
                temp_inv = footprint2local_trans.inverseTimes(origin_trans);
                temp2.pose.position.x = temp_inv.getOrigin().getX();
                temp2.pose.position.y = temp_inv.getOrigin().getY();
                temp2.pose.position.z = temp_inv.getOrigin().getZ();
                inv_quad = temp_inv.getRotation();
                tf::quaternionTFToMsg(inv_quad, temp2.pose.orientation);
                ROS_ERROR("pose_in: x: %.2lf, y: %.2lf, yaw: %.2lf", pose_out_.pose.position.x, pose_out_.pose.position.y, yaw);
                // tf::poseMsgToEigen(pose_in_.pose,TransformR2L);
                // TransformL2R = TransformR2L.inverse();
                // tf::poseEigenToMsg(TransformL2R,temp2.pose);
                temp2.header = pose_out_.header;

                relocation_pose_ = temp2;
            }
        }
    }
    if(getLocalPosebyTf(local_frame_id_, pose_out_))
    {
        relocation_pose_ = pose_out_;
    }
    return true;
}

bool BZPlannerROS::getLocalStatus()
{
    tf::Stamped<tf::Pose> t1,t2;
    if(goal_helper_.getPoseStatus() && pose_helper_.getPoseStatus())
    {
        return true;
    }
    return false;
}

unsigned int BZPlannerROS::checkQuaternion(const geometry_msgs::PoseStamped& pose)
{   
    try
    {
        tf::assertQuaternionValid(pose.pose.orientation);
    }
    catch(tf::TransformException& ex)
    {   
        ++check_quad_num_;
        ROS_ERROR("%s, malformed times:%u", ex.what(),check_quad_num_);
        return check_quad_num_;
    }
    check_quad_num_ = 0;
    return check_quad_num_;
}

bool BZPlannerROS::checkQuaternionOnce(const geometry_msgs::PoseStamped& pose)
{
    try
    {
        tf::assertQuaternionValid(pose.pose.orientation);
    }
    catch(tf::TransformException& ex)
    {   
        ROS_ERROR("%s, malformed times:%u", ex.what(),check_quad_num_);
        return false;
    }
    return true;
}

bool BZPlannerROS::getLocalPosebyTf(const std::string& local_frame_id, geometry_msgs::PoseStamped& local_pose_by_tf)
{   
    if(!(relocation_frame_ == "map"||relocation_frame_ == "base_footprint"))
    {
        ROS_ERROR("Relocation frame can't be parsed!");
        return false;
    }
    tf::StampedTransform local_tf;
    geometry_msgs::PoseStamped origin_pose;
    origin_pose.header.frame_id = local_frame_id;
    origin_pose.header.stamp = ros::Time::now();
    origin_pose.pose.position.x = 0.0;
    origin_pose.pose.position.y = 0.0;
    origin_pose.pose.position.z = 0.0;
    origin_pose.pose.orientation.x = 0.0;
    origin_pose.pose.orientation.y = 0.0;
    origin_pose.pose.orientation.z = 0.0;
    origin_pose.pose.orientation.w = 1.0;
    if(relocation_frame_ == "map")
    {
        try
        {
            tf_->waitForTransform(relocation_frame_, local_frame_id, ros::Time(0), ros::Duration(1.0));
            tf_->transformPose(relocation_frame_, ros::Time(0), origin_pose, local_frame_id, local_pose_by_tf); 
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return false; //exit(1);
        }
    }
    if(relocation_frame_ == "base_footprint")
    {   
        origin_pose.header.frame_id = "base_footprint";
        try
        {
            tf_->waitForTransform(local_frame_id, relocation_frame_, ros::Time(0), ros::Duration(1.0));
            tf_->transformPose(local_frame_id, ros::Time(0), origin_pose, relocation_frame_, local_pose_by_tf); 
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return false; //exit(1);
        }
    }

    return true;
}

}; // namespace
