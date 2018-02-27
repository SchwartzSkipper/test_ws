/*********************************************************************
* Author: GrayLoo
*********************************************************************/
#ifndef BZ_PLANNER_ROS_H_
#define BZ_PLANNER_ROS_H_

#include <iostream>
#include <cmath>
#include <memory>
#include <mutex>
#include <functional>

#include <Eigen/Core>

#include <ros/console.h>
#include <nav_core/base_local_planner.h>
#include <bz_local_planner/BZPlannerConfig.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/goal_functions.h>

#include <pluginlib/class_list_macros.h>
#include <bz_local_planner/bz_planner_utility.h>

namespace bz_local_planner {

class BZPlannerROS : public nav_core::BaseLocalPlanner {
public:
    BZPlannerROS();
    ~BZPlannerROS();
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
private:
    void reconfigureCB(BZPlannerConfig &config, uint32_t level);
    void publishBezierPlan(const std::vector<geometry_msgs::Pose2D>& points);
	void publishBezierCtrlPoints(const std::vector<geometry_msgs::Pose2D>& points);
    void publishLocalPlan(base_local_planner::Trajectory& path);
    double calcObstacleCost(base_local_planner::Trajectory& traj, std::vector<geometry_msgs::Point> footprint_spec);
    void generateSimTraj(geometry_msgs::Twist& cmd_vel, base_local_planner::Trajectory& traj);
    Eigen::Vector3f calcNewPosition(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt);
    bool calcBezierVel(geometry_msgs::Twist& cmd_vel, std::vector<geometry_msgs::Pose2D>& points, geometry_msgs::PoseStamped& goal);
	bool globalPlanConversion(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, Bzstruct& a);
	void bezierParams(Bzstruct& bzstr, geometry_msgs::PoseStamped& current_p, geometry_msgs::PoseStamped& goal);
    // goal reach functions
    bool goalReachBZLen();
    bool goalReachXY();
    bool goalReachEucDis();
    bool goalReachXYYaw();
    bool goalReachEucDisYaw();
private:
	std::string odom_topic_;
	std::mutex dyn_params_mutex_;
	std::shared_ptr<base_local_planner::ObstacleCostFunction> obstacle_cost_;
	std::shared_ptr<dynamic_reconfigure::Server<bz_local_planner::BZPlannerConfig>> dyn_server_;
	std::vector<std::function<bool(void)>> is_goal_reach_funcs_;
	ros::Publisher bz_plan_pub_;
	ros::Publisher bz_ctrl_points_pub_;
	ros::Publisher local_plan_pub_;
	base_local_planner::LocalPlannerUtil planner_util_;
	base_local_planner::OdometryHelperRos odom_helper_;
	tf::Stamped<tf::Pose> current_pose_;
	tf::TransformListener* tf_;
	costmap_2d::Costmap2DROS* costmap_ros_;
	geometry_msgs::PoseStamped goal_;
	std::string global_frame_;
	geometry_msgs::PoseStamped final_goal_;
	bool initialized_;
	// dynamic params
	double sim_time_;
	double sim_granularity_;
	double stop_time_buffer_;
	double bz_length_tolerance_;
	double x_tolerance_;
	double y_tolerance_;
	double xy_tolerance_; // Euclidean distance
	double yaw_tolerance_;
	double max_vel_x_;
	double min_vel_x_;
	double angular_ratio_;
	double vel_ratio_;
	double minimum_dist_;
	double wheel_base_;
	/*
	goal reach level:
	0: by bezier curve length
	1: by x, y tolerance
	2: by Euclidean distance
	3: by x, y, yaw tolerance
	4: by Euclidean distance, yaw tolerance
	 */
	int goal_reach_level_;
	
	// bezier params
	double x_offset_pos_;
	double x_offset_neg_;
	double source_u_;
	double target_v_;

	double current_tolerance_;

	//global plan conversion
	bool final_goal_lock_;

	//The selection of forward or backward movement after the initialization of global plan 
	Bzstruct select;
};

}; // namespace
#endif
