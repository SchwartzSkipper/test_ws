/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef YOCS_WAYPOINT_NAVI_HPP_
#define YOCS_WAYPOINT_NAVI_HPP_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sound_play/SoundRequest.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/WaypointList.h>
#include <yocs_msgs/NavgationCtrlService.h>
#include <scheduling_msgs/ReportNavigationControlStatus.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace yocs
{

/*
 * TODO
 *  * think about how to best visualise the waypoint(s)/trajectory(ies) which are being executed
 *  * add RViz interface to yocs_waypoint_provider
 */

class WaypointsGoalNode
{
public:
  WaypointsGoalNode();
  ~WaypointsGoalNode();

  bool init();

  void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps);

  void trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs);

  void navCtrlCB(const yocs_msgs::NavigationControl::ConstPtr& nav_ctrl);

  bool nav_ctrl_service_callback(yocs_msgs::NavgationCtrlService::Request &req,yocs_msgs::NavgationCtrlService::Response &res);

  bool reportNavigationControlStatus(scheduling_msgs::ReportNavigationControlStatus srv,int max_retry_time=5);

  void spin();
private:
  const geometry_msgs::PoseStamped NOWHERE;

  enum { NONE = 0,
         GOAL,
         LOOP
       } mode_;

  enum { IDLE = 0,
         START,
         ACTIVE,
         COMPLETED
       } state_;

  double      frequency_;
  double      close_enough_;
  double      goal_timeout_;
  bool report_nav_ctrl_status_using_service_;

	// bezier params
	double x_offset_pos_;
	double x_offset_neg_;
	double source_u_;
	double target_v_;
	double angular_ratio_;
	double vel_ratio_;
	double wheel_base_;
  double min_bez_vel_;
  double max_bez_vel_;

  std::string failure_mode_;
  std::string robot_frame_;
  std::string world_frame_;

  std::vector<yocs_msgs::Waypoint>           waypoints_;
  std::vector<yocs_msgs::Waypoint>::iterator waypoints_it_;

  geometry_msgs::PoseStamped tail_;
  geometry_msgs::PoseStamped mark_;
  geometry_msgs::PoseStamped goal_;

  yocs_msgs::WaypointList wp_list_;
  yocs_msgs::TrajectoryList traj_list_;

  ros::NodeHandle nh_;
  ros::NodeHandle ph_;

  tf::TransformListener tf_listener_;
  ros::Subscriber    waypoints_sub_;
  ros::Subscriber    trajectories_sub_;
  ros::Subscriber    waypoint_user_sub_;
  ros::Publisher     waypoint_user_pub_;
  ros::Subscriber nav_ctrl_sub_;
  ros::Publisher  nav_ctrl_status_pub_;
  ros::Publisher nav_ctrl_pub_;
  ros::Publisher cmd_vel_set_sub_;
  ros::Publisher cmd_string_pub_;
  ros::Publisher scan_marker_pub_;
  ros::Subscriber scan_marker_sub_;
  ros::Publisher marker_pub_;
  ros::Subscriber marker_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_vel_manual_pub_;
  ros::Publisher robotsound_pub_;
  ros::Publisher initialpose_pub_;
  ros::Subscriber robot_pose_sub_;
  bool idle_status_update_sent_;
  bool is_user_sub_;
  bool is_user_sub_ok_;
  std::string user_data_;
  std::string init_pub_;

  ros::Publisher plc_io_set_pub_;
  ros::Publisher plc_io_on_pub_;
  ros::Publisher plc_io_off_pub_;
  ros::Publisher gui_p_pub_;
  ros::Publisher gui_q_pub_;

  ros::Subscriber local_marker_sub_;
  ros::Publisher local_marker_pub_;

  ros::ServiceServer nav_ctrl_service_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;

  bool cancelAllGoals(double timeout = 2.0);

  void resetWaypoints();

  bool equals(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b);

  bool equals(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

  //void publishStatusUpdate(const uint8_t& status);

  void publishStatusUpdate(const uint8_t& status, const std::string& waypoint_name);

  void waypoint_user_sub(const std_msgs::String::ConstPtr& msg);
  void scan_marker_sub(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void marker_sub(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void line_sub(const geometry_msgs::Pose::ConstPtr& msg);
  void tail_sub(const geometry_msgs::Pose::ConstPtr& msg);
};

} // namespace yocs

#endif /* YOCS_WAYPOINT_NAVI_HPP_ */
