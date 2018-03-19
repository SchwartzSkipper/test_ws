/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_waypoints_navi/waypoints_navi.hpp"
#include "nav_msgs/Path.h"
#include "json/json.h"

/*
 * TODO
 *  * think about how to best visualise the waypoint(s)/trajectory(ies) which are being executed
 *  * add RViz interface to yocs_waypoint_provider
 */

namespace yocs
{

WaypointsGoalNode::WaypointsGoalNode() : ph_("~"),
                                         mode_(NONE),
                                         state_(IDLE),
                                         frequency_(5), // 5 hz
                                         close_enough_(0.1), // 10 cm
                                         goal_timeout_(60.0), // 60 sec
                                         idle_status_update_sent_(false),
                                         move_base_ac_("move_base", true),
                                         is_user_sub_(false),
                                         is_user_sub_ok_(false),
                                         user_data_(""),
                                         failure_mode_("NONE"),
                                         x_offset_pos_(1.0),
                                         x_offset_neg_(1.0),
                                         source_u_(0.3),
                                         target_v_(0.5),
                                         angular_ratio_(5.0),
                                         vel_ratio_(0.3),
                                         wheel_base_(1.55),
                                         min_bez_vel_(0.2),
                                         max_bez_vel_(0.5),
                                         x_tolerance_(0.05),
                                         y_tolerance_(0.05),
                                         yaw_tolerance_(0.05),
                                         manual_comp_(0.05)
{}

WaypointsGoalNode::~WaypointsGoalNode()
{}

bool WaypointsGoalNode::init()
{
  ph_.param("frequency",      frequency_,     1.0);
  ph_.param("close_enough",   close_enough_,  0.3);  // close enough to next waypoint
  ph_.param("goal_timeout",   goal_timeout_, 30.0);  // maximum time to reach a waypoint
  ph_.param<bool>("report_nav_ctrl_status_using_service",report_nav_ctrl_status_using_service_,true);

  ph_.param("failure_mode",   failure_mode_,   std::string("NONE"));
  ph_.param("robot_frame",    robot_frame_,    std::string("/base_footprint"));
  ph_.param("world_frame",    world_frame_,    std::string("/map"));
  ph_.param("init_pub",       init_pub_,       std::string("init"));

  // reset goal way points
  waypoints_.clear();
  waypoints_it_ = waypoints_.end();

  waypoints_sub_  = nh_.subscribe("waypoints",  1, &WaypointsGoalNode::waypointsCB, this);
  trajectories_sub_  = nh_.subscribe("trajectories",  1, &WaypointsGoalNode::trajectoriesCB, this);
  nav_ctrl_sub_  = nh_.subscribe("nav_ctrl", 1, &WaypointsGoalNode::navCtrlCB, this);
  nav_ctrl_status_pub_  = nh_.advertise<yocs_msgs::NavigationControlStatus>("nav_ctrl_status", 1, true);
  waypoint_user_pub_  = nh_.advertise<std_msgs::String>("waypoint_user_pub", 1, true);
  waypoint_user_sub_ = nh_.subscribe("waypoint_user_sub", 1000, &WaypointsGoalNode::waypoint_user_sub, this);
  nav_ctrl_pub_  = nh_.advertise<yocs_msgs::NavigationControl>("nav_ctrl", 1, true);
  cmd_vel_set_sub_ = nh_.advertise<std_msgs::Float64>("cmd_vel_set", 1, true);
  cmd_string_pub_ = nh_.advertise<std_msgs::String>("cmd_string", 1, true);
  scan_marker_pub_ = nh_.advertise<std_msgs::Float64>("/scan_marker/trig", 1, true);
  marker_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("source_mark", 1, true);
  marker_sub_ = nh_.subscribe("target_mark", 1, &WaypointsGoalNode::marker_sub, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  cmd_vel_manual_pub_ = nh_.advertise<std_msgs::Bool>("cmd_vel_manual", 1, true);
  robotsound_pub_ = nh_.advertise<sound_play::SoundRequest>("robotsound", 1, true);
  initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
  
  plc_io_set_pub_ = nh_.advertise<std_msgs::Int16>("plc_io_set", 1, true);
  plc_io_on_pub_ = nh_.advertise<std_msgs::Int16>("plc_io_on", 1, true);
  plc_io_off_pub_ = nh_.advertise<std_msgs::Int16>("plc_io_off", 1, true);
  gui_p_pub_ = ph_.advertise<nav_msgs::Path>("points", 1, true);
  gui_q_pub_ = ph_.advertise<nav_msgs::Path>("qoints", 1, true);

  nav_ctrl_service_ = nh_.advertiseService("nav_ctrl_service", &WaypointsGoalNode::nav_ctrl_service_callback, this);

  while ((move_base_ac_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
  {
    ROS_WARN_THROTTLE(1, "Waiting for move_base action server to come up...");
  }

  return true;
}

void WaypointsGoalNode::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps)
{
  wp_list_ = *wps;
  ROS_INFO_STREAM("Received " << wp_list_.waypoints.size() << " way points.");

  if ("" != init_pub_) {
    for (unsigned int wp = 0; wp < wp_list_.waypoints.size(); ++wp)
    {
      if (init_pub_ == wp_list_.waypoints[wp].name)
      {
        ros::Publisher nav_ctrl_pub = nh_.advertise<yocs_msgs::NavigationControl>("nav_ctrl", 1, true);
        yocs_msgs::NavigationControl msg;
        msg.control = 1;
        msg.goal_name = init_pub_;
        nav_ctrl_pub.publish(msg);
        break;
      }
      ROS_WARN("No init name mateches");
    }
    init_pub_ = "";
  }
}

void WaypointsGoalNode::trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs)
{
  traj_list_ = *trajs;
  ROS_INFO_STREAM("Received " << traj_list_.trajectories.size() << " trajectories.");
}

bool WaypointsGoalNode::nav_ctrl_service_callback(yocs_msgs::NavgationCtrlService::Request &req,yocs_msgs::NavgationCtrlService::Response &res){

  yocs_msgs::NavigationControl nav_ctrl;
  nav_ctrl.control = req.msg.control;
  nav_ctrl.goal_name = req.msg.goal_name;
  nav_ctrl_pub_.publish(nav_ctrl);
  res.success = true;

  return true;
}

bool WaypointsGoalNode::reportNavigationControlStatus(scheduling_msgs::ReportNavigationControlStatus srv, int max_retry_time){
    bool success=ros::service::call("nav_ctrl_status_service",srv);
    int retry_time=0;
    while(!(success && srv.response.received) && retry_time++<max_retry_time){
        success=ros::service::call("nav_ctrl_status_service",srv);
        ros::Duration(0.2).sleep();
    }
    return (success && srv.response.received);
}

void WaypointsGoalNode::navCtrlCB(const yocs_msgs::NavigationControl::ConstPtr& nav_ctrl )
{
  if (nav_ctrl->control == yocs_msgs::NavigationControl::STOP)
  {
    if ((state_ == START) || (state_ == ACTIVE))
    {
      ROS_INFO_STREAM("Stopping current execution ...");
      cancelAllGoals();
      resetWaypoints();
      ROS_INFO_STREAM("Current execution stopped.");
      idle_status_update_sent_ = false;
      state_ = IDLE;
      is_user_sub_ = true;
      is_user_sub_ok_ = true;
      publishStatusUpdate(yocs_msgs::NavigationControlStatus::CANCELLED, nav_ctrl->goal_name);//
    }
    else
    {
      ROS_WARN_STREAM("Cannot stop way point/trajectory execution, because nothing is being executed.");
    }
  }
  else if ((nav_ctrl->control == yocs_msgs::NavigationControl::START))
  {
      if ((state_ == START) || (state_ == ACTIVE))
      {
        ROS_INFO_STREAM("Stopping current execution ...");
        cancelAllGoals();
        resetWaypoints();
        ROS_INFO_STREAM("Current execution stopped.");
        idle_status_update_sent_ = false;
        state_ = IDLE;
        is_user_sub_ = true;
        is_user_sub_ok_ = true;
//        publishStatusUpdate(yocs_msgs::NavigationControlStatus::CANCELLED, nav_ctrl->goal_name);//
      }
    if ((state_ == IDLE) || (state_ == COMPLETED))
    {
      resetWaypoints();
      // If provided goal is among the way points or trajectories, add the way point(s) to the goal way point list
      bool goal_found = false;
      for (unsigned int wp = 0; wp < wp_list_.waypoints.size(); ++wp)
      {
        if (nav_ctrl->goal_name == wp_list_.waypoints[wp].name)
        {
          waypoints_.push_back(wp_list_.waypoints[wp]);
          waypoints_it_ = waypoints_.begin();
          goal_found = true;
          ROS_INFO_STREAM("Prepared to navigate to way point '" << nav_ctrl->goal_name << "'.");
          continue;
        }
      }
      if (!goal_found)
      {
        for (unsigned int traj = 0; traj < traj_list_.trajectories.size(); ++traj)
        {
          if (nav_ctrl->goal_name == traj_list_.trajectories[traj].name)
          {
            for (unsigned int wp = 0; wp < traj_list_.trajectories[traj].waypoints.size(); ++wp)
            {
              waypoints_.push_back(traj_list_.trajectories[traj].waypoints[wp]);
            }
            waypoints_it_ = waypoints_.begin();
            goal_found = true;
            ROS_INFO_STREAM("Prepared to navigate along the trajectory '" << nav_ctrl->goal_name << "'.");
            ROS_INFO_STREAM("# of way points = " << waypoints_.size());
          }
        }
      }
      if (goal_found)
      {
        state_ = START;
        mode_  = GOAL;
      }
      else
      {
        ROS_WARN_STREAM("Could not find provided way point or trajectory.");
      }
    }
    else
    {
      ROS_WARN_STREAM("Cannot start way point/trajectory execution, because navigator is currently active. "
                      << "Please stop current activity first.");
    }
  }
  else
  {
    // TODO: handle PAUSE
    ROS_WARN_STREAM("'Pause' not yet implemented.");
  }
}

bool WaypointsGoalNode::cancelAllGoals(double timeout)
{
  actionlib::SimpleClientGoalState goal_state = move_base_ac_.getState();
  if ((goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
      (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
      (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
      (goal_state != actionlib::SimpleClientGoalState::PREEMPTED))
  {
    // We cannot cancel a REJECTED, ABORTED, SUCCEEDED or LOST goal
    ROS_WARN("Cannot cancel move base goal, as it has %s state!", goal_state.toString().c_str());
    publishStatusUpdate(yocs_msgs::NavigationControlStatus::ERROR, "");
    return true;
  }

  ROS_INFO("Canceling move base goal with %s state...", goal_state.toString().c_str());
  move_base_ac_.cancelAllGoals();
  if (move_base_ac_.waitForResult(ros::Duration(timeout)) == false)
  {
    ROS_WARN("Cancel move base goal didn't finish after %.2f seconds: %s",
             timeout, goal_state.toString().c_str());
    publishStatusUpdate(yocs_msgs::NavigationControlStatus::ERROR, "");
    return false;
  }

  ROS_INFO("Cancel move base goal succeed. New state is %s", goal_state.toString().c_str());
  publishStatusUpdate(yocs_msgs::NavigationControlStatus::CANCELLED, "");
  return true;
}

void WaypointsGoalNode::resetWaypoints()
{
  ROS_DEBUG("Full reset: clear markers, delete waypoints and goal and set state to IDLE");
  waypoints_.clear();
  waypoints_it_ = waypoints_.end();
  goal_  = NOWHERE;
  mode_  = NONE;
}

void WaypointsGoalNode::spin()
{
  char* agv_name = getenv("AGV_NAME");
  std::string AGV_NAME;
  if(agv_name != NULL) {
    AGV_NAME = agv_name;
    AGV_NAME = std::string(AGV_NAME).substr(AGV_NAME.rfind("/") + 1, AGV_NAME.length() - 1);
  }
  
  move_base_msgs::MoveBaseGoal mb_goal;

  ros::Rate rate(frequency_);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();

    if (state_ == START)
    {
      if (mode_ == LOOP)
      {
        if (waypoints_it_ == waypoints_.end())
        {
            waypoints_it_ = waypoints_.begin();
        }
      }

      if (waypoints_it_ < waypoints_.end())
      {
        Json::Reader json_reader;
        Json::Value json_value;
        if (!json_reader.parse(waypoints_it_->header.frame_id, json_value, false))
        {
          return; //TODO: report error
        }

        publishStatusUpdate(yocs_msgs::NavigationControlStatus::RUNNING, waypoints_it_->name);

        if ("timer" == json_value["type"].asString())
        {
          sleep(atof(json_value["goal_timeout"].asString().c_str()));
          waypoints_it_++;
        }
        else if ("publisher" == json_value["type"].asString())
        {
          std_msgs::String msg;
          if (json_value.isMember("data")) {
            msg.data = json_value["data"].asString();
          } else {
            msg.data = waypoints_it_->name;
          }
          waypoint_user_pub_.publish(msg);
          waypoints_it_++;
        }
        else if ("subscriber" == json_value["type"].asString())
        {
          is_user_sub_ = false;
          while (ros::ok() && !is_user_sub_)
          {
            ros::spinOnce();
          }
          if (state_ == IDLE)
          {
            continue;
          }
          waypoints_it_++;
        }
        else if ("pubsuber" == json_value["type"].asString())
        {
          std_msgs::String msg;
          msg.data = waypoints_it_->name;
          user_data_ = "";
          is_user_sub_ok_ = false;
          while (ros::ok() && !is_user_sub_ok_)
          {
            usleep(10000);
            waypoint_user_pub_.publish(msg);
            is_user_sub_ = false;
            ros::Time time_in = ros::Time::now();
            ros::Duration time_out = ros::Duration(0.2);
            while (ros::ok() && !is_user_sub_)
            {
              ros::spinOnce(); // waiting with interrupt
              if (ros::Time::now() - time_in > time_out)
              {
                break;
              }
              usleep(10);
            }
            if (user_data_ == json_value["failure_mode"].asString())
            {
              is_user_sub_ok_ = true;
            }
          }
          if (state_ == IDLE)
          {
            continue;
          }
          waypoints_it_++;
        }
        else if ("looper" == json_value["type"].asString())
          {
              if ("" == json_value["failure_mode"].asString() || "NONE" == json_value["failure_mode"].asString() || "LOOP" == json_value["failure_mode"].asString())
              {
                  waypoints_it_ = waypoints_.begin();
              }
              else
              {
                  yocs_msgs::NavigationControl nav_ctrl;
                  nav_ctrl.control = yocs_msgs::NavigationControl::START;
                  nav_ctrl.goal_name = json_value["failure_mode"].asString();
                  nav_ctrl_pub_.publish(nav_ctrl);
              }
          }
          else if ("cmd_vel_set_sub" == json_value["type"].asString())
          {
              std_msgs::Float64 msg;
              msg.data = atof(json_value["close_enough"].asString().c_str());
              cmd_vel_set_sub_.publish(msg);
              waypoints_it_++;
          }
          else if ("cmd_vel" == json_value["type"].asString())
          {
              geometry_msgs::Twist msg;
              if (atof(json_value["close_enough"].asString().c_str())) {
                msg.linear.x = atof(json_value["close_enough"].asString().c_str());
              } else {
                msg.linear.x = waypoints_it_->pose.position.x;
                msg.linear.y = waypoints_it_->pose.position.y;
                msg.linear.z = waypoints_it_->pose.position.z;
                msg.angular.x = waypoints_it_->pose.orientation.x;
                msg.angular.y = waypoints_it_->pose.orientation.y;
                msg.angular.z = waypoints_it_->pose.orientation.z;
              }
              cmd_vel_pub_.publish(msg);
              double timeout = atof(json_value["goal_timeout"].asString().c_str());
              if (timeout > 0) {
                  sleep(timeout);
                  geometry_msgs::Twist msg_stop;
                  cmd_vel_pub_.publish(msg_stop);
              }
              waypoints_it_++;
          }
          else if ("shell" == json_value["type"].asString())
          {
              std_msgs::String msg;
              msg.data = waypoints_it_->name;
              cmd_string_pub_.publish(msg);
              waypoints_it_++;
          }
          else if ("shutdown" == json_value["type"].asString())
          {
              int system_rtn = system("sudo shutdown -h now");
              waypoints_it_++;
          }
          else if ("dynparam" == json_value["type"].asString())
          { 
            if(AGV_NAME.size()!=0){//currently we cannot solve the namespace in simulation environment
              std::string system_str = "rosrun dynamic_reconfigure dynparam set";
              system_str += " ";
              system_str += "/" + AGV_NAME;
              system_str += json_value["node"].asString();
              system_str += " ";
              system_str += json_value["param"].asString();
              system_str += " ";
              system_str += json_value["value"].asString();
              int system_rtn = system(system_str.c_str());
            }else{
              std::string value;
              std::string ns=ros::this_node::getNamespace();
              if((int)json_value["value"].asString().find("'") == -1)
              {
                 value = "'" + json_value["value"].asString() + "'";
              }
              std::string system_str = "rosrun dynamic_reconfigure dynparam set /"+ns+json_value["node"].asString()+" "+json_value["param"].asString()+" " + value;
              int system_rtn = system(system_str.c_str());

            }
            waypoints_it_++;
          }
          else if ("sound_play" == json_value["type"].asString())
          {
              sound_play::SoundRequest msg;
              msg.sound = sound_play::SoundRequest::PLAY_FILE;
              if ("STOP" == json_value["failure_mode"].asString()) {
                msg.command = sound_play::SoundRequest::PLAY_STOP;
              } else if ("ONCE" == json_value["failure_mode"].asString()) {
                msg.command = sound_play::SoundRequest::PLAY_ONCE;
              } else if ("START" == json_value["failure_mode"].asString()) {
                msg.command = sound_play::SoundRequest::PLAY_START;
              }
              // msg.volume = atof(json_value["close_enough"].asString().c_str());
              // msg.arg = "/home/hitrobot/workspaces/hitrobot/dbparam/" + waypoints_it_->name + ".wav";
              msg.arg = "/home/hitrobot/workspaces/hitrobot/dbparam/sound.wav";
              msg.arg2 = "";
              robotsound_pub_.publish(msg);
              waypoints_it_++;
          }
          else if ("plc_io" == json_value["type"].asString())  /// TODO: move into rosbridge_driver later
          {
            std_msgs::Int16 msg;
            msg.data = atoi(json_value["plc_io"].asString().c_str());
            if (json_value["plc_io_cmd"].asString() == "set") {
              plc_io_set_pub_.publish(msg);
            } else if (json_value["plc_io_cmd"].asString() == "on") {
              plc_io_on_pub_.publish(msg);
            } else if (json_value["plc_io_cmd"].asString() == "off") {
              plc_io_off_pub_.publish(msg);
            }
            waypoints_it_++;
          }
          else if ("call_srv" == json_value["type"].asString())  /// TODO: move into rosbridge_driver later
          {
            std::string srv_name;
            std_srvs::Empty srv;
            srv_name = json_value["srv_name"].asString().c_str();
            if(ros::service::call(srv_name,srv))
            {
              ROS_INFO("Calling service %s in the waypoint", srv_name.c_str());
            }
            else{
              ROS_WARN("Calling %s service failed in the waypoint", srv_name.c_str());
            }
            waypoints_it_++;
          }
          else if ("initial_pose" == json_value["type"].asString())
          {
            geometry_msgs::PoseWithCovarianceStamped msg;
            msg.header.frame_id = json_value["frame_id"].asString();
            msg.header.stamp = ros::Time::now();
            bool init_msg = false;

            if (json_value.isMember("home")) {
              std::string home_name = json_value["home"].asString() + AGV_NAME;
              for (unsigned int wp = 0; wp < wp_list_.waypoints.size(); ++wp)
              {
                if (home_name == wp_list_.waypoints[wp].name)
                {
                  msg.pose.pose = wp_list_.waypoints[wp].pose;
                  init_msg = true;
                  break;
                }
              }
            } else {
              msg.pose.pose = waypoints_it_->pose;
              init_msg = true;
            }

            if (init_msg) {
              msg.pose.covariance[6*0+0] = 0.5 * 0.5;
              msg.pose.covariance[6*1+1] = 0.5 * 0.5;
              msg.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
              initialpose_pub_.publish(msg);
              ROS_INFO("Do init pose in waypoints");
            } else {
              ROS_ERROR("No init pose mateches");
            }
            waypoints_it_++;
            //state_ = ACTIVE;
          }
          else if ("scan_marker" == json_value["type"].asString())
          {
              std_msgs::Float64 msg;
              msg.data = atof(json_value["close_enough"].asString().c_str());

              std::string temp_frame_id = waypoints_it_->header.frame_id;
              double temp_yaw = waypoints_it_->pose.position.z;

              scan_marker_sub_ = nh_.subscribe("scan_marker/pose", 1, &WaypointsGoalNode::scan_marker_sub, this);
              usleep(200000); // wait for nh_.subscribe to registration if publish latch is false
              is_user_sub_ = false;
              while (ros::ok() && !is_user_sub_)
              {
                  scan_marker_pub_.publish(msg);
                  sleep(1); // resend after 1s
                  ros::spinOnce();
              }
              scan_marker_sub_.shutdown();
              if (state_ == IDLE)
              {
                  continue;
              }

              geometry_msgs::PoseStamped pose_in;
              geometry_msgs::PoseStamped pose_out;
              pose_in.header = waypoints_it_->header;
              pose_in.pose = waypoints_it_->pose;
              waypoints_it_->header.frame_id = temp_frame_id;
              waypoints_it_->pose.position.z = temp_yaw;
              // ROS_ERROR("haha: %s", pose_in.header.frame_id);
              tf::StampedTransform transform;
              try{
                  tf_listener_.waitForTransform("/base_link", "/base_laser", ros::Time(0), ros::Duration(5.0));
                  tf_listener_.transformPose("/map", pose_in, pose_out);
              }
              catch (tf::TransformException& ex){
                  ROS_ERROR("%s",ex.what());
                  exit(0);
              }

//              waypoints_it_++;
              mb_goal.target_pose.header = pose_out.header;
              mb_goal.target_pose.pose.position = pose_out.pose.position;
              // TODO use the heading from robot loc to next (front)
              mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose_out.pose.orientation) + temp_yaw);

              close_enough_ = 0.0;
              goal_timeout_ = atof(json_value["goal_timeout"].asString().c_str()) ? atof(json_value["goal_timeout"].asString().c_str()) : DBL_MAX;
              failure_mode_ = json_value["failure_mode"].asString();
              
              if (failure_mode_ == "LOOP2") {
                  std_msgs::String msg;
                  msg.data = "pub_loop2";
                  cmd_string_pub_.publish(msg);
              }

              ROS_INFO("New goal: %.2f, %.2f, %.2f",
                       mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                       tf::getYaw(mb_goal.target_pose.pose.orientation));
              move_base_ac_.sendGoal(mb_goal);

              state_ = ACTIVE;
          }

          else if ("pallet" == json_value["type"].asString())
          {
              geometry_msgs::PoseWithCovarianceStamped pose;
              pose.header.stamp = ros::Time::now();
              pose.header.frame_id = "base_laser_down";
              pose.pose.covariance[0] = 0.6; //0.06;
              pose.pose.covariance[7] = 1.6; //0.18;
              pose.pose.pose.position.x = waypoints_it_->pose.position.x;
              pose.pose.pose.position.y = waypoints_it_->pose.position.y;
              pose.pose.pose.orientation.z = waypoints_it_->pose.orientation.z;
              pose.pose.pose.orientation.w = waypoints_it_->pose.orientation.w;
              // msg.data = atof(json_value["close_enough"].asString().c_str());
              
              std::string temp_frame_id = waypoints_it_->header.frame_id;              
              
              marker_sub_ = nh_.subscribe("fine_pallet_pose", 1, &WaypointsGoalNode::scan_marker_sub, this);
              usleep(200000); // wait for nh_.subscribe to registration if publish latch is false
              is_user_sub_ = false;
              while (ros::ok() && !is_user_sub_)
              {
                  marker_pub_.publish(pose);
                  sleep(1); // resend after 1s
                  ros::spinOnce();
              }
              marker_sub_.shutdown();
              if (state_ == IDLE)
              {
                  continue;
              }
    
              geometry_msgs::PoseStamped pose_in;
              geometry_msgs::PoseStamped pose_out;
              pose_in.header = waypoints_it_->header;
              pose_in.pose = waypoints_it_->pose;
              waypoints_it_->header.frame_id = temp_frame_id;
              // ROS_ERROR("haha: %s", pose_in.header.frame_id.c_str());
              tf::StampedTransform transform;
              try{
                  tf_listener_.waitForTransform("/base_link", "/base_laser_down", ros::Time(0), ros::Duration(5.0));
                  tf_listener_.transformPose("/map", pose_in, pose_out);
              }
              catch (tf::TransformException& ex){
                  ROS_ERROR("%s",ex.what());
                  exit(0);
              }
    
    //              waypoints_it_++;
              mb_goal.target_pose.header = pose_out.header;
              // TODO use the heading from robot loc to next (front)
              double angle = atan2(pose_out.pose.orientation.z, pose_out.pose.orientation.w) * 2;
              double length = atof(json_value["close_enough"].asString().c_str());
              mb_goal.target_pose.pose.position = pose_out.pose.position;
              mb_goal.target_pose.pose.position.x = length * cos(angle) + pose_out.pose.position.x;
              mb_goal.target_pose.pose.position.y = length * sin(angle) + pose_out.pose.position.y;
              mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose_out.pose.orientation) + M_PI);
    
              close_enough_ = 0.0;
              goal_timeout_ = atof(json_value["goal_timeout"].asString().c_str()) ? atof(json_value["goal_timeout"].asString().c_str()) : DBL_MAX;
              failure_mode_ = json_value["failure_mode"].asString();
    
              // if (failure_mode_ == "LOOP2") {
              //     std_msgs::String msg;
              //     msg.data = "pub_loop2";
              //     cmd_string_pub_.publish(msg);
              // }
    
              ROS_INFO("New goal: %.2f, %.2f, %.2f",
                       mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                       tf::getYaw(mb_goal.target_pose.pose.orientation));
              move_base_ac_.sendGoal(mb_goal);
    
              state_ = ACTIVE;
          }

          else if ("tail" == json_value["type"].asString())
          {
            if (json_value.isMember("mark") && json_value["mark"].asString() == "true") {
              geometry_msgs::PoseStamped target_marker;
              tf::StampedTransform transform;
              try{
                tf_listener_.waitForTransform("/map", mark_.header.frame_id, ros::Time(0), ros::Duration(5.0));
                tf_listener_.transformPose("/map", mark_, target_marker);
              }
              catch (tf::TransformException& ex){
                ROS_ERROR("%s",ex.what());
                exit(0);
              }

              if (json_value.isMember("x_offset_pos")){
                x_offset_pos_ = atof(json_value["x_offset_pos"].asString().c_str());
              }
              
              if (json_value.isMember("x_offset_neg")){
                x_offset_neg_ = atof(json_value["x_offset_neg"].asString().c_str());
              }

              if (json_value.isMember("source_u")){
                source_u_ = atof(json_value["source_u"].asString().c_str());
              }

              if (json_value.isMember("target_v")){
                target_v_ = atof(json_value["target_v"].asString().c_str());
              }

              if (json_value.isMember("angular_ratio")){
                angular_ratio_ = atof(json_value["angular_ratio"].asString().c_str());
              }

              if (json_value.isMember("vel_ratio")){
                vel_ratio_ = atof(json_value["vel_ratio"].asString().c_str());
              }

              if (json_value.isMember("min_bez_vel")){
                min_bez_vel_ = atof(json_value["min_bez_vel"].asString().c_str());
              }

              if (json_value.isMember("max_bez_vel")){
                max_bez_vel_ = atof(json_value["max_bez_vel"].asString().c_str());
              }
              
              if (json_value.isMember("x_tolerance")){
                x_tolerance_ = atof(json_value["x_tolerance"].asString().c_str());
              }

              if (json_value.isMember("y_tolerance")){
                y_tolerance_ = atof(json_value["y_tolerance"].asString().c_str());
              }

              if (json_value.isMember("yaw_tolerance")){
                yaw_tolerance_ = atof(json_value["yaw_tolerance"].asString().c_str());
              }

              if (json_value.isMember("manual_comp")){
                manual_comp_ = atof(json_value["manual_comp"].asString().c_str());
              }

              flag_ = false;
              
              //TODO: use matrix for universal cases
              double yaw = tf::getYaw(target_marker.pose.orientation);
              double cos_yaw = cos(yaw);
              double sin_yaw = sin(yaw);
              tail_.pose.position = target_marker.pose.position;
              tail_.pose.position.x = target_marker.pose.position.x + waypoints_it_->pose.position.x * cos_yaw - waypoints_it_->pose.position.y * sin_yaw;
              tail_.pose.position.y = target_marker.pose.position.y + waypoints_it_->pose.position.x * sin_yaw + waypoints_it_->pose.position.y * cos_yaw;
              tail_.pose.position.z = target_marker.pose.position.z + waypoints_it_->pose.position.z;
              tail_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + tf::getYaw(waypoints_it_->pose.orientation));
            } else {
              geometry_msgs::PoseStamped msg;
              // msg.header.stamp = ros::Time::now();
              msg.header.frame_id = json_value["frame_id"].asString();
              msg.pose = waypoints_it_->pose;
              try{
                tf_listener_.waitForTransform("/map", msg.header.frame_id, ros::Time(0), ros::Duration(5.0));
                tf_listener_.transformPose("/map", msg, tail_);
              }
              catch (tf::TransformException& ex){
                ROS_ERROR("%s",ex.what());
                exit(0);
              }
            }
            
            sleep(1);
            robot_pose_sub_ = nh_.subscribe("robot_pose",  1, &WaypointsGoalNode::tail_sub, this);

            ros::Time time_in = ros::Time::now();
            ros::Duration time_out = ros::Duration(json_value.isMember("goal_timeout") ? atof(json_value["goal_timeout"].asString().c_str()) : DBL_MAX);
            is_user_sub_ = false;
            while (ros::ok() && !is_user_sub_)
            {
              usleep(10000);
              ros::spinOnce();
              if (ros::Time::now() - time_in > time_out)
              {
                geometry_msgs::Twist cmd_vel;
                cmd_vel_pub_.publish(cmd_vel);
                is_user_sub_ = true;
              }
            }
            robot_pose_sub_.shutdown();
            sleep(1);
            geometry_msgs::Twist cmd_vel;
            cmd_vel_pub_.publish(cmd_vel);
            
            if (state_ == IDLE)
            {
              continue;
            }

            waypoints_it_++;
          }
          else if ("mark" == json_value["type"].asString() || "workstation" == json_value["type"].asString())
          {
            if (!json_value.isMember("mark")) {
              geometry_msgs::PoseStamped msg;
              // msg.header.stamp = ros::Time::now();
              msg.header.frame_id = json_value["frame_id"].asString();
              msg.pose = waypoints_it_->pose;
              try{
                tf_listener_.waitForTransform("/map", msg.header.frame_id, ros::Time(0), ros::Duration(5.0));
                tf_listener_.transformPose("/map", msg, mark_);
              }
              catch (tf::TransformException& ex){
                ROS_ERROR("%s",ex.what());
                exit(0);
              }

              if (json_value.isMember("call")) {
                waypoints_it_++;
                for (unsigned int traj = 0; traj < traj_list_.trajectories.size(); ++traj)
                {
                  if (json_value["call"].asString() == traj_list_.trajectories[traj].name)
                  {
                    for (int wp = traj_list_.trajectories[traj].waypoints.size() - 1; wp >= 0; --wp)
                    {
                      waypoints_it_ = waypoints_.insert(waypoints_it_, traj_list_.trajectories[traj].waypoints[wp]);
                    }
                  }
                }
                waypoints_it_--;
              }
              // const geometry_msgs::PoseStamped msg_test;
              // const geometry_msgs::PoseStamped::ConstPtr msg_ptr = new geometry_msgs::PoseStamped(msg_test);
              // marker_sub(msg_ptr);
            } else if (json_value["mark"].asString() == "strip") {

              std_msgs::Float64 msg;
              msg.data = atof(json_value["close_enough"].asString().c_str());

              std::string temp_frame_id = waypoints_it_->header.frame_id;
              double temp_yaw = waypoints_it_->pose.position.z;

              scan_marker_sub_ = nh_.subscribe("scan_marker/pose", 1, &WaypointsGoalNode::scan_marker_sub, this);
              usleep(200000); // wait for nh_.subscribe to registration if publish latch is false
              is_user_sub_ = false;
              while (ros::ok() && !is_user_sub_)
              {
                  scan_marker_pub_.publish(msg);
                  sleep(1); // resend after 1s
                  ros::spinOnce();
              }
              scan_marker_sub_.shutdown();
              if (state_ == IDLE)
              {
                  continue;
              }

              geometry_msgs::PoseStamped pose_in;
              pose_in.header = waypoints_it_->header;
              pose_in.pose = waypoints_it_->pose;
              waypoints_it_->header.frame_id = temp_frame_id;
              waypoints_it_->pose.position.z = temp_yaw;
              // ROS_ERROR("haha: %s", pose_in.header.frame_id);
              tf::StampedTransform transform;
              try{
                  tf_listener_.waitForTransform("/base_link", "/base_laser", ros::Time(0), ros::Duration(5.0));
                  tf_listener_.transformPose("/map", pose_in, mark_);
              }
              catch (tf::TransformException& ex){
                  ROS_ERROR("%s",ex.what());
                  exit(0);
              }

              ROS_ERROR("frame_id: %s x: %f y: %f z: %f w: %f", mark_.header.frame_id.c_str(), mark_.pose.position.x, mark_.pose.position.y, mark_.pose.orientation.z, mark_.pose.orientation.w);

            } else if (json_value["mark"].asString() == "pallet") {
              geometry_msgs::PoseWithCovarianceStamped source_marker;
              // source_marker.header.stamp = ros::Time::now();
              source_marker.header.frame_id = json_value["frame_id"].asString();
              source_marker.pose.pose = waypoints_it_->pose;
              source_marker.pose.covariance[0] = 0.6; //atof(json_value["close_enough"].asString().c_str()); //0.6; //0.06;
              source_marker.pose.covariance[7] = 1.6; //atof(json_value["close_enough"].asString().c_str()); //1.6; //0.18;
              // source_marker.pose.covariance[35] = atof(json_value["close_enough"].asString().c_str());
  
              is_user_sub_ = false;
              while (ros::ok() && !is_user_sub_)
              {
                marker_pub_.publish(source_marker);
                sleep(1); // resend after 1s
                ros::spinOnce();
              }
              if (state_ == IDLE)
              {
                continue;
              }
              ROS_ERROR("frame_id: %s x: %f y: %f z: %f w: %f", mark_.header.frame_id.c_str(), mark_.pose.orientation.x, mark_.pose.orientation.y, mark_.pose.orientation.z, mark_.pose.orientation.w);
            } else {
              ROS_ERROR("unknown mark type");
            }
              
            waypoints_it_++;
          }
          else if ("goto" == json_value["type"].asString())
          {
            yocs_msgs::NavigationControl nav_ctrl;
            nav_ctrl.control = yocs_msgs::NavigationControl::START;
            nav_ctrl.goal_name = json_value["failure_mode"].asString();
            nav_ctrl_pub_.publish(nav_ctrl);
          }
          else if ("call" == json_value["type"].asString())
          {
            waypoints_it_++;
            for (unsigned int traj = 0; traj < traj_list_.trajectories.size(); ++traj)
            {
              if (json_value["failure_mode"].asString() == traj_list_.trajectories[traj].name)
              {
                for (int wp = traj_list_.trajectories[traj].waypoints.size() - 1; wp >= 0; --wp)
                {
                  waypoints_it_ = waypoints_.insert(waypoints_it_, traj_list_.trajectories[traj].waypoints[wp]);
                }
              }
            }
          }
          else if ("goal" == json_value["type"].asString())
          {
            mb_goal.target_pose.header.stamp = ros::Time::now();
            mb_goal.target_pose.header.frame_id = json_value["frame_id"].asString();

            if (json_value.isMember("mark") && json_value["mark"].asString() == "true") {
              if (json_value.isMember("line") && json_value["line"].asString() == "true") {
                robot_pose_sub_ = nh_.subscribe("robot_pose",  1, &WaypointsGoalNode::line_sub, this);
    
                is_user_sub_ = false;
                while (ros::ok() && !is_user_sub_)
                {
                  usleep(10000);
                  ros::spinOnce();
                }
                robot_pose_sub_.shutdown();
              }
                
              geometry_msgs::PoseStamped target_marker;
              tf::StampedTransform transform;
              try{
                tf_listener_.waitForTransform(json_value["frame_id"].asString(), mark_.header.frame_id, ros::Time(0), ros::Duration(5.0));
                tf_listener_.transformPose(json_value["frame_id"].asString(), mark_, target_marker);
              }
              catch (tf::TransformException& ex){
                ROS_ERROR("%s",ex.what());
                exit(0);
              }

              //TODO: use matrix for universal cases
              double yaw = tf::getYaw(target_marker.pose.orientation);
              double cos_yaw = cos(yaw);
              double sin_yaw = sin(yaw);
              mb_goal.target_pose.pose.position = target_marker.pose.position;
              mb_goal.target_pose.pose.position.x = target_marker.pose.position.x + waypoints_it_->pose.position.x * cos_yaw - waypoints_it_->pose.position.y * sin_yaw;
              mb_goal.target_pose.pose.position.y = target_marker.pose.position.y + waypoints_it_->pose.position.x * sin_yaw + waypoints_it_->pose.position.y * cos_yaw;
              mb_goal.target_pose.pose.position.z = target_marker.pose.position.z + waypoints_it_->pose.position.z;
              mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + tf::getYaw(waypoints_it_->pose.orientation));
            } else {
              mb_goal.target_pose.pose = waypoints_it_->pose;
              //        mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // TODO use the heading from robot loc to next (front)
            }

            close_enough_ = atof(json_value["close_enough"].asString().c_str());
            goal_timeout_ = atof(json_value["goal_timeout"].asString().c_str()) ? atof(json_value["goal_timeout"].asString().c_str()) : DBL_MAX;
            failure_mode_ = json_value["failure_mode"].asString();

            ROS_INFO("New goal: %.2f, %.2f, %.2f",
                     mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                     tf::getYaw(mb_goal.target_pose.pose.orientation));
            move_base_ac_.sendGoal(mb_goal);

            state_ = ACTIVE;
          }
          else
          {
            ROS_ERROR_STREAM("Illegal type.");
          }

          if (START == state_ && waypoints_it_ >= waypoints_.end())
          {
            waypoints_it_--;
            ROS_INFO_STREAM("haha: No more way points to go to.");
            state_ = COMPLETED;
          }
      }
      else
      {
        ROS_ERROR_STREAM("Cannot start execution. Already at the last way point.");
        idle_status_update_sent_ = false;
        state_ = IDLE;
      }

      // TODO: This is a horrible workaround for a problem I cannot solve: send a new goal
      // when the previous one has been cancelled return immediately with succeeded state
      //
      // Marcus: Don't understand this case (yet). Commenting out until we need it.
//        int times_sent = 0;
//        while ((move_base_ac_.waitForResult(ros::Duration(0.1)) == true) &&
//               (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
//        {
//          move_base_ac_.sendGoal(mb_goal);
//          times_sent++;
//        }
//        if (times_sent > 1)
//        {
//          ROS_WARN("Again the strange case of instantaneous goals... (goal sent %d times)", times_sent);
//        }
    }
    else if (state_ == ACTIVE)
    {
      actionlib::SimpleClientGoalState goal_state = move_base_ac_.getState();

      // We are still pursuing a goal...
      if ((goal_state == actionlib::SimpleClientGoalState::ACTIVE) ||
          (goal_state == actionlib::SimpleClientGoalState::PENDING) ||
          (goal_state == actionlib::SimpleClientGoalState::RECALLED) ||
          (goal_state == actionlib::SimpleClientGoalState::PREEMPTED))
      {
        // check if we timed out
        if ((ros::Time::now() - mb_goal.target_pose.header.stamp).toSec() >= goal_timeout_)
        {
          ROS_WARN("Cannot reach goal after %.2f seconds; request a new one (current state is %s)",
                    goal_timeout_, move_base_ac_.getState().toString().c_str());
          if (waypoints_it_ < (waypoints_.end() - 1))
          {
            ROS_INFO_STREAM("Requesting next way point.");
            waypoints_it_++;
            state_ = START;
          }
          else
          {
            ROS_INFO_STREAM("No more way points to go to.");
            state_ = COMPLETED;
          }
        }
        // When close enough to current goal (except for the final one!), go for the
        // next waypoint, so we avoid the final slow approach and subgoal obsession
        // if (waypoints_it_ < waypoints_.end())
        if (waypoints_it_ < (waypoints_.end() - 1))
        {
          tf::StampedTransform robot_gb, goal_gb;
          try
          {
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gb);
          }
          catch (tf::TransformException& e)
          {
            ROS_WARN("Cannot get tf %s -> %s: %s", world_frame_.c_str(), robot_frame_.c_str(), e.what());
            continue;
          }

          mtk::pose2tf(mb_goal.target_pose, goal_gb);
          double distance = mtk::distance2D(robot_gb, goal_gb);
          if (distance <= fabs(close_enough_))
          {
            if (waypoints_it_ < (waypoints_.end() - 1))
            {
              ROS_INFO("Close enough to current goal (%.2f <= %.2f m).", distance, close_enough_);
              ROS_INFO_STREAM("Requesting next way point.");
              waypoints_it_++;
              state_ = START;
              if (close_enough_ < 0) {
                cancelAllGoals();
              }
            }
            else
            {
              ROS_INFO_STREAM("Reached final way point.");
              cancelAllGoals();
              state_ = COMPLETED;
            }
          }
          else
          {
            // keep going until get close enough
          }
        }
        else
        {
          // keep going, since we approaching last way point
        }
      }
      else // actionlib::SimpleClientGoalState::SUCCEEDED, REJECTED, ABORTED, LOST
      {
        if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Go to goal successfully completed: %.2f, %.2f, %.2f",
                   mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                   tf::getYaw(mb_goal.target_pose.pose.orientation));
          if (waypoints_it_ < (waypoints_.end() - 1))
          {
            ROS_INFO_STREAM("Requesting next way point.");
            waypoints_it_++;
            state_ = START;
          }
          else
          {
            ROS_INFO_STREAM("Reached final way point.");
            state_ = COMPLETED;
          }
        }
        else
        {
          ROS_ERROR("Go to goal failed: %s.", move_base_ac_.getState().toString().c_str());
          if ("LOOP" == failure_mode_)
          {
              ROS_INFO_STREAM("Requesting this way point again.");
              state_ = START;
          }
          else if ("LOOP2" == failure_mode_)
          {
              ROS_INFO_STREAM("Requesting this way point without scan_marker again.");
              geometry_msgs::Twist msg;
              msg.linear.x = -0.1;
              cmd_vel_pub_.publish(msg);
              sleep(1);
              msg.linear.x = 0.0;
              cmd_vel_pub_.publish(msg);

              ROS_INFO("New goal: %.2f, %.2f, %.2f",
                       mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                       tf::getYaw(mb_goal.target_pose.pose.orientation));
              move_base_ac_.sendGoal(mb_goal);
              state_ = ACTIVE;
          }
          else if ("BACK" == failure_mode_)
          {
              if (waypoints_it_ > waypoints_.begin())
              {
                ROS_INFO_STREAM("Requesting prev way point.");
                  waypoints_it_--;
                  state_ = START;
              }
              else
              {
                ROS_INFO_STREAM("No more prev way points to go to.");
                  state_ = COMPLETED;
              }
          }
          else if ("BACK2" == failure_mode_)
          {
              if (waypoints_it_ > (waypoints_.begin() + 1))
              {
                  geometry_msgs::Twist msg;
                  msg.linear.x = -0.1;
                  cmd_vel_pub_.publish(msg);
                  sleep(5);
                  msg.linear.x = 0.0;
                  cmd_vel_pub_.publish(msg);
                  ROS_INFO_STREAM("Requesting prev2 way point.");
                  waypoints_it_--;
                  waypoints_it_--;
                  state_ = START;
              }
              else
              {
                  ROS_INFO_STREAM("No more prev2 way points to go to.");
                  state_ = COMPLETED;
              }
          }
          else
          {
              if (waypoints_it_ < (waypoints_.end() - 1))
              {
                ROS_INFO_STREAM("Requesting next way point.");
                waypoints_it_++;
                state_ = START;
              }
              else
              {
                ROS_INFO_STREAM("No more next way points to go to.");
                state_ = COMPLETED;
              }
          }
        }
      }
    }
    else if(state_ == COMPLETED)
    {
      // publish update
      publishStatusUpdate(yocs_msgs::NavigationControlStatus::COMPLETED, waypoints_it_->name);
      idle_status_update_sent_ = false;
      state_ = IDLE;
    }
    else // IDLE
    {
      if (!idle_status_update_sent_)
      {
        publishStatusUpdate(yocs_msgs::NavigationControlStatus::IDLING, "");
        idle_status_update_sent_ = true;
      }
    }
  }
}

bool WaypointsGoalNode::equals(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
  return ((a.pose.position.x == b.pose.position.x) &&
          (a.pose.position.y == b.pose.position.y) &&
          (a.pose.position.z == b.pose.position.z));
  // TODO make decent, with rotation (tk::minAngle, I think) and frame_id and put in math toolkit
}

bool WaypointsGoalNode::equals(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
  return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

void WaypointsGoalNode::publishStatusUpdate(const uint8_t& status, const std::string& waypoint_name)
{
  yocs_msgs::NavigationControlStatus msg;
  scheduling_msgs::ReportNavigationControlStatus srv; //As that we only care about the status COMPLETED, CANCELLED and ERROR, we call service when the status is the one of three
  if (status == yocs_msgs::NavigationControlStatus::IDLING)
  {
    msg.status = yocs_msgs::NavigationControlStatus::IDLING;
    msg.status_desc = "Idling";
    msg.waypoint_name = waypoint_name;
    nav_ctrl_status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::RUNNING)
  {
    msg.status = yocs_msgs::NavigationControlStatus::RUNNING;
    msg.status_desc = "Navigating to way point.";
    msg.waypoint_name = waypoint_name;
    nav_ctrl_status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::PAUSED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::PAUSED;
    msg.status_desc = "Navigation on hold.";
    msg.waypoint_name = waypoint_name;
    nav_ctrl_status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::COMPLETED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::COMPLETED;
    msg.status_desc = "Reached final destination.";
    msg.waypoint_name = waypoint_name;
    srv.request.status= status;
    srv.request.waypoint_name=waypoint_name;
    if(report_nav_ctrl_status_using_service_){
        if(!reportNavigationControlStatus(srv))
            ROS_ERROR_STREAM("Failed to call nav_ctrl_status_service: "<<waypoint_name);
    }
    nav_ctrl_status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::CANCELLED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::CANCELLED;
    msg.status_desc = "Navigation cancelled.";
    msg.waypoint_name = waypoint_name;
    srv.request.status= status;
    srv.request.waypoint_name=waypoint_name;
    if(report_nav_ctrl_status_using_service_){
        if(!reportNavigationControlStatus(srv))
            ROS_ERROR_STREAM("Failed to call nav_ctrl_status_service: "<<waypoint_name);
    }
    nav_ctrl_status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::ERROR)
  {
    msg.status = yocs_msgs::NavigationControlStatus::ERROR;
    msg.status_desc = "An error occurred.";
    msg.waypoint_name = waypoint_name;
    srv.request.status= status;
    srv.request.waypoint_name=waypoint_name;
    if(report_nav_ctrl_status_using_service_){
        if(!reportNavigationControlStatus(srv))
            ROS_ERROR_STREAM("Failed to call nav_ctrl_status_service: "<<waypoint_name);
    }
    nav_ctrl_status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::SUB_CANCELLED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::SUB_CANCELLED;
    msg.status_desc = "subscriber cancelled.";
    msg.waypoint_name = waypoint_name;
    nav_ctrl_status_pub_.publish(msg);
  }
  else
  {
    ROS_ERROR_STREAM("Cannot publish unknown status updated!");
  }
}

void WaypointsGoalNode::waypoint_user_sub(const std_msgs::String::ConstPtr& msg)
{
  if (waypoints_it_ != waypoints_.end())
  {
    int pos = msg->data.find(':');
    if (pos)
    {
      // deal with warn exception (e.g. timeout)
        if (msg->data.substr(0, pos) == "warn" && msg->data.substr(pos + 1, msg->data.length() - pos - 1) == waypoints_it_->name) {
            user_data_ = "warn";
            is_user_sub_ = true;
        }
        else if (msg->data.substr(0, pos) == waypoints_it_->name)
        {
            user_data_ = msg->data.substr(pos + 1, msg->data.length() - pos - 1);
            is_user_sub_ = true;
        }
        else if (msg->data.substr(0, pos) == "sub_manual")
        {
          int pos2 = msg->data.rfind(',');
          std::string code2 = msg->data.substr(pos2 + 1, msg->data.length() - pos2 - 1);
          if (code2 == "512")
          {
            // ROS_INFO_STREAM("Stopping current execution ...");
            // cancelAllGoals();
            // resetWaypoints();
            // ROS_INFO_STREAM("Current execution stopped.");
            idle_status_update_sent_ = false;
            state_ = IDLE;
            is_user_sub_ = true;
            is_user_sub_ok_ = true;
            yocs_msgs::NavigationControl nav_ctrl;
            nav_ctrl.control = yocs_msgs::NavigationControl::START;
            nav_ctrl.goal_name = "power_off";
            nav_ctrl_pub_.publish(nav_ctrl);
            waypoint_user_sub_.shutdown();
          }
          std_msgs::Bool manual_pub;
          manual_pub.data = (atoi(code2.c_str()) & 0x04);
          cmd_vel_manual_pub_.publish(manual_pub);
        }
    }
    else
    {
        if (msg->data == waypoints_it_->name)
        {
            is_user_sub_ = true;
        }
    }
  }
}

void WaypointsGoalNode::scan_marker_sub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    waypoints_it_->header = msg->header;
    waypoints_it_->pose.position.x = msg->pose.position.x;
    waypoints_it_->pose.position.y = msg->pose.position.y;
    waypoints_it_->pose.position.z = msg->pose.position.z;
    waypoints_it_->pose.orientation.w = msg->pose.orientation.w;
    waypoints_it_->pose.orientation.x = msg->pose.orientation.x;
    waypoints_it_->pose.orientation.y = msg->pose.orientation.y;
    waypoints_it_->pose.orientation.z = msg->pose.orientation.z;

    is_user_sub_ = true;
}

void WaypointsGoalNode::marker_sub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  try{
    tf_listener_.waitForTransform("/map", msg->header.frame_id, ros::Time::now(), ros::Duration(5.0));
    tf_listener_.transformPose("/map", *msg, mark_);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    exit(0);
  }
  // mark_.header = msg->header;
  // mark_.pose = msg->pose;

  is_user_sub_ = true;
}

void WaypointsGoalNode::line_sub(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::Pose2D pr, ps, pt;

  ps.x = msg->position.x;
  ps.y = msg->position.y;
  ps.theta = tf::getYaw(msg->orientation);
  pt.x = mark_.pose.position.x;
  pt.y = mark_.pose.position.y;
  pt.theta = tf::getYaw(mark_.pose.orientation);

  pr.theta = atan2(ps.y - pt.y, ps.x - pt.x);
  double pr_len = sqrt(pow(ps.x - pt.x, 2) + pow(ps.y - pt.y, 2));
  double px_len = pr_len * cos(pr.theta -pt.theta);

  int pos = (fmod(ps.theta - pt.theta + M_PI*2, M_PI*2) < M_PI) ? 1 : -1;
  waypoints_it_->pose.position.x = px_len;
  waypoints_it_->pose.position.y = fabs(waypoints_it_->pose.position.y) * pos;
  waypoints_it_->pose.orientation.z = fabs(waypoints_it_->pose.orientation.z) * pos;

  is_user_sub_ = true;
}

void WaypointsGoalNode::tail_sub(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Pose2D pr, ps, pt, pu, pv, trans;
  // std::vector<geometry_msgs::Pose2D> pl;
  
  ps.x = msg->position.x;
  ps.y = msg->position.y;
  ps.theta = tf::getYaw(msg->orientation);
  double ps_tan = tan(ps.theta);
  if(flag_)
  {
    if(flag2_ == 1)
    {
      pt.x = tail_.pose.position.x + manual_comp_;
      ROS_ERROR("X: %f",pt.x);
    }
    else if(flag2_ == 2)
    {
      pt.x = tail_.pose.position.x - manual_comp_;
      ROS_ERROR("X: %f, flag: %d",pt.x, flag2_);
    }
  }
  else{
    pt.x = tail_.pose.position.x;
  }
  pt.y = tail_.pose.position.y;
  pt.theta = tf::getYaw(tail_.pose.orientation);
  double pt_tan = tan(pt.theta);
  double trans_yaw = pt.theta - M_PI_2;
  trans.x = ps.x * cos(trans_yaw) + ps.y * sin(trans_yaw) - pt.x;
  trans.y = - ps.x * sin(trans_yaw) + ps.y * cos(trans_yaw) - pt.y;
  double judge_yaw = atan2(trans.y, trans.x);
  if(!flag_)
  { 
    if(judge_yaw < M_PI_2)
    {
      flag2_ = 1;
    }
    else if(judge_yaw > M_PI_2)
    {
      flag2_ = 2;
    }
    flag_ = true;
  }
  pr.x = (ps_tan * ps.x - pt_tan * pt.x - ps.y + pt.y) / (ps_tan - pt_tan);
  pr.y = ps_tan * (pr.x - ps.x) + ps.y;
  pr.theta = atan2(ps.y - pt.y, ps.x - pt.x);
  double pr_len = sqrt(pow(ps.x - pt.x, 2) + pow(ps.y - pt.y, 2));
  double px_len = pr_len * cos(pr.theta -pt.theta);
  // ROS_ERROR("x: %f, y: %f, judge_yaw: %f", trans.x,trans.y,judge_yaw,pr.theta);

  // if (fabs(px_len) < x_tolerance) {
  //   cmd_vel_pub_.publish(cmd_vel);
  //   is_user_sub_ = true;
  //   return;
  // }

  if((fabs(ps.x - pt.x) <= x_tolerance_) 
      && (fabs(ps.y - pt.y) <= y_tolerance_) && (angles::shortest_angular_distance(ps.theta, pt.theta) <= yaw_tolerance_))
  {
    cmd_vel_pub_.publish(cmd_vel);
    is_user_sub_ = true;
    return;
  }

  double px_offset = (pr_len > (x_offset_pos_ + x_offset_neg_)) ? x_offset_pos_ : (pr_len - x_offset_neg_);
  double pr_dir = (fabs(pt.theta - pr.theta) > M_PI) ? (2*M_PI - fabs(pt.theta - pr.theta)) : (fabs(pt.theta - pr.theta));
  if (pr_dir < M_PI_2) {
    pr_len = - pr_len;
    px_offset = - px_offset;
  }
  pt.x = pt.x - px_offset * cos(pt.theta);
  pt.y = pt.y - px_offset * sin(pt.theta);

  // bool dir1 = (fabs(ps_tan) < 1) ? ((fabs(ps.theta) < M_PI_2) ? (pr.x > ps.x) : (ps.x > pr.x)) : ((ps.theta > 0) ? (pr.y > ps.y) : (ps.y > pr.y));
  // bool dir2 = (fabs(pt_tan) < 1) ? ((fabs(pt.theta) < M_PI_2) ? (pr.x > pt.x) : (pt.x > pr.x)) : ((pt.theta > 0) ? (pr.y > pt.y) : (pt.y > pr.y));
  // bool dir0 = (dir1) ? (!dir2) : (dir2);

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
  // double ddx = 3*dx_a*tSquared + 2*dx_b*t + dx_c;
  // double ddy = 3*dy_a*tSquared + 2*dy_b*t + dy_c; 
  double dxy = sqrt(pow(dx - ps.x, 2) + pow(dy - ps.y, 2));
  double dyaw_temp = ps.theta - (pr_len < 0 ? M_PI : 0) * (ps.theta > 0 ? 1 : -1);

  double dyaw2 = atan2(dy-ps.y, dx-ps.x);
  double dyaw = fmod(dyaw2 - dyaw_temp, M_PI * 2);
  if (dyaw > M_PI) dyaw -= M_PI*2;
  if (dyaw < -M_PI) dyaw += M_PI*2;
  // ROS_ERROR("ps.theta: %f, dyaw_temp: %f, dyaw2: %f, dyaw: %f", ps.theta, dyaw_temp, dyaw2, dyaw);
  // double dyaw = atan2(ddy, ddx) - ps.theta;
  
  cmd_vel.linear.x = (pr_len > 0 ? 1 : -1);
  cmd_vel.angular.z = fabs(cmd_vel.linear.x / dxy) * dyaw * angular_ratio_;
  double cmd_vel_factor = (1 / sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.angular.z * wheel_base_, 2)));
  cmd_vel_factor *= fmin(vel_ratio_ * fmax(fabs(px_len) - x_offset_pos_, 0) + min_bez_vel_, max_bez_vel_);
  cmd_vel.linear.x *= cmd_vel_factor;
  cmd_vel.angular.z *= cmd_vel_factor;

  cmd_vel_pub_.publish(cmd_vel);              

  nav_msgs::Path gui_p;
  gui_p.header.frame_id = "map";
  gui_p.header.stamp = ros::Time::now();
  gui_p.poses.resize(4);
  gui_p.poses[0].pose.position.x = ps.x;
  gui_p.poses[0].pose.position.y = ps.y;
  gui_p.poses[1].pose.position.x = pu.x;
  gui_p.poses[1].pose.position.y = pu.y;
  gui_p.poses[2].pose.position.x = pv.x;
  gui_p.poses[2].pose.position.y = pv.y;
  gui_p.poses[3].pose.position.x = pt.x;
  gui_p.poses[3].pose.position.y = pt.y;
  gui_p_pub_.publish(gui_p);

  int gui_int = 10;
  double gui_float = 1 / gui_int;
  nav_msgs::Path gui_q;
  gui_q.header.frame_id = "map";
  gui_q.header.stamp = ros::Time::now();
  gui_q.poses.resize(gui_int);
  for (int i = 0; i < gui_int; i++) {
    t = gui_float * (i + 1);
    tSquared = pow(t,2);
    tCubed = tSquared*t;
    gui_q.poses[i].pose.position.x = dx_a*tCubed + dx_b*tSquared\
            + dx_c*t + dx_d;
    gui_q.poses[i].pose.position.y = dy_a*tCubed + dy_b*tSquared\
            + dy_c*t + dy_d;
    gui_q.poses[i].pose.orientation.w = 1;
  }
  gui_q_pub_.publish(gui_q);
}

} // namespace yocs
