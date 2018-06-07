/********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/
#include <bz_local_planner/pose_helper_ros.h>

namespace bz_local_planner {

PoseHelperRos::PoseHelperRos(std::string pose_topic) {
  setPoseTopic( pose_topic);
  pose_gained_ = false;
}

void PoseHelperRos::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_ONCE("pose received!");
    pose_gained_ = true;

  //we assume that the Pose is published in the frame of the base
  boost::mutex::scoped_lock lock(pose_mutex_);
  base_pose_.pose.position.x = msg->pose.position.x;
  base_pose_.pose.position.y = msg->pose.position.y;
  base_pose_.pose.position.z = msg->pose.position.z;
  base_pose_.pose.orientation.z = msg->pose.orientation.z;
  base_pose_.pose.orientation.w = msg->pose.orientation.w;
  base_pose_.header.frame_id = msg->header.frame_id;
//  ROS_DEBUG_NAMED("dwa_local_planner", "In the Pose callback with velocity values: (%.2f, %.2f, %.2f)",
//      base_pose_.twist.twist.linear.x, base_pose_.twist.twist.linear.y, base_pose_.twist.twist.angular.z);
}

//copy over the Pose information
void PoseHelperRos::getPose(geometry_msgs::PoseStamped& base_pose) {
  boost::mutex::scoped_lock lock(pose_mutex_);
  base_pose = base_pose_;
  base_pose.header.stamp = ros::Time();
}

void PoseHelperRos::setPoseTopic(std::string pose_topic)
{
  if( pose_topic != pose_topic_ )
  {
    pose_topic_ = pose_topic;

    if( pose_topic_ != "" )
    {
      ros::NodeHandle gn;
      pose_sub_ = gn.subscribe<geometry_msgs::PoseStamped>( pose_topic_, 1000, boost::bind( &PoseHelperRos::poseCallback, this, _1 ));
    }
    else
    {
      pose_sub_.shutdown();
    }
  }
}

} /* namespace bz_local_planner */
