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

#ifndef POSE_HELPER_ROS_H_
#define POSE_HELPER_ROS_H_

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace bz_local_planner {

class PoseHelperRos {
public:

  /** @brief Constructor.
   * @param odom_topic The topic on which to subscribe to Pose
   *        messages.  If the empty string is given (the default), no
   *        subscription is done. */
  PoseHelperRos(std::string odom_topic = "");
  ~PoseHelperRos() {}

  /**
   * @brief  Callback for receiving Pose data
   * @param msg An Pose message
   */
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void getPose(geometry_msgs::PoseStamped& base_pose);

  /** @brief Set the Pose topic.  This overrides what was set in the constructor, if anything.
   *
   * This unsubscribes from the old topic (if any) and subscribes to the new one (if any).
   *
   * If odom_topic is the empty string, this just unsubscribes from the previous topic. */
  void setPoseTopic(std::string pose_topic);

  /** @brief Return the current Pose topic. */
  std::string getPoseTopic() const { return pose_topic_; }
  
  bool getPoseStatus() const { return pose_gained_; }

  friend class BZPlannerROS;

private:
  //odom topic
  std::string pose_topic_;
  
  // we listen on Pose on the odom topic
  ros::Subscriber pose_sub_;
  geometry_msgs::PoseStamped base_pose_;
  boost::mutex pose_mutex_;
  
  bool pose_gained_;
  // global tf frame id
  std::string frame_id_; ///< The frame_id associated this data
};

} /* namespace bz_local_planner */
#define CHUNKY 1
#endif /* POSE_HELPER_ROS_H_ */
