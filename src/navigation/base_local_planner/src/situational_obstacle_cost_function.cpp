/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, HRG
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 * Author: Honggang Gou
 *********************************************************************/
#include <base_local_planner/situational_obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {
SituationalObstacleCostFunction::SituationalObstacleCostFunction(costmap_2d::Costmap2D* global_costmap, costmap_2d::Costmap2D* local_costmap) 
    : global_costmap_(global_costmap), local_costmap_(local_costmap), sum_scores_(false) {
  if (local_costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*local_costmap);
    obfun_ = boost::shared_ptr<base_local_planner::ObstacleCostFunction>(new base_local_planner::ObstacleCostFunction(local_costmap));
  }
}

SituationalObstacleCostFunction::~SituationalObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
  delete obfun_;
}

void SituationalObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed, bool judge_by_maximum, bool sum_scores) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
  judge_by_maximum_ = judge_by_maximum;
  sum_scores_ = sum_scores;
}

void SituationalObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool SituationalObstacleCostFunction::prepare() {
  return true;
}

double SituationalObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  traj_status_ = trajectoryClassifier(traj, global_costmap_, local_costmap_);
  switch(traj_status_){
    case UNKNOWN_ZONE :{
      //TODO: distinguish unknown and normal ?
      return obfun_->scoreTrajectory(traj);
    }
    case NORMAL_ZONE :{
      return obfun_->scoreTrajectory(traj);
    }
    case CAUTIOUS_ZONE :{
      for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
        traj.getPoint(i, px, py, pth);
        double f_cost = footprintCost(px, py, pth,
            scale, footprint_spec_,
            local_costmap_, world_model_);
        if(f_cost < 0){
          // if the trajectory hits an obstacle, we prefer a smooth pausing maneuver rather than avoiding.
          return DBL_MAX;
        }
        if(sum_scores_)
            cost +=  f_cost;
        else
            cost = std::max(cost, f_cost);
            
        return cost;
      }
    }
    default :{
      ROS_ERROR("Situational_obstacle_cost_function: Trajectory status not defined.");
      return -1;
    }
  }
}

unsigned int SituationalObstacleCostFunction::getCellStatus(const double& x, const double& y, 
                                                                    costmap_2d::Costmap2D* global_costmap, 
                                                                    costmap_2d::Costmap2D* local_costmap) {   
  unsigned int cx, cy, mx, my = 0;
  if(!local_costmap->worldToMap(x, y, cx, cy)){
    return UNKNOWN_ZONE;
  }
  //to do: bounds check?
  global_costmap->worldToMap(x, y, mx, my);
  if((global_costmap->getCost(mx,my) <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) && (global_costmap->getCost(mx,my) >= costmap_2d::FREE_SPACE)){
    return CAUTIOUS_ZONE;
  }
  else if(global_costmap->getCost(mx,my) == costmap_2d::NO_INFORMATION){
    return UNKNOWN_ZONE;
  }
  else{
    return NORMAL_ZONE;
  }
}

unsigned int SituationalObstacleCostFunction::trajectoryClassifier(Trajectory &traj, costmap_2d::Costmap2D* global_costmap,
                                                                          costmap_2d::Costmap2D* local_costmap) {
  double px, py, pth;
  unsigned int max_count, temp;
  clearPointStatus();
  clearTrajStatus();
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    points_list_.push_back(getCellStatus(px, py, global_costmap, local_costmap));
  }
  if(!points_list_.empty()){
    for(auto iter = points_list_.begin(); iter != points_list_.end(); ++iter){
      ++point_status_counts_[*iter];
    }
    if(judge_by_maximum_){
      temp = (point_status_counts_[CAUTIOUS_ZONE] >= point_status_counts_[NORMAL_ZONE]) ? CAUTIOUS_ZONE : NORMAL_ZONE; 
      max_count = (temp >= point_status_counts_[UNKNOWN_ZONE]) ? temp : UNKNOWN_ZONE;
      return max_count;
    }
    eles{
      if(0 != point_status_counts_[UNKNOWN_ZONE])
        return UNKNOWN_ZONE;
      else{
        return (point_status_counts_[CAUTIOUS_ZONE] >= point_status_counts_[NORMAL_ZONE]) ? CAUTIOUS_ZONE : NORMAL_ZONE;
      }
    }
  }
}

unsigned int SituationalObstacleCostFunction::getTrajStatus() {
  return traj_status_;
}

void SituationalObstacleCostFunction::clearPointStatus(){
  if(points_list_.empty())
    return;
  else
    points_list_.clear();
}

void SituationalObstacleCostFunction::clearTrajStatus(){
  if(point_status_counts_.empty())
    return;
  else
    point_status_counts_.clear();
}

double SituationalObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double SituationalObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* local_costmap,
    base_local_planner::WorldModel* world_model) {
  
  return obfun_->footprintCost(x, y, th, scale, footprint_spec, local_costmap, world_model);
}

} /* namespace base_local_planner */
