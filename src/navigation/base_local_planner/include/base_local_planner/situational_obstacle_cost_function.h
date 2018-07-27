/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, HRG.
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
 *  Author: Honggang Gou
 *********************************************************************/

#ifndef SITUATIONAL_OBSTACLE_COST_FUNCTION_H_
#define SITUATIONAL_OBSTACLE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/situational_map_grid.h>
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {
static const unsigned int UNKNOWN_ZONE = 2;
static const unsigned int CAUTIOUS_ZONE = 1;
static const unsigned int NORMAL_ZONE = 0;

/**
 * class SituationalObstacleCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class SituationalObstacleCostFunction : public TrajectoryCostFunction {

public:
  SituationalObstacleCostFunction(costmap_2d::Costmap2D* global_costmap, costmap_2d::Costmap2D* local_costmap);
  ~SituationalObstacleCostFunction();

  bool prepare();
  double scoreTrajectory(Trajectory &traj);

  void setSumScores(bool score_sums){ sum_scores_=score_sums; }

  void setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed, bool judge_by_maximum);
  void setFootprint(std::vector<geometry_msgs::Point> footprint_spec);

  // helper functions, made static for easy unit testing
  static double getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
  static double footprintCost(
      const double& x,
      const double& y,
      const double& th,
      double scale,
      std::vector<geometry_msgs::Point> footprint_spec,
      costmap_2d::Costmap2D* local_costmap,
      base_local_planner::WorldModel* world_model);
  
  inline unsigned int getCellStatus(const double& x, const double& y, costmap_2d::Costmap2D* global_costmap, costmap_2d::Costmap2D* local_costmap);
  
  unsigned int trajectoryClassifier(Trajectory &traj, costmap_2d::Costmap2D* global_costmap, costmap_2d::Costmap2D* local_costmap);

  void clearPointStatus();
  void clearTrajStatus();

private:
  costmap_2d::Costmap2D* local_costmap_;
  costmap_2d::Costmap2D* global_costmap_;
  std::vector<geometry_msgs::Point> footprint_spec_;
  std::vector<unsigned int> points_list_;
  std::map<unsigned int, size_t> point_status_counts_;
  base_local_planner::WorldModel* world_model_;
  base_local_planner::SituationalMapGrid* map_grid_;
  double max_trans_vel_;
  bool sum_scores_;
  bool judge_by_maximum_;
  //footprint scaling with velocity;
  double max_scaling_factor_, scaling_speed_;
  boost::shared_ptr<base_local_planner::ObstacleCostFunction> obfun_;
};

} /* namespace base_local_planner */
#endif /* SITUATIONAL_OBSTACLE_COST_FUNCTION_H_ */
