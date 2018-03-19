/*********************************************************************
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

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

/* ouiyeah @ 2017-04-19
Add bool param to reuse scoreTrajectory with or without obstacle_costs.
//*/
#ifdef IGNORE_OUIYEAH
  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
#else
      //TODO: combine scoreTrajectory and scoreTrajectory2
  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
      bool obstacle_costs_enabled = false;
      return scoreTrajectory2(traj, best_traj_cost, obstacle_costs_enabled);
  }

  double SimpleScoredSamplingPlanner::scoreTrajectory2(Trajectory& traj, double best_traj_cost, bool& obstacle_costs_enabled) {
    bool obstacle_costs_masked = obstacle_costs_enabled;
#endif

    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;

/* ouiyeah @ 2017-04-21
Add bool param to reuse scoreTrajectory with or without costs_function.
//*/
#ifdef IGNORE_OUIYEAH
      if (score_function_p->getScale() == 0) {
        continue;
      }

      double cost = score_function_p->scoreTrajectory(traj);
#else
      if (score_function_p->getScale() == 0) {
          if (gen_id != 1) continue;
      }

      if (gen_id > 1)
      {
          MapGridCostFunction* score_function_p_map = (MapGridCostFunction*)score_function_p;
          if (obstacle_costs_masked)
          {
              score_function_p_map->setCostMask(true);
          }
          else
          {
              score_function_p_map->setCostMask(false);
          }
      }

      double cost = score_function_p->scoreTrajectory(traj);

      if (obstacle_costs_enabled)
      {
          if (gen_id == 1) //obstacle_costs_function
          {
              if (cost < 0)
              {
                  obstacle_costs_enabled = false;
              }
              cost = 0;
          }
      }
#endif

      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }

#ifdef IGNORE_OUIYEAH
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
#else
      if (cost != 0) {
          if (score_function_p->getScale() == 0) {
              cost *= 0.01;
          } else {
              cost *= score_function_p->getScale();
          }
      }
#endif

      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }

  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;

/* ouiyeah @ 2017-04-19
Add bool param to reuse scoreTrajectory with or without obstacle_costs.
//*/
#ifdef IGNORE_OUIYEAH
#else
    std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin() + 1;
    TrajectoryCostFunction* score_function_p = *score_function;
    bool best_traj_enabled = true;
#endif

    bool gen_success;
    int count, count_valid;
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories()) {
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) {
          // TODO use this for debugging
          continue;
        }

/* ouiyeah @ 2017-04-19
Add bool param to reuse scoreTrajectory with or without obstacle_costs.
//*/
#ifdef IGNORE_OUIYEAH
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
#else
        bool obstacle_costs_enabled = true;
        if (score_function_p->getScale() > 0) {
            loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
        } else {
            loop_traj_cost = scoreTrajectory2(loop_traj, best_traj_cost, obstacle_costs_enabled);
        }
#endif

        if (all_explored != NULL) {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }

        if (loop_traj_cost >= 0) {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;

/* ouiyeah @ 2017-04-19
Add bool param to reuse scoreTrajectory with or without obstacle_costs.
//*/
#ifdef IGNORE_OUIYEAH
#else
            best_traj_enabled = obstacle_costs_enabled;
#endif

          }
        }
        count++;
        if (max_samples_ > 0 && count >= max_samples_) {
          break;
        }        
      }
      if (best_traj_cost >= 0) {

/* ouiyeah @ 2017-04-20
Add bool param to reuse scoreTrajectory with or without obstacle_costs.
//*/
#ifdef IGNORE_OUIYEAH
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
#else
        if (best_traj_enabled)
        {
            traj.xv_ = best_traj.xv_;
            traj.yv_ = best_traj.yv_;
            traj.thetav_ = best_traj.thetav_;
            traj.cost_ = best_traj_cost;
            traj.resetPoints();
            double px, py, pth;
            for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
                best_traj.getPoint(i, px, py, pth);
                traj.addPoint(px, py, pth);
            }
        }
        else
        {
            traj.xv_ = 0;
            traj.yv_ = 0;
            traj.thetav_ = 0;
            traj.cost_ = best_traj_cost;
            traj.resetPoints();
            double px, py, pth, scale;
            for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
                best_traj.getPoint(i, px, py, pth);
                traj.addPoint(px, py, pth);

                double cost = score_function_p->scoreTrajectory(traj);
                if (cost < 0)
                {
                    scale = (double)i / best_traj.getPointsSize();
                    if (best_traj.xv_ * scale < 0.05) {
                        scale = 0;
                    }
                    break;
                }
            }
            traj.xv_ = best_traj.xv_ * scale;
            traj.yv_ = best_traj.yv_ * scale;
            traj.thetav_ = best_traj.thetav_ * scale;
        }
#endif

      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

  
}// namespace
