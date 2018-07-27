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
#ifndef SITUATIONAL_MAP_CELL_H_
#define SITUATIONAL_MAP_CELL_H_

#include <base_local_planner/trajectory_inc.h>
#include <costmap_2d/cost_values.h>
namespace base_local_planner {
  /**
   * @class MapCell
   * @brief Stores path distance and goal distance information used for scoring trajectories
   */
  class SituationalMapCell{
    public:
      /**
       * @brief  Default constructor
       */
      SituationalMapCell();

      /**
       * @brief  Copy constructor
       * @param smc The SituationalMapCell to be copied
       */
      SituationalMapCell(const SituationalMapCell& smc);

      unsigned char getCellCost()
      {
        return obstacle_cost_;
      }

      unsigned int cx, cy; ///< @brief Cell index in the grid map

      enum dz_status
      {
        NORMAL,
        CAUTIOUS,
        UNKNOWN
      }defined_zone_status;
      unsigned char obstacle_cost_;
  };
};

#endif
