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
#ifndef SITUATIONAL_MAP_GRID_H_
#define SITUATIONAL_ROLLOUT_MAP_GRID_H_

#include <vector>
#include <iostream>
#include <base_local_planner/trajectory_inc.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <base_local_planner/situational_map_cell.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner{
  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   */
  class SituationalMapGrid{
    public:
      /**
       * @brief  Creates a 0x0 map by default
       */
      SituationalMapGrid();

      /**
       * @brief  Creates a map of size_x by size_y
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       */
      SituationalMapGrid(unsigned int size_x, unsigned int size_y);


      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A reference to the desired cell
       */
      inline SituationalMapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A copy of the desired cell
       */
      inline SituationalMapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }

      inline SituationalMapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Destructor for a MapGrid
       */
      ~SituationalMapGrid(){}

      /**
       * @brief  Copy constructor for a MapGrid
       * @param mg The MapGrid to copy 
       */
      SituationalMapGrid(const SituationalMapGrid& mg);

      /**
       * @brief  Assignment operator for a MapGrid
       * @param mg The MapGrid to assign from 
       */
      SituationalMapGrid& operator= (const SituationalMapGrid& mg);


      /**
       * @brief  check if we need to resize
       * @param size_x The desired width
       * @param size_y The desired height
       */
      void sizeCheck(unsigned int size_x, unsigned int size_y);

      /**
       * @brief  check if we need to adjust the resolutions of global and local costmaps
       */    
      bool resolutionCheck(const costmap_2d::Costmap2D& global_costmap, const costmap_2d::Costmap2D& local_costmap);

      /**
       * @brief Utility to share initialization code across constructors
       */
      void commonInit();

      /**
       * @brief  Returns a 1D index into the MapCell array for a 2D index
       * @param x The desired x coordinate
       * @param y The desired y coordinate
       * @return The associated 1D index 
       */
      size_t getIndex(int x, int y);

      /**
       * return a value that indicates cell is in obstacle
       */
      inline double obstacleCosts() {
        return map_.size();
      }

      SituationalMapCell getMapCell(unsigned int cell_x, unsigned int cell_y) const;

      /**
       * returns a value indicating cell was not reached by wavefront
       * propagation of set cells. (is behind walls, regarding the region covered by grid)
       */
      inline double unreachableCellCosts() {
        return map_.size() + 1;
      }

      void updateGrid(const costmap_2d::Costmap2D& global_costmap, const costmap_2d::Costmap2D& local_costmap);

      /**
       * increase global plan resolution to match that of the costmap by adding points linearly between global plan points
       * This is necessary where global planners produce plans with few points.
       * @param global_plan_in input
       * @param global_plan_output output
       * @param resolution desired distance between waypoints
       */
      static void adjustMapCellResolution(const costmap_2d::Costmap2D& local_costmap,
            const costmap_2d::Costmap2D& global_costmap);

      unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid

    private:

      std::vector<SituationalMapCell> map_; ///< @brief Storage for the MapCells

  };
};

#endif
