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
 *********************************************************************/s
#include <base_local_planner/situational_map_grid.h>
#include <costmap_2d/cost_values.h>
using namespace std;

namespace base_local_planner{

  SituationalMapGrid::SituationalMapGrid()
    : size_x_(0), size_y_(0)
  {
  }

  SituationalMapGrid::SituationalMapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  SituationalMapGrid::SituationalMapGrid(const SituationalMapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void SituationalMapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t SituationalMapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  SituationalMapCell SituationalMapGrid::getMapCell(unsigned int cell_x, unsigned int cell_y) const
  {
    ROS_ASSERT((size_x_ * cell_y + cell_x) < size_x_ * size_y_ );
    return map_[getIndex(cell_x, cell_y)];
  }

  SituationalMapGrid& SituationalMapGrid::operator= (const SituationalMapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  void SituationalMapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }

  bool SituationalMapGrid::resolutionCheck(const costmap_2d::Costmap2D& global_costmap, const costmap_2d::Costmap2D& local_costmap)
  {
    if(local_costmap.getResolution() != global_costmap.getResolution())
    {
      ROS_ERROR("SituationalMapGrid: Resolution of local costmap is higher than global costmap resolution!");
      return false;
    }
    // to do adjust resolutions
    return true;
  }

  void SituationalMapGrid::updateGrid(const costmap_2d::Costmap2D& global_costmap, const costmap_2d::Costmap2D& local_costmap)
  { 
    double wx, wy, mx, my;
    unsigned int index;
    if (!resolutionCheck(global_costmap, local_costmap))
      return;
    sizeCheck(local_costmap.getSizeInCellsX(), local_costmap.getSizeInCellsY());
    for(unsigned int i = 0; i < local_costmap.getSizeInCellsY(); ++i){
      for(unsigned int j = 0; j < local_costmap.getSizeInCellsX(); ++j)
      {
        index = global_costmap.getSizeInCellsX() * i + j;
        
        map_[index].obstacle_cost_ = local_costmap.getCost(i, j);
        local_costmap.mapToWorld(i, j, wx, wy);
        // to check the size bounds?
        global_costmap.worldToMap(wx, wy, mx, my);  
        if((global_costmap.getCost(mx,my) <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) && (global_costmap.getCost(mx,my) >= costmap_2d::FREE_SPACE))
        {
          map_[index].defined_zone_status = CAUTIOUS;
        }
        else if(global_costmap.getCost(mx,my) == costmap_2d::NO_INFORMATION)
        {
          map_[index].defined_zone_status = UNKNOWN;
        }
        else
        {
          map_[index].defined_zone_status = NORMAL;
        }
      }
    }
  }


};
