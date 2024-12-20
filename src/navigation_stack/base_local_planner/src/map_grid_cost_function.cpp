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

#include <base_local_planner/map_grid_cost_function.h>

namespace base_local_planner {

MapGridCostFunction::MapGridCostFunction(costmap_2d::Costmap2D* costmap,
    double xshift,
    double yshift,
    bool is_local_goal_function,
    CostAggregationType aggregationType) :
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    aggregationType_(aggregationType),
    xshift_(xshift),
    yshift_(yshift),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(true) {
      ROS_INFO_STREAM("\t aggregation type from constructor " << aggregationType_);
    }


void MapGridCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}


bool MapGridCostFunction::prepare() {
  map_.resetPathDist();

  if (is_local_goal_function_) {
    map_.setLocalGoal(*costmap_, target_poses_);
  } else {
    map_.setTargetCells(*costmap_, target_poses_);
  }
  return true;
}



double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py) {
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}


double MapGridCostFunction::getCellCostsTraversability(unsigned int px, unsigned int py) {
  unsigned char val_costmap = costmap_->getCost(px, py);        
  double ddval_costmap_ = 0.0;

  if (val_costmap == FREE_SPACE_TRAVERSABILITY_NON_TRAVERSABLE ) {
    // ROS_INFO("val_costmap : NON_TRAVERSABLE"); 
    ddval_costmap_ = 250.0;
  }

  else if (val_costmap == FREE_SPACE_TRAVERSABILITY_GRASS ) {
    // ROS_INFO("val_costmap : GRASS"); 
    ddval_costmap_ = 0.0;
  }
  else if (val_costmap == FREE_SPACE_TRAVERSABILITY_ASPHALT ) {
    // ROS_INFO("val_costmap : ASPHALT"); 
    ddval_costmap_ = 0.0;
  }
  return ddval_costmap_;
}





double MapGridCostFunction::scoreTrajectory(Trajectory &traj) {
  // ROS_INFO_STREAM("my aggregation type " << aggregationType_);  // Stays with the one in the construxtor defintiion
  double cost = 0.0;
  if (aggregationType_ == Product) {
    cost = 1.0;
  }

  double px, py, pth;
  unsigned int cell_x, cell_y;
  double grid_dist;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);

    // translate point forward if specified
    if (xshift_ != 0.0) {
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }

    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);
      return -4.0;
    }


    /*original*/
    if (aggregationType_ != Traversability)
    {
      grid_dist = getCellCosts(cell_x, cell_y);     //<<< DING DING DING!!. Original. getCellCosts is only distance from each cell to goal
      // ROS_INFO("getCellCosts : %f", grid_dist ); 

      // if a point on this trajectory has no clear path to the goal... it may be invalid
      if (stop_on_failure_) {
        if (grid_dist == map_.obstacleCosts()) {
          return -3.0;
        } else if (grid_dist == map_.unreachableCellCosts()) {
          return -2.0;
        }
      }

    }


    /* dagr, altern 2 - add costmap value */
    // unsigned char val_costmap = costmap_->getCost(cell_x, cell_y);      //ideally, here is the traversability
    // // if ((val_costmap>=0)&&(val_costmap<=250))     {      ROS_INFO("Raw val map from >=0 to 250: %i", val_costmap);    } //gives ONLY many zeroes
    // // if ((val_costmap>0)&&(val_costmap<=250))     {      ROS_INFO("Raw val map from >0 to 250: %i", val_costmap);    } //does not show anything
    // if (val_costmap>0)     {      ROS_INFO("Raw val map >0: %i", val_costmap);    } //does not show anything


    else if (aggregationType_ == Traversability)
    {

      // unsigned char val_costmap = costmap_->getCost(cell_x, cell_y);      // traversability value from local costmap
      // // ROS_INFO("Raw val map : %d", val_costmap );

      // if (val_costmap == FREE_SPACE_TRAVERSABILITY_NON_TRAVERSABLE ) {
      //   ROS_INFO("val_costmap : NON_TRAVERSABLE"); 
      //   dval_costmap_ = 250.0;
      // }

      // else if (val_costmap == FREE_SPACE_TRAVERSABILITY_GRASS ) {
      //   ROS_INFO("val_costmap : GRASS"); 
      //   dval_costmap_ = 0.0;
      // }
      // else if (val_costmap == FREE_SPACE_TRAVERSABILITY_ASPHALT ) {
      //   ROS_INFO("val_costmap : ASPHALT"); 
      //   dval_costmap_ = 0.0;
      // }

      // // grid_dist = getCellCosts(cell_x, cell_y);     // distance to goal
      // // // ROS_INFO("getCellCosts : %f", grid_dist ); 

      // // //if a point on this trajectory has no clear path to the goal... it may be invalid
      // // if (stop_on_failure_) {
      // //   if (grid_dist == map_.obstacleCosts()) {
      // //     // ROS_INFO("0000  obstacleCosts!!!!"); 
      // //     return -3.0;
      // //   } else if (grid_dist == map_.unreachableCellCosts()) {
      // //     // ROS_INFO("2222  obstacleCosts!!!!"); 
      // //     return -2.0;
      // //   }
      // // }


      // dval_costmap_ = getCellCostsTraversability(cell_x, cell_y);
      // grid_dist = dval_costmap_*0.1;      // check FACTOR (.01)
      // grid_dist = dval_costmap_;      // check FACTOR (.01)

      grid_dist = getCellCostsTraversability(cell_x, cell_y);
      // ROS_INFO("  Traversability cell cost : %f", grid_dist ); 
    }



    switch( aggregationType_ ) {
    case Last:
      // ROS_INFO("\tLAST");
      cost = grid_dist;
      break;
    case Sum:
      // ROS_INFO("\tSUM");
      cost += grid_dist;
      break;
    case Product:
      // ROS_INFO("\tPRODUCT");
      if (cost > 0) {
        cost *= grid_dist;
      }
      break;
    case Traversability:        //same than "Last", but with trav values
      // ROS_INFO("\tTRAV");
        cost = grid_dist;
      break;
    }
  }
  return cost;
}






} /* namespace base_local_planner */
