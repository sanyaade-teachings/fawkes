
/***************************************************************************
 *  viaualization_thread_base.h - Visualization Thread Base
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013  AllemaniACs
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_COLLI_VISUALIZATION_THREAD_BASE_H_
#define __PLUGINS_COLLI_VISUALIZATION_THREAD_BASE_H_


#ifndef HAVE_VISUAL_DEBUGGING
#  error ColliVisualizationThread was disabled by build flags
#endif

#include <utils/time/time.h>
//#include <Eigen/Core>
//#include <Eigen/StdVector>
#include <aspect/tf.h>
#include <geometry/hom_point.h>
#include <string.h>
#include<string>
#include <vector>

using namespace fawkes;
using namespace std;

class ColliVisualizationThreadBase
{
 public:

  virtual ~ColliVisualizationThreadBase();

  virtual void visualize(const std::string frame_id,vector<HomPoint> cells,vector<HomPoint> near_cells,vector<HomPoint> far_cells,vector<HomPoint> middle_cells,
                         HomPoint m_RoboGridPos,HomPoint m_LaserGridPos, vector<HomPoint> laser_cells,
                         vector< HomPoint > plan,HomPoint motor_des,int cell_width, int cell_height,HomPoint target,
                         int grid_width, int grid_height, HomPoint motor_real,HomPoint localTarget,HomPoint target_odom,
                         vector<HomPoint > orig_laser_points,vector<HomPoint > search_occ,vector<HomPoint > astar_found_occ,
                         vector<HomPoint > free_cells,vector<HomPoint > seen_states,HomPoint modTarget) throw()=0;

};

#endif
