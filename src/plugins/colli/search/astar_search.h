
/***************************************************************************
 *  astar_search.h - Interpretation class interface for A* for Colli-A*
 *
 *  Created: Sat Jul 13 18:06:21 2013
 *  Copyright  2002  Stefan Jacobs
 *             2012  Safoura Rezapour Lakani
 *             2013  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef _PLUGINS_COLLI_SEARCH_ASTAR_SEARCH_H_
#define _PLUGINS_COLLI_SEARCH_ASTAR_SEARCH_H_

#include "abstract_search.h"
#include "astar.h"

#ifdef _COLLI_VISUALIZE_
#include "vis/visualize.h"
#endif

#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class Configuration;

/** @class ColliSearch <plugins/colli/search/astar_search.h>
 * This is the plan class. It tries to translate the found plan to interpreteable
 * things for the rest of the program.
 *
 * Here the plan from A* is managed and cut into small pieces.
 * Also usable methods for managing the plan are implemented here.
 */
class ColliSearch: public ColliAbstractSearch
{

public:

  //
  ColliSearch( Logger* logger, Configuration *config, ColliLaserOccupancyGrid * occGrid );

  //
  virtual ~ColliSearch();


  // update complete plan things
  // precondition: the occupancy grid has to be updated previously!
  void Update( int roboX, int roboY, int targetX, int targetY  );

  // returns, if the update was successful or not.
  // precondition: update had to be called.
  bool UpdatedSuccessful();

  // ** getter for visualization ** //
  std::vector< HomPoint > GetPlan();


  ColliLaserOccupancyGrid* get_grid()
  {
    return m_pOccGrid;
  }

  OccupancyGrid * get_astar_grid()
  {
    return m_pAStar->get_occ_grid();
  }

  std::vector<HomPoint > get_occ_astar_search()
  {
    return m_pAStar->get_occ_astar();
  }

  std::vector<HomPoint > get_astar_states()
  {
    return m_pAStar->get_seen_states();
  }

  HomPoint get_mod_target()
  {
    return m_TargetPosition;
  }

  HomPoint get_adjust_robo();
private:

  /** Returns the current, modified waypoint to drive to.
   */
  HomPoint CalculateLocalTarget();

  /** Adjust the waypoint if it is not the final point.
   */
  HomPoint AdjustWaypoint( const HomPoint &local_target );

  /** Returns the current trajectory point to drive to.
   */
  HomPoint CalculateLocalTrajectoryPoint( );

  /** Method for checking if an obstacle is between two points.
   */
  bool IsObstacleBetween( const HomPoint &a, const HomPoint &b,
        const int maxcount );


  // --------------------------------- //
  //    VARIABLES
  // --------------------------------- //

  ColliAStar * m_pAStar;                // the A* search algorithm
  std::vector< HomPoint > m_vPlan;    // the local representation of the plan

  HomPoint m_RoboPosition, m_TargetPosition;
  bool m_UpdatedSuccessful;
  int robo_widthX,robo_widthY;
  bool adjust_robopos;
  // SJ TODO: Delete paint stuff
#ifdef _COLLI_VISUALIZE_
  CVisualize * m_pVis;
#endif

  int m_RobocupMode;

  Logger* loggerAstar;


};


} // namespace fawkes

#endif
