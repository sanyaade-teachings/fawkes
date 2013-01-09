//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$             */
/*                                                                      */
/* Description: This is the interpretation class interface for A* of    */
/*              Colli-A*                                                */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This class tries to translate the found plan to interpreteable */
/*       things for the rest of the program.                            */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_ASTARSEARCH_H_
#define _COLLI_ASTARSEARCH_H_



//#include <utils/geometry/point.h>
#include <utils/math/types.h>
#include "abstract_search.h"
#include "astar.h"

#include <logging/logger.h>
#include <config/config.h>
#ifdef _COLLI_VISUALIZE_
#include "vis/visualize.h"
#endif


/** This is the plan class.
 *  Here the plan from A* is managed and cut into small pieces.
 *    Also usable methods for managing the plan are implemented here.
 */
class CSearch: public CAbstractSearch
{
  
public:
  
  //
  CSearch( Logger* logger, Configuration *config, CLaserOccupancyGrid * occGrid );

  //    
  virtual ~CSearch();


  // update complete plan things
  // precondition: the occupancy grid has to be updated previously!
  void Update( int roboX, int roboY, int targetX, int targetY  );

  // returns, if the update was successful or not.
  // precondition: update had to be called.
  bool UpdatedSuccessful();

  // ** getter for visualization ** //
  std::vector< HomPoint > GetPlan();
  
  
  CLaserOccupancyGrid* get_grid()
  {
    return m_pOccGrid;
  }

  OccupancyGrid * get_astar_grid()
  { 
    return m_pAStar->get_occ_grid();
  }
  
  vector<HomPoint > get_occ_astar_search()
  {
    return m_pAStar->get_occ_astar();
  }

  vector<HomPoint > get_astar_states()
  {
    return m_pAStar->get_seen_states();
  }

  HomPoint get_mod_target()
  {
    return m_TargetPosition;
  }
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

  CAStar * m_pAStar;                // the A* search algorithm
  std::vector< HomPoint > m_vPlan;    // the local representation of the plan

  //Point m_RoboPosition, m_TargetPosition;
  HomPoint m_RoboPosition, m_TargetPosition;
  bool m_UpdatedSuccessful;

  
  // SJ TODO: Delete paint stuff 
#ifdef _COLLI_VISUALIZE_
  CVisualize * m_pVis;
#endif

  int m_RobocupMode;

  Logger* loggerAstar;


};


#endif
