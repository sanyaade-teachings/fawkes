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
/* $Id$                */
/*                                                                      */
/* Description: This is the AStar-interface for A* of Colli-A*          */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */

#ifndef _COLLI_ASTAR_H_
#define _COLLI_ASTAR_H_


#include <vector>
#include <queue>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "astar_state.h"
#include "../robo-utils/occupancygrid/occupancygrid.h"

#include <utils/math/types.h>
#include <geometry/hom_point.h>
#include <logging/logger.h>
#include <config/config.h>

using namespace fawkes;
using namespace std;


/** Class AStar.
 *  This is an implementation of the A* search algorithm in a 
 *    highly efficient way (I hope ;-).
 */
class CAStar
{
public:

  /** Constructor.
   *  This is the constructor for the AStar Object.
   *  @param occGrid is a pointer to an COccupancyGrid to search through.
   *  @param dbg is a pointer to the debug object.
   */
  CAStar( Logger* logger, Configuration *config, OccupancyGrid * occGrid );


  /** Destructor.
   *  This destructs the AStarObject.
   */
  ~CAStar();


  /* =========================================== */
  /* ************* PUBLIC METHODS ************** */
  /* =========================================== */


  /** Solves the given assignment.
   *  This starts the search for a path through the occupance grid to the
   *    target point.
   *  Performing astar search over the occupancy grid and returning the solution.
   */
  void Solve( const HomPoint &RoboPos, const HomPoint &TargetPos, 
	      std::vector< HomPoint > &solution );


  /** Method, returning the nearest point outside of an obstacle.
   *  @return a new modified point.
   */
  HomPoint RemoveTargetFromObstacle( int targetX, int targetY, int stepX, int stepY);
  
  OccupancyGrid * get_occ_grid()
  {
    return m_pOccGrid;
  }

  vector<HomPoint > get_occ_astar()
  {
    return occ_cells;
  }
  vector<HomPoint > get_seen_states()
  {
    return seen_states;
  }
  void get_grid();
private:

  vector<HomPoint > occ_cells;
  vector<HomPoint > seen_states;
  /* =========================================== */
  /* ************ PRIVATE VARIABLES ************ */
  /* =========================================== */

  // this is the local reference to the occupancy grid.
  OccupancyGrid * m_pOccGrid;
  unsigned int m_Width;
  unsigned int m_Height;

  // this is the local robot position and target point.
  CAStarState m_pRoboPos;
  CAStarState m_pTargetState;

  // This is a state vector... 
  // It is for speed purposes. So I do not have to do a new each time
  //   I have to malloc a new one each time.
  std::vector< CAStarState * > m_vAStarStates;

  // maximum number of states available for a* and current index
  int m_MaxStates;
  int m_AStarStateCount;

  // this is AStars openlist
  struct cmp
  {
    bool operator() ( CAStarState * a1, CAStarState * a2 ) const
    {
      return (a1->m_TotalCost > a2->m_TotalCost);
    }
  };
  std::priority_queue< CAStarState *, std::vector< CAStarState * >, cmp > m_pOpenList;
  
  // this is AStars closedList
  std::map< int, int > m_hClosedList;

  /* =========================================== */
  /* ************ PRIVATE METHODS ************** */
  /* =========================================== */

  // Search with AStar through the OccGrid
  CAStarState * Search();

  // Calculate a unique key for a given coordinate
  int CalculateKey( int x, int y );

  // Check if the state is a goal
  bool IsGoal( CAStarState * state );

  // Calculate heuristic for a given state
  int Heuristic( CAStarState * state );

  // Generate all children for a given State
  void GenerateChildren( CAStarState * father );

  // Generates a solution sequence for a given state
  void GetSolutionSequence( CAStarState * node, std::vector< HomPoint > &solution );

  Logger* loggerASS;

};

#endif
