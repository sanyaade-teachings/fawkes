/***************************************************************************
 *  astar.h - AStar-interface for A* of Colli-A*
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

#ifndef _PLUGINS_COLLI_SEARCH_ASTAR_H_
#define _PLUGINS_COLLI_SEARCH_ASTAR_H_

#include "astar_state.h"
#include "../robo-utils/occupancygrid/occupancygrid.h"

#include <geometry/hom_point.h>

#include <vector>
#include <queue>
#include <map>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  class Logger;
  class Configuration;

  /** Class ColliAStar.
   *  This is an implementation of the A* search algorithm in a
   *    highly efficient way (I hope ;-).
   */
  class ColliAStar
  {
  public:

    /** Constructor.
     *  This is the constructor for the AStar Object.
     *  @param occGrid is a pointer to an COccupancyGrid to search through.
     *  @param dbg is a pointer to the debug object.
     */
    ColliAStar(Logger* logger, Configuration *config, OccupancyGrid * occGrid);

    /** Destructor.
     *  This destructs the AStarObject.
     */
    ~ColliAStar();

    /* =========================================== */
    /* ************* PUBLIC METHODS ************** */
    /* =========================================== */

    /** Solves the given assignment.
     *  This starts the search for a path through the occupance grid to the
     *    target point.
     *  Performing astar search over the occupancy grid and returning the solution.
     */
    void
    Solve(const HomPoint &RoboPos, const HomPoint &TargetPos, std::vector<HomPoint> &solution);

    /** Method, returning the nearest point outside of an obstacle.
     *  @return a new modified point.
     */
    HomPoint
    RemoveTargetFromObstacle(int targetX, int targetY, int stepX, int stepY);

    OccupancyGrid *
    get_occ_grid()
    {
      return m_pOccGrid;
    }

    std::vector<HomPoint>
    get_occ_astar()
    {
      return occ_cells;
    }
    std::vector<HomPoint>
    get_seen_states()
    {
      return seen_states;
    }
    void
    get_grid();
  private:

    std::vector<HomPoint> occ_cells;
    std::vector<HomPoint> seen_states;
    /* =========================================== */
    /* ************ PRIVATE VARIABLES ************ */
    /* =========================================== */

    // this is the local reference to the occupancy grid.
    OccupancyGrid * m_pOccGrid;
    unsigned int m_Width;
    unsigned int m_Height;

    // this is the local robot position and target point.
    ColliAStarState m_pRoboPos;
    ColliAStarState m_pTargetState;

    // This is a state vector...
    // It is for speed purposes. So I do not have to do a new each time
    //   I have to malloc a new one each time.
    std::vector<ColliAStarState *> m_vAStarStates;

    // maximum number of states available for a* and current index
    int m_MaxStates;
    int m_AStarStateCount;

    // this is AStars openlist
    struct cmp
    {
      bool
      operator()(ColliAStarState * a1, ColliAStarState * a2) const
      {
        return (a1->m_TotalCost > a2->m_TotalCost);
      }
    };
    std::priority_queue<ColliAStarState *, std::vector<ColliAStarState *>, cmp> m_pOpenList;

    // this is AStars closedList
    std::map<int, int> m_hClosedList;

    /* =========================================== */
    /* ************ PRIVATE METHODS ************** */
    /* =========================================== */

    // Search with AStar through the OccGrid
    ColliAStarState *
    Search();

    // Calculate a unique key for a given coordinate
    int
    CalculateKey(int x, int y);

    // Check if the state is a goal
    bool
    IsGoal(ColliAStarState * state);

    // Calculate heuristic for a given state
    int
    Heuristic(ColliAStarState * state);

    // Generate all children for a given State
    void
    GenerateChildren(ColliAStarState * father);

    // Generates a solution sequence for a given state
    void
    GetSolutionSequence(ColliAStarState * node, std::vector<HomPoint> &solution);

    Logger* loggerASS;

  };

} // namespace fawkes

#endif
