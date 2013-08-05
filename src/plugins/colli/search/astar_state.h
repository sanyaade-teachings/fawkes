/***************************************************************************
 *  abstract_state.h - Class for an A*-search state
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

#ifndef _PLUGINS_COLLI_SEARCH_ASTAR_STATE_H_
#define _PLUGINS_COLLI_SEARCH_ASTAR_STATE_H_

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  /** AStarState.
   *  This is the class for an A* State.
   */
  class ColliAStarState
  {
  public:

    /**  This is the standard constructor.
     */
    ColliAStarState();

    /**  This is another standard constuctor, this time parametrized.
     *   @param x is the x coordinate.
     *   @param y is the y coordinate.
     *   @param pastCost is the total left cost.
     *   @param father is a pointer to the predecessor of this
     *          AStarState.
     */
    ColliAStarState(int x, int y, int pastCost, ColliAStarState * father);

    /// Destructor.
    ~ColliAStarState();

    // Coordinates
    int m_X, m_Y;

    // Predecessor
    ColliAStarState * m_pFather;

    // Costs
    int m_PastCost, m_TotalCost;
  };

  /* ************************************************************************** */
  /* ***********************  IMPLEMENTATION DETAILS  ************************* */
  /* ************************************************************************** */

// Standard Constructor
  inline
  ColliAStarState::ColliAStarState()
  {
    m_pFather = 0;
    m_X = m_Y = 0;
    m_TotalCost = 0;
    m_PastCost = 0;
  }

// Another Constructor
  inline
  ColliAStarState::ColliAStarState(int x, int y, int pastCost, ColliAStarState * father)
  {
    m_X = x;
    m_Y = y;
    m_PastCost = pastCost;
    m_pFather = father;
  }

// Standard Destructor
  inline
  ColliAStarState::~ColliAStarState()
  {
  }

} // namespace fawkes

#endif
