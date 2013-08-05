/***************************************************************************
 *  abstract_search.h - Abstract search class for arbitrary search algorithm
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

#ifndef _PLUGINS_COLLI_SEARCH_ABSTRACT_SEARCH_H_
#define _PLUGINS_COLLI_SEARCH_ABSTRACT_SEARCH_H_

#include <logging/logger.h>

#include "og_laser.h"
#include <geometry/hom_point.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  /** @class ColliAbstractSearch <plugins/colli/search/abstract_search.h>
   * This is the abstract search interpretation class for
   * an arbitrary search algorithm to find its way through
   * an Occupancy grid from a robopos to a targetpos.
   *
   * This class tries to translate the found plan to interpreteable
   * things for the rest of the program.
   */
  class ColliAbstractSearch
  {

  public:

    ///
    ColliAbstractSearch(Logger* logger, ColliLaserOccupancyGrid * occGrid);

    ///
    virtual
    ~ColliAbstractSearch();

    /** update complete plan things
     *  precondition: the occupancy grid has to be updated previously!
     */
    virtual void
    Update(int roboX, int roboY, int targetX, int targetY) = 0;

    /** Returns after an update, if the update was successful.
     */
    virtual bool
    UpdatedSuccessful() = 0;

    /** return pointer to the local target. do not modify afterwards
     *  precondition: Update has to be called before this is ok here
     */
    const HomPoint&
    GetLocalTarget();

    /** return pointer to the local trajectory point. do not modify afterwards
     *  precondition: Update has to be called before this is ok here
     */
    const HomPoint&
    GetLocalTrajec();

    inline void
    update_occ(ColliLaserOccupancyGrid * occGrid);

  protected:

    // the occupancy grid
    ColliLaserOccupancyGrid * m_pOccGrid;

    // the calculated information where to drive to
    HomPoint m_LocalTarget, m_LocalTrajectory;
  };

  inline
  ColliAbstractSearch::ColliAbstractSearch(Logger* logger, ColliLaserOccupancyGrid * occGrid)
  {
    logger->log_info("ColliAbstractSearch", "ColliAbstractSearch(Constructor): Entering\n");
    m_pOccGrid = occGrid;
    logger->log_info("ColliAbstractSearch", "ColliAbstractSearch(Constructor): Exiting\n");
  }

  inline
  ColliAbstractSearch::~ColliAbstractSearch()
  {
  }

  inline void
  ColliAbstractSearch::update_occ(ColliLaserOccupancyGrid * occGrid)
  {
    m_pOccGrid = occGrid;
  }

  inline const HomPoint&
  ColliAbstractSearch::GetLocalTarget()
  {
    return m_LocalTarget;
  }

  inline const HomPoint&
  ColliAbstractSearch::GetLocalTrajec()
  {
    return m_LocalTrajectory;
  }

} // namespace fawkes

#endif
