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
/* $Id$          */
/*                                                                      */
/* Description: This is the abstract search interpretation class for    */
/*              an arbitrary search algorithm to find its way through   */
/*              an Occupancy grid from a robopos to a targetpos.        */
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


#ifndef _COLLI_ABSTRACTSEARCH_H_
#define _COLLI_ABSTRACTSEARCH_H_

#include <logging/logger.h>

//#include <utils/geometry/point.h>
#include "og_laser.h"
#include <utils/math/types.h>
#include <geometry/hom_point.h>

using namespace fawkes;
using namespace std;

/** This is the basic plan class.
 */
class CAbstractSearch
{
  
public:
  
  ///
  CAbstractSearch( Logger* logger, CLaserOccupancyGrid * occGrid );

  ///
  virtual ~CAbstractSearch();


  /** update complete plan things
   *  precondition: the occupancy grid has to be updated previously!
   */
  virtual void Update( int roboX, int roboY, int targetX, int targetY) = 0;


  /** Returns after an update, if the update was successful.
   */
  virtual bool UpdatedSuccessful() = 0;


  /** return pointer to the local target. do not modify afterwards 
   *  precondition: Update has to be called before this is ok here
   */
  //const Point& GetLocalTarget();
  const HomPoint& GetLocalTarget();

  /** return pointer to the local trajectory point. do not modify afterwards
   *  precondition: Update has to be called before this is ok here
   */
  //const Point& GetLocalTrajec();
  const HomPoint& GetLocalTrajec();

  inline void update_occ(CLaserOccupancyGrid * occGrid);

protected:

  // the occupancy grid
  CLaserOccupancyGrid * m_pOccGrid;

  // the calculated information where to drive to
  //Point m_LocalTarget, m_LocalTrajectory;
  HomPoint m_LocalTarget, m_LocalTrajectory;
};




inline CAbstractSearch::CAbstractSearch( Logger* logger, CLaserOccupancyGrid * occGrid )
{
  //BB_DBG(4) << "CAbstractSearch(Constructor): Entering" << std::endl;
  logger->log_info("CAbstractSearch","CAbstractSearch(Constructor): Entering\n");
  m_pOccGrid = occGrid;
  //BB_DBG(4) << "CAbstractSearch(Constructor): Exiting" << std::endl;
  logger->log_info("CAbstractSearch","CAbstractSearch(Constructor): Exiting\n");
}


inline CAbstractSearch::~CAbstractSearch()
{
}


inline void CAbstractSearch::update_occ(CLaserOccupancyGrid * occGrid)
{
  m_pOccGrid = occGrid;
}

inline const HomPoint& CAbstractSearch::GetLocalTarget()
{
  return m_LocalTarget;
}


inline const HomPoint& CAbstractSearch::GetLocalTrajec()
{
  return m_LocalTrajectory;
}



#endif
