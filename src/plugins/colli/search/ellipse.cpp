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
/* $Id$              */
/*                                                                      */
/* Description: This is the colli implementation of a fast ellipse      */
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

#include "ellipse.h"
#include "../common/defines.h"

#include <vector>
#include <iostream>

/** construct a new ellipse with given values. */
CFastEllipse::CFastEllipse( int radius_width, int radius_height, int robocup_mode )
{
  float dist = 1000.0;
  float dist_near = 1000.0;
  float dist_middle = 1000.0;
  float dist_far = 1000.0;

  int maxRad = std::max( radius_width, radius_height );

  for ( int y = -(maxRad+6); y <= (maxRad+6); y++ )
  {
    for ( int x = -(maxRad+6); x <= (maxRad+6); x++ )
      {
        dist = sqr(((float)x/(float)radius_width)) + sqr(((float)y/(float)radius_height));
        dist_near = sqr(((float)x/(float)(radius_width+2))) + sqr(((float)y/(float)(radius_height+2)));
        dist_middle = sqr(((float)x/(float)(radius_width+4))) + sqr(((float)y/(float)(radius_height+4)));
/*      if ( robocup_mode == 1 ) */
/*        { */
/*          ; // ignore far distance obstacles */
/*        } */
/*      else */
/*        { */
            dist_far = sqr((float)x/(float)(radius_width+6))+sqr((float)y/(float)(radius_height+6));
//        }

        if ( (dist > 1.0) && (dist_near > 1.0) &&
             (dist_middle > 1.0) && (dist_far > 1.0) )
          {
            ; // not in grid!
          }
        else if ( (dist > 1.0) && (dist_near > 1.0) &&
                  (dist_middle > 1.0) && (dist_far <= 1.0) )
          {
            m_OccupiedCells.push_back( x );
            m_OccupiedCells.push_back( y );
            m_OccupiedCells.push_back( (int)_COLLI_CELL_FAR_ );
          }
        else if ( (dist > 1.0) && (dist_near > 1.0) &&
                  (dist_middle <= 1.0) )
          {
            m_OccupiedCells.push_back( x );
            m_OccupiedCells.push_back( y );
            m_OccupiedCells.push_back( (int)_COLLI_CELL_MIDDLE_ );
          }
        else if ( (dist > 1.0) && (dist_near <= 1.0) &&
                  (dist_middle <= 1.0) )
          {
            m_OccupiedCells.push_back( x );
            m_OccupiedCells.push_back( y );
            m_OccupiedCells.push_back( (int)_COLLI_CELL_NEAR_ );
          }
        else if ( (dist <= 1.0) && (dist_near <= 1.0) &&
                  (dist_middle <= 1.0) )
          {
            m_OccupiedCells.push_back( x );
            m_OccupiedCells.push_back( y );
            m_OccupiedCells.push_back( (int)_COLLI_CELL_OCCUPIED_ );
          }
      }
  }
}

CFastEllipse::~CFastEllipse()
{
  m_OccupiedCells.clear();
}