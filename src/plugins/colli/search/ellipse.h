
/***************************************************************************
 *  ellipse.h - A colli implementation of a fast ellipse
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

#ifndef _PLUGINS_COLLI_SEARCH_ELLIPSE_H_
#define _PLUGINS_COLLI_SEARCH_ELLIPSE_H_

#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColliFastEllipse
{
 public:

  // center 0, 0
  // construct a new ellipse with given values
  ColliFastEllipse( int radius_width, int radius_height,
                int robocup_mode );
  ~ColliFastEllipse();


  // Return the occupied cells with their values
  inline const std::vector< int > GetEllipse()
  {
    return m_OccupiedCells;
  }


  inline int GetKey()
  {
    return m_Key;
  }

  inline void SetKey( int key )
  {
    m_Key = key;
  }


 private:

  // the occ cells, size is dividable through 3, 'cause:
  // [i]   = x coord,
  // [i+1] = y coord,
  // [i+2] = costs
  std::vector< int > m_OccupiedCells;


  // a unique identifier for each ellipse
  int m_Key;

  inline float sqr( float x )
  {
    return (x*x);
  }

};

} // end of namespace fawkes

#endif