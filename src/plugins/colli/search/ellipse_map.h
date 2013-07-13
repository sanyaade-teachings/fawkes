
/***************************************************************************
 *  ellipse_map.h - A colli implementation of a collection offast ellipse
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

#ifndef _PLUGINS_COLLI_SEARCH_ELLIPSE_MAP_H_
#define _PLUGINS_COLLI_SEARCH_ELLIPSE_MAP_H_

#include "ellipse.h"

#include <vector>
#include <map>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColliEllipseMap
{
 public:

  //  ColliEllipseMap( int max_radius_width, int max_radius_height, int robocup_mode );

  ColliEllipseMap( );

  ~ColliEllipseMap() { m_mEllipses.clear(); }


  const std::vector< int > GetEllipse( int width, int height, int robocup_mode );


 private:

  std::map< unsigned int, ColliFastEllipse * > m_mEllipses;

};



//inline ColliEllipseMap::ColliEllipseMap( int max_radius_width, int max_radius_height,
//         int robocup_mode )

inline ColliEllipseMap::ColliEllipseMap()
{
//   for ( unsigned int x = 0; x < (unsigned int)max_radius_width; x++ )
//     {
//       for ( unsigned int y = 0; y < (unsigned int)max_radius_height; y++ )
//  {
//    ColliFastEllipse * ellipse = new ColliFastEllipse( x, y, robocup_mode );

//    // What it does is the following: x * 2^16 + y. This is unique,
//    //   because first it does a bit shift for 16 bits, and adds (or)
//    //   afterwards a number that is smaller tham 16 bits!
//    unsigned int key = (x << 16) | y;
//    ellipse->SetKey( key );
//    m_mEllipses[ key ] = ellipse;
//  }
//     }
}


inline const std::vector< int > ColliEllipseMap::GetEllipse( int width, int height, int robocup_mode )
{
  unsigned int key = ((unsigned int)width << 16) | (unsigned int)height;

  std::map< unsigned int, ColliFastEllipse * >::iterator p = m_mEllipses.find( key );
  if ( p == m_mEllipses.end() ) // ellipse nicht gefunden!
    {
      ColliFastEllipse * ellipse = new ColliFastEllipse( width, height, robocup_mode );
      ellipse->SetKey( key );
      m_mEllipses[ key ] = ellipse;
      return ellipse->GetEllipse();
    }
  else // ellipse in p gefunden
    {
      return m_mEllipses[ key ]->GetEllipse();
    }
}

} // end of namespace fawkes

#endif