
/***************************************************************************
 *  trig_table.h - Implementation class trigonometry table
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

#ifndef _PLUGINS_COLLI_ROBO_UTILS_GEOMETRY_TRIG_TABLE_H_
#define _PLUGINS_COLLI_ROBO_UTILS_GEOMETRY_TRIG_TABLE_H_

#include <cmath>
#include <vector>
#include <utils/math/angle.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TrigTable <plugins/colli/robo-utils/geometry/trig_table.h>
 * Contains the implementation for putting laser scans in a
 *    Occupancy Grid (which is the father of this object).
 */

class TrigTable
{

public:

  /** @param Resolution is the number of steps per degree!
   *  @doc e.G. TrigTable( 4 ) means 4 steps per each degree.
   */
  TrigTable( int resolution_per_degree );

  /** destruct object
   */
  ~TrigTable();

  /** return linear interpolated sinus.
   */
  float GetSin( float radians );

  /** return linear interpolated cosinus.
   */
  float GetCos( float radians );


private:

  std::vector< float > m_vSinTable;
  std::vector< float > m_vCosTable;

  float InterpolateSin( float index_to_interpolate );
  float InterpolateCos( float index_to_interpolate );
  int m_Resolution;
};




inline TrigTable::TrigTable( int resolution_per_degree )
{
  m_Resolution = resolution_per_degree;
  m_vSinTable.reserve( 360*resolution_per_degree );
  m_vCosTable.reserve( 360*resolution_per_degree );

  for ( int i = 0; i < 360*resolution_per_degree; i++ )
    {
      m_vSinTable.push_back( sin( (i*M_PI)/(180.0*resolution_per_degree) ) );
      m_vCosTable.push_back( cos( (i*M_PI)/(180.0*resolution_per_degree) ) );
    }
}


inline TrigTable::~TrigTable( )
{
  m_vSinTable.clear();
  m_vCosTable.clear();
}


inline float TrigTable::GetSin( float radians )
{
  radians = normalize_rad( radians );
  float degree = rad2deg( radians ) * m_Resolution;
  int index = (int)degree;
  if ( index - degree == 0 )
    return m_vSinTable[index];
  else
    return InterpolateSin( degree );
}


inline float TrigTable::GetCos( float radians )
{
  radians = normalize_rad( radians );
  float degree = rad2deg( radians ) * m_Resolution;
  int index = (int)degree;
  if ( index - degree == 0 )
    return m_vCosTable[index];
  else
    return InterpolateCos( degree );

}


inline float TrigTable::InterpolateSin( float index_to_interpolate )
{
  float left = index_to_interpolate - (int)index_to_interpolate;
  float right = 1-left;
  return ( right* m_vSinTable[(int)index_to_interpolate] +
     left * m_vSinTable[((int)index_to_interpolate+1)%(360*m_Resolution)] );
}


inline float TrigTable::InterpolateCos( float index_to_interpolate )
{
  float left = index_to_interpolate - (int)index_to_interpolate;
  float right = 1-left;
  return ( right* m_vCosTable[(int)index_to_interpolate] +
     left * m_vCosTable[((int)index_to_interpolate+1)%(360*m_Resolution)] );
}

} // namespace fawkes

#endif
