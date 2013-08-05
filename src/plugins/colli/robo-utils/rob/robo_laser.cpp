/***************************************************************************
 *  robo_laser.cpp - Provides access to the pure laser data
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

#include "robo_laser.h"

#include <interfaces/Laser360Interface.h>
#include <utils/time/time.h>

#include <cmath>
#include <cstdio>
#include <iostream>

using namespace std;

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  const float Laser::LASER_X_OFFSET = 0.21;

// Initialize the Laser by giving the Laser_Client to it, and
// calculating the number of readings we get ( the size of the pos array )
// and initialize it directly
  Laser::Laser(Laser360Interface * laser, string remoteHostname)
  {
    newtime = new Time();
    newtime = &(newtime->stamp());
    oldtime = new Time();
    m_pLaserScannerObj = laser;

    m_NumberOfReadings = m_pLaserScannerObj->maxlenof_distances();

    if (m_NumberOfReadings < 1) {
      cout << endl << "*********" << endl;
      cout << "EXCEPTION in Laser(Constructor):" << endl;
      cout << "          Found less than 1 readings!" << endl;
      cout << "          Throwing exception 1" << endl << endl;
      return;
    }

    try {
      m_pReadings = new LaserPoint(m_NumberOfReadings);
    } catch (...) {
      return;
    }
    /*
     // calculate ignore reagions to floats from degree readings
     m_fIgnoreFRStart = ((float)m_IgnoreFRStart * M_PI)/180.0;
     m_fIgnoreFREnd   = ((float)m_IgnoreFREnd * M_PI)/180.0;

     m_fIgnoreRRStart = ((float)m_IgnoreRRStart * M_PI)/180.0;
     m_fIgnoreRREnd   = ((float)m_IgnoreRREnd * M_PI)/180.0;

     m_fIgnoreRLStart = ((float)m_IgnoreRLStart * M_PI)/180.0;
     m_fIgnoreRLEnd   = ((float)m_IgnoreRLEnd * M_PI)/180.0;

     m_fIgnoreFLStart = ((float)m_IgnoreFLStart * M_PI)/180.0;
     m_fIgnoreFLEnd   = ((float)m_IgnoreFLEnd * M_PI)/180.0;
     else
     {*/
    // Config non valid!
    m_fIgnoreFRStart = -1;
    m_fIgnoreFREnd = -1;

    m_fIgnoreRRStart = -1;
    m_fIgnoreRREnd = -1;

    m_fIgnoreRLStart = -1;
    m_fIgnoreRLEnd = -1;

    m_fIgnoreFLStart = -1;
    m_fIgnoreFLEnd = -1;
    // }
  }

// Destruct objects is killing all memory allocation
  Laser::~Laser()
  {
    delete this->newtime;
    delete this->oldtime;
    delete this->m_pReadings;
  }

// ** just as help function ** //
  inline float
  number2rad(int nr, int numberOfReadings = 360)
  {
    return (((float) nr) / ((float) numberOfReadings)) * 2. * M_PI;
  }

// Update the laserdata. Call this with the Laser_Clientect in your Loop.
// -1 is no new data, so nothing is to do; 0 is ok; 1 is error
  int
  Laser::UpdateLaser()
  {
    m_pLaserScannerObj->read();
    oldtime->set_time(newtime->get_sec(), newtime->get_usec());
    newtime = &(newtime->stamp());

    m_NumberOfReadings = m_pLaserScannerObj->maxlenof_distances();
    // get all readings
    CalculateReadings();
    CalculatePositions();

    return 0;
  }

// put all readings in scan object
  void
  Laser::CalculateReadings()
  {
    for (int i = 0; i < m_NumberOfReadings; i++) {
      m_pReadings->SetRadians(i, number2rad(i, m_NumberOfReadings));
      m_pReadings->SetLength(i, m_pLaserScannerObj->distances(i));
    }
  }

  void
  Laser::CalculatePositions()
  {
    for (int i = 0; i < m_NumberOfReadings; i++)
      m_pReadings->SetPos(i);
  }

//void Laser::transform(tf::TransformListener *tf_listener,string laser_frame)
  void
  Laser::transform(tf::Transformer *tf_listener, string laser_frame)
  {
    for (int i = 0; i < m_NumberOfReadings; i++) {
      float posX = m_pReadings->GetPosX(i);
      float posY = m_pReadings->GetPosY(i);
      tf::Stamped<tf::Point> base_point;
      tf::Stamped<tf::Point> laser_point(tf::Point(posX, posY, 0), fawkes::Time(0, 0), laser_frame);
      try {
        tf_listener->transform_point("/base_link", laser_point, base_point);

      } catch (...) {
        cout << "can't transform to the base link" << endl;
      }
      /* m_pReadings->SetPosX(i,posX);
       m_pReadings->SetPosY(i,-posY);*/
      m_pReadings->SetPosX(i, base_point.x());
      m_pReadings->SetPosY(i, -base_point.y());

      // ** in the case, we do not get laser readings in /base_laser
      if (m_pReadings->GetLength(i) <= 0) continue;

      tf::Stamped<tf::Point> blaser_point;
      tf::Stamped<tf::Point> tpoint(tf::Point(posX, posY, 0), fawkes::Time(0, 0), laser_frame);
      try {
        tf_listener->transform_point("/base_laser", tpoint, blaser_point);
        float length = sqrt(pow(blaser_point.x(), 2) + pow(blaser_point.y(), 2));
        m_pReadings->SetLength(i, length);
      } catch (...) {
        cout << "can't transform to the base laser" << endl;
      }
    }
    laser_frame_ = laser_frame;
  }

  /* =========================================================================================== */
  /* GETTER STUFF */
  /* =========================================================================================== */

// -------------------------- //
// LASERREADINGS GETTER       //
// -------------------------- //

// returns laser reading length
  float
  Laser::GetReadingLength(const int number) const
  {
    return m_pReadings->GetLength(number);
  }

// returns laser reading its positions x coordinate
  float
  Laser::GetReadingPosX(const int number) const
  {
    return m_pReadings->GetPosX(number);
  }

// returns laser reading its positions y coordinate
  float
  Laser::GetReadingPosY(const int number) const
  {
    return m_pReadings->GetPosY(number);
  }

// -------------------------- //
// MISC GETTER                //
// -------------------------- //

// returns number of readings available
  int
  Laser::GetNumberOfReadings() const
  {
    return m_NumberOfReadings;
  }

// returns current laserdata timestamp
  /* NOT USED Timestamp Laser::GetCurrentTimestamp() const
   {
   return m_pLaserScannerObj->GetTimestamp();
   }*/

// returns angle for reading
  float
  Laser::GetRadiansForReading(const int number) const
  {
    return m_pReadings->GetRadians(number);
  }

  /* NOT USED float Laser::TimeDiff() const
   {
   return (*newtime - *oldtime);
   }*/

// the famous is pipe method
  bool
  Laser::IsPipe(float i) const
  {
    i = normalize_rad(i);

    int reading_num = (int) (((i + 0.001) / (2.0 * M_PI)) * m_NumberOfReadings);
    // look below for explanation

    if (GetReadingLength(reading_num) == 0.0) return true;

    if (((i >= m_fIgnoreFRStart) && (i <= m_fIgnoreFREnd)) || ((i >= m_fIgnoreRRStart) && (i <= m_fIgnoreRREnd))
        || ((i >= m_fIgnoreRLStart) && (i <= m_fIgnoreRLEnd)) || ((i >= m_fIgnoreFLStart) && (i <= m_fIgnoreFLEnd))) {
      // Meist sind die Kabel das Problem!!!!
      // Wenn Danger if turning im Colli erscheint, erst Kabel checken.
      //      cout << "Got pipe at " << i << endl;
      return true;
    }

    return false;
  }

  bool
  Laser::IsOnlyPipe(float i) const
  {
    i = normalize_rad(i);

    //  int reading_num = (int)( ( (i+0.001) / (2.0*M_PI)) * m_NumberOfReadings );
    // -0.01 because of numerical problems...
    // numbers were before something like 1 2 3 5 6 6 7
    //    cout << i << " has reading num " << reading_num << " and length : " << GetReadingLength( reading_num ) << endl;

//   if ( ( (i > (26.0*M_PI)/180.0) && (i < (37.0*M_PI)/180.0) )
//        || ( (i > (95.0*M_PI)/180.0) && (i < (115.0*M_PI)/180.0) )
//        || ( (i > ((246.0-360.0)*M_PI)/180.0) && (i < ((263.0-360.0)*M_PI)/180.0) )
//        || ( (i > ((323.0-360.0)*M_PI)/180.0) && (i < ((334.0-360.0)*M_PI)/180.0) )
//       )

    if (((i > m_fIgnoreFRStart) && (i < m_fIgnoreFREnd)) || ((i > m_fIgnoreRRStart) && (i < m_fIgnoreRREnd))
        || ((i > m_fIgnoreRLStart) && (i < m_fIgnoreRLEnd)) || ((i > m_fIgnoreFLStart) && (i < m_fIgnoreFLEnd))) {
      // Meist sind die Kabel das Problem!!!!
      // Wenn Danger if turning im Colli erscheint, erst Kabel checken.
      //      cout << "Got pipe at " << i << endl;
      return true;
    }

    return false;

  }

} // namespace fawkes
