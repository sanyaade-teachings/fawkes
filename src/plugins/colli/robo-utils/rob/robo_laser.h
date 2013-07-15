
/***************************************************************************
 *  robo_laser.h - Provides access to the pure laser data
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

#ifndef _PLUGINS_COLLI_ROBO_UTILS_ROB_ROBO_LASER_H_
#define _PLUGINS_COLLI_ROBO_UTILS_ROB_ROBO_LASER_H_


#include "robo_laserpoint.h"

#include <tf/transform_listener.h>

#include <string.h>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Laser360Interface;
class Time;

/** My laser class.
 *  This is a class to access the laserreadings in a
 *   fast and efficient way.
 */
class Laser
{
 public:

  // =============================== //
  // CLASS METHODS                   //
  // =============================== //

  /** Constructor.
   *  This is the constructor. Has to be called with the laser
   *   object.
   *  \exception (int 1) The number of readings given by the laser are
   *             smaller or equal 0.... Perhaps the laser is currently
   *             offline. Try again!
   *  \exception (int 2) The readings array could not be allocated!
   *  @param laser is the bbClients Laser_Client.
   */
  //Laser( Laser360Interface *laser,
  // string remoteHostname="default" ) ;

  Laser( Laser360Interface *laser,
   std::string remoteHostname="default" ) ;

  /** Destructor.
   */
  ~Laser();


  // =============================== //
  // CONTINOUS METHOD                //
  // =============================== //

  /** Updates the laserdata.
   *  Call this with the Laser_Clientect in your
   *   Loop (the laserobject has to be updated
   *  previously, or you get no new data!).
   *  @return  -1 is no new data, so nothing is to do;
   *            0 is ok;
   *            1 is an error occured;
   */
  int UpdateLaser( );

  void transform(tf::Transformer *listener,std::string laser_frame);
  std::string laser_frame_;
  // ================================================= //
  // Return the actual readings. Nothing interpolated. //
  // ================================================= //

  /** Returns a special readings length.
   *  @param number is the number of this reading.
   *  @return float is the numbers length.
   */
  float GetReadingLength( const int number ) const;

  /** Returns a special readings x coordinate.
   *  @param number is the number of this reading.
   *  @return float is the numbers x coordinate.
   */
  float GetReadingPosX  ( const int number ) const;

  /** Returns a special readings y coordinate.
   *  @param number is the number of this reading.
   *  @return float is the numbers y coordinate.
   */
  float GetReadingPosY  ( const int number ) const;


  bool IsPipe( float i ) const;
  bool IsOnlyPipe( float i ) const;

  // ================================================= //
  // RETURN Misc Things                                //
  // ================================================= //

  // Return the number of readings we got.
  int GetNumberOfReadings() const;

  // Return the angle in radians for this reading
  float GetRadiansForReading( const int number ) const;

  // Returns the date of the current laserdata.
  // NOT USED Timestamp GetCurrentTimestamp() const;

  float TimeDiff() const;
  // ======================================================= //

  /** Offset of the laser X coordinate behind the motor axle */
  static const float LASER_X_OFFSET;

 protected:

  // the number of readings
  int m_NumberOfReadings;
  float m_Resolution;

  // our readings
  LaserPoint * m_pReadings;

 private:

  // METHODS, you don't have to care about

  void CalculateReadings();
  void CalculatePositions();

  // VARIABLES

  // the laser
  //Laser360Interface *m_pLaserScannerObj;
  Laser360Interface *m_pLaserScannerObj;
  Time * newtime;
  Time * oldtime;


  // are the pipes valid???
  bool m_bValidConfig;


  // ignore following readings
  int m_IgnoreFRStart;
  int m_IgnoreFREnd;

  int m_IgnoreRRStart;
  int m_IgnoreRREnd;

  int m_IgnoreRLStart;
  int m_IgnoreRLEnd;

  int m_IgnoreFLStart;
  int m_IgnoreFLEnd;

  // ignore following float readings
  float m_fIgnoreFRStart;
  float m_fIgnoreFREnd;

  float m_fIgnoreRRStart;
  float m_fIgnoreRREnd;

  float m_fIgnoreRLStart;
  float m_fIgnoreRLEnd;

  float m_fIgnoreFLStart;
  float m_fIgnoreFLEnd;


};


} // namespace fawkes

#endif

