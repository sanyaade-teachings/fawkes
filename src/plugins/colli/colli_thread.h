
/***************************************************************************
 *  colli_thread.h - Colli Thread
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013  AllemaniACs
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

#ifndef __PLUGINS_COLLI_COLLI_THREAD_H_
#define __PLUGINS_COLLI_COLLI_THREAD_H_

#include "drive_realization/quadratic_motor_instruct.h"
#include "drive_modes/select_drive_mode.h"
#include "search/og_laser.h"
#include "search/astar_search.h"
#include "robo-utils/rob/robo_laser.h"

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <utils/time/clock.h>
#include <utils/math/types.h>
#include <utils/math/angle.h>
#include <interfaces/MotorInterface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <interfaces/Position2DTrackInterface.h>
#include <interfaces/NavigatorInterface.h>
#include <geometry/hom_point.h>
#include <blackboard/remote.h>
#include <tf/transform_publisher.h>

#include <iostream>
#include <string.h>
#include <string>
#include <cstring>
#include <math.h>
#include <time.h>
#include <vector>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265

#ifndef _COLLI_CELL_CONSTANTS_
#define _COLLI_CELL_CONSTANTS_     1
#define _COLLI_CELL_OCCUPIED_   1000.0
#define _COLLI_CELL_NEAR_          4.0 // near an obstacle    | COST  6!
#define _COLLI_CELL_MIDDLE_        3.0 // rel.near an obstacle| COST  4!
#define _COLLI_CELL_FAR_           2.0 // far from an obstacle| COST  2!
#define _COLLI_CELL_FREE_          1.0 // default free value  | COST  1!
#endif

// Colli States
enum ColliState {
  NothingToDo,          // Indicating that nothing is to do
  OrientAtTarget,       // Indicating that the robot is at target and has to orient
  DriveToOrientPoint,   // Drive to the orientation point
  DriveToTarget,        // Drive to the target
};

const string default_hostname = "";
#ifdef HAVE_VISUAL_DEBUGGING
class ColliVisualizationThreadBase;
#endif

class ColliThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::TransformAspect,
  public fawkes::BlackBoardAspect
{
 public:
  ColliThread();
  virtual ~ColliThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 protected: virtual void run() { Thread::run(); }

 private:
 #ifdef HAVE_VISUAL_DEBUGGING
  ColliVisualizationThreadBase *visthread_;
  void visualize_cells();
  void set_visualization_thread(ColliVisualizationThreadBase *visthread);
  void visualize_grid();
 #endif
  fawkes::MotorInterface  *m_pMopoObj;
  fawkes::MotorInterface  *motor_des;
  fawkes::Laser360Interface *m_pLaserScannerObj;
  fawkes::NavigatorInterface *m_pColliTargetObj;
  fawkes::NavigatorInterface *m_pColliDataObj;

  fawkes::NavigatorInterface *ninit;

  void init_laser();
  void update_navi();

  fawkes::tf::TransformPublisher *m_tf_pub_odom;

  Laser*                          m_pLaser;            // laser interface for easy use
  CSearch*                        m_pSearch;           // our plan module which calculates the info

  CSelectDriveMode*               m_pSelectDriveMode;  // the drive mode selection module

  CBaseMotorInstruct*             m_pMotorInstruct;    // the motor instructor module

  CLaserOccupancyGrid*            m_pLaserOccGrid;     // the grid to drive on
  /* ************************************************************************ */
  /* PRIVATE VARIABLES THAT HAVE TO BE HANDLED ALL OVER THE MODULE            */
  /* ************************************************************************ */
  HomPoint  m_RoboGridPos;        // the robots position in the grid
  HomPoint  m_LaserGridPos;       // the laser its position in the grid ( not equal to robopos!!! )
  HomPoint  m_TargetGridPos;      // the targets position in the grid

  HomPoint  m_LocalGridTarget, m_LocalTarget;   // the local target (grid/relative)
  HomPoint  m_LocalGridTrajec, m_LocalTrajec;   // the local trajec (grid/relative)

  float m_ProposedTranslation;  // the proposed translation that should be realized in MotorInstruct
  float m_ProposedRotation;     // the proposed rotation that should be realized in MotorInstruct

  ColliState m_ColliStatus;     // representing current colli status

  float m_oldTargetX, m_oldTargetY, m_oldTargetOri;   // for init problems
  float m_TargetPointX, m_TargetPointY;               // for Update

  float m_OldX, m_OldY, m_OldOri;  // for updating occgrid and performing pipe compensation
  float m_Updx, m_Updy, m_UpdOri;
  float m_OldTargetPointX,m_OldTargetPointY;
  int escape_count;                // count escaping behaviour

  // Config file constants that are read at the beginning
  int m_ColliFrequency;                          // frequency of the colli
  float m_OccGridHeight, m_OccGridWidth;         // occgrid field sizes
  int m_OccGridCellHeight, m_OccGridCellWidth;   // occgrid cell sizes
  float m_MaximumRoboIncrease;                   // maximum increasement of the robots size
  int m_RobocupMode;                             // indicator if robocup or not
  int robo_widthX,robo_widthY;
  // stop on target stuff
  std::vector< float > m_oldAnglesToTarget;      // the old angles to the target

  // Do we  use a RWI Style Robot
  bool isRwiRobot;

  vector<HomPoint > m_vSolution;

  string laser_frame;
  string naviface_id;
  string laser_iface_id;
  string motor_iface_id;
  /* ************************************************************************ */
  /* PRIVATE METHODS                                                          */
  /* ************************************************************************ */
  /// Register all BB-Interfaces at the Blackboard.
  void RegisterAtBlackboard();

  /// Initialize all modules used by the Colli
  void InitializeModules();

  /// Get the newest values from the blackboard
  void UpdateBB();

  /// Check, in what state the colli is, and what to do
  void UpdateColliStateMachine();

  /// Calculate all information out of the updated blackboard data
  void UpdateOwnModules();

  /// Check, if we have to do escape mode, or if we have to drive the ordinary way ;-)
  bool CheckEscape();

  float GetMotorTranslation(float vtrans, float vori);
  float GetMotorOri(float odom_ori);

  void publish_odom();
  HomPoint transform_odom_to_base(HomPoint point);
  HomPoint transform_base_to_odom(HomPoint point);
  HomPoint nearest_cell_to_target();
  float motor_distance;
  bool adjust_robopos;
  inline float sqr( float x )
  {
    return (x*x);
  }

};

#endif

