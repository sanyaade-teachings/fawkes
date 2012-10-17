#ifndef __PLUGINS_COLLI_THREAD_H_
#define __PLUGINS_COLLI_THREAD_H_

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

#include "drive_realization/quadratic_motor_instruct.h"
#include "drive_modes/select_drive_mode.h"
#include "search/og_laser.h"
#include "search/astar_search.h"
#include "robo-utils/rob/robo_laser.h"
#define PI 3.14159265

#ifndef _COLLI_CELL_CONSTANTS_
#define _COLLI_CELL_CONSTANTS_     1
#define _COLLI_CELL_OCCUPIED_   1000.0
#define _COLLI_CELL_NEAR_          4.0 // near an obstacle    | COST  6!
#define _COLLI_CELL_MIDDLE_        3.0 // rel.near an obstacle| COST  4!
#define _COLLI_CELL_FAR_           2.0 // far from an obstacle| COST  2!
#define _COLLI_CELL_FREE_          1.0 // default free value  | COST  1!
#endif

//#incluse "colli_laser.h"
// Colli States
enum ColliState
  {
    NothingToDo,          // Indicating that nothing is to do
    OrientAtTarget,       // Indicating that the robot is at target and has to orient
    DriveToOrientPoint,   // Drive to the orientation point
    DriveToTarget,        // Drive to the target
  };

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

using namespace fawkes;
using namespace std;


const string default_hostname = "";
#ifdef HAVE_VISUAL_DEBUGGING
class ColliVisualizationThreadBase;
//class ColliNavigationThreadBase;
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
 #ifdef HAVE_VISUAL_DEBUGGING
  ColliVisualizationThreadBase *visthread_;
  void visualize_cells();
  void set_visualization_thread(ColliVisualizationThreadBase *visthread);
  void visualize_grid();

  //ColliNavigationThreadBase *navthread_;
  //void set_navigation_thread(ColliNavigationThreadBase *navthread);
 #endif

  ColliThread();
  virtual ~ColliThread();
  virtual void init();
  virtual void loop();

  virtual void finalize();
 private:
  fawkes::MotorInterface  *mopo_obj;
  fawkes::MotorInterface  *m_pMopoObj;
  fawkes::Laser360Interface *m_pLaserScannerObj;
  //fawkes::Laser720Interface *m_pLaserScannerObj;
  fawkes::NavigatorInterface *m_pColliTargetObj; 
  fawkes::NavigatorInterface *m_pColliDataObj;

  fawkes::Laser360Interface *m_pLaserScannerObjTest;
  fawkes::Laser720Interface *laser720;  
  fawkes::NavigatorInterface *ninit;
  fawkes::Laser360Interface *laserDeadSpots;
  fawkes::BlackBoard * bb_;

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

  int escape_count;                // count escaping behaviour

  // Config file constants that are read at the beginning
  int m_ColliFrequency;                          // frequency of the colli
  float m_OccGridHeight, m_OccGridWidth;         // occgrid field sizes
  int m_OccGridCellHeight, m_OccGridCellWidth;   // occgrid cell sizes
  float m_MaximumRoboIncrease;                   // maximum increasement of the robots size
  int m_RobocupMode;                             // indicator if robocup or not


  // stop on target stuff
  std::vector< float > m_oldAnglesToTarget;      // the old angles to the target

  // Do we  use a RWI Style Robot
  bool isRwiRobot;



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

 protected: virtual void run() { Thread::run(); }
 
};

#endif

