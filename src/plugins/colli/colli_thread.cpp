
/***************************************************************************
 *  colli_thread.cpp - Colli Thread
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

#include "colli_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
 #include "visualization_thread_base.h"
#endif

#include "common/defines.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/NavigatorInterface.h>

using namespace fawkes;
using namespace std;


ColliThread::ColliThread()
  : Thread("ColliThread", Thread::OPMODE_CONTINUOUS)
{
 #ifdef HAVE_VISUAL_DEBUGGING
  visthread_ = NULL;
#endif

}


ColliThread::~ColliThread()
{
}


void
ColliThread::init()
{
  logger->log_info(name(),"COLLI (Constructor): Constructing...\n");
  if (!config->exists("/plugins/colli/Colli_FREQUENCY") )
  {
    cout << "***** ERROR *****: Could not get: Colli_FREQUENCY"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_ColliFrequency = (int)(1000.0/(float)config->get_int( "/plugins/colli/Colli_FREQUENCY" ));
    cout << "Colli_FREQUENCY " <<m_ColliFrequency << endl;
  }

  if (!config->exists("/plugins/colli/OccGrid_HEIGHT") )
  {
    cout << "***** ERROR *****: Could not get: OccGrid_HEIGHT"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_OccGridHeight = config->get_float( "/plugins/colli/OccGrid_HEIGHT" );
    //cout << "OccGrid_HEIGHT " << m_OccGridHeight << endl;
  }

  if (!config->exists("/plugins/colli/OccGrid_WIDTH") )
  {
    cout << "***** ERROR *****: Could not get: OccGrid_WIDTH"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_OccGridWidth  = config->get_float( "/plugins/colli/OccGrid_HEIGHT" );
    //cout << "OccGrid_HEIGHT " <<m_OccGridWidth << endl;
  }

  if (!config->exists("/plugins/colli/OccGrid_CELL_HEIGHT") )
  {
    cout << "***** ERROR *****: Could not get: OccGrid_CELL_HEIGHT"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_OccGridCellHeight = config->get_int("/plugins/colli/OccGrid_CELL_HEIGHT");
    //cout << "OccGrid_CELL_HEIGHT " <<m_OccGridCellHeight  << endl;
  }

  if (!config->exists("/plugins/colli/OccGrid_CELL_WIDTH") )
  {
    cout << "***** ERROR *****: Could not get: OccGrid_CELL_WIDTH"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_OccGridCellWidth  = config->get_int("/plugins/colli/OccGrid_CELL_WIDTH");
    //cout << "OccGrid_CELL_WIDTH " << m_OccGridCellWidth << endl;
  }

  if (!config->exists("/plugins/colli/Colli_MAX_ROBO_INCREASE") )
  {
    cout << "***** ERROR *****: Could not get: Colli_MAX_ROBO_INCREASE"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_MaximumRoboIncrease = config->get_float("/plugins/colli/Colli_MAX_ROBO_INCREASE");
    //cout << "Colli_MAX_ROBO_INCREASE " << m_MaximumRoboIncrease << endl;
  }

  if (!config->exists("/plugins/colli/Colli_ROBOCUP_MODE") )
  {
    cout << "***** ERROR *****: Could not get: Colli_ROBOCUP_MODE"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_RobocupMode = config->get_int("/plugins/colli/Colli_ROBOCUP_MODE");
    //cout << "Colli_ROBOCUP_MODE " << m_RobocupMode << endl;
  }

  if (!config->exists("/plugins/colli/Navigator_interface_id") )
  {
    cout << "***** ERROR *****: Could not get: Navigator_interface_id"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    naviface_id = config->get_string("/plugins/colli/Navigator_interface_id");
  }

  if (!config->exists("/plugins/colli/Laser_interface_id") )
  {
    cout << "***** ERROR *****: Could not get: Laser_interface_id"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    laser_iface_id = config->get_string("/plugins/colli/Laser_interface_id");
  }

  if (!config->exists("/plugins/colli/Motor_interface_id") )
  {
    cout << "***** ERROR *****: Could not get: Motor_interface_id"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    motor_iface_id = config->get_string("/plugins/colli/Motor_interface_id");
  }

  /* As default we use a football player AllemaniACs robot */
  if (default_hostname == "carl_rc.informatik.rwth-aachen.de")
    {
      isRwiRobot = true;
      cout << "COLLI (Constructor): Using a RWI Robot so this effects robs position in grid" << endl;
    }
  else
    {
      isRwiRobot = false;
      cout << "COLLI (Constructor): Using Colli for an AllemaniACs IKEA Style Robot" << endl;
    }

  if (!config->exists("/plugins/colli/MotorDistance") )
  {
    cout << "***** ERROR *****: Could not get: MotorDistance" << endl;
    motor_distance = 0;
  }
  else
  {
    motor_distance = config->get_float("/plugins/colli/MotorDistance");
  }

  if (!config->exists("/plugins/colli/adjust_robo_pos") )
  {
    cout << "***** ERROR *****: Could not get: adjust_robo_pos" << endl;
    adjust_robopos = false;
  }
  else
  {
    adjust_robopos = config->get_bool("/plugins/colli/adjust_robo_pos");
  }

  if( !config->exists("/plugins/colli/Roboshape/WIDTH_X") )
     adjust_robopos = false;
  else
    robo_widthX = config->get_float("/plugins/colli/Roboshape/WIDTH_X") * 100.;

  if( !config->exists("/plugins/colli/Roboshape/WIDTH_Y") )
    adjust_robopos = false;
  else
    robo_widthY = config->get_float("/plugins/colli/Roboshape/WIDTH_Y") * 100.;

  for ( unsigned int i = 0; i < 10; i++ )
    m_oldAnglesToTarget.push_back( 0.0 );

  srand( time( NULL ) );

  logger->log_info(name(),"COLLI (Constructor): Entering initialization ...\n");

  RegisterAtBlackboard();
  InitializeModules();


  m_oldTargetX   = m_pColliTargetObj->dest_x();
  m_oldTargetY   = m_pColliTargetObj->dest_y();
  m_oldTargetOri = m_pColliTargetObj->dest_ori();

  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  escape_count = 0;

  // Initialize the alive stuff
  /*BBAlive( "laser", 4 );
  BBAlive( "sim_robot", 4 );
  BBAlive( "laser_obstacles", 4);*/

  logger->log_info(name(),"COLLI (Constructor): Initialization done.\n");
  laser_frame = "/base_laser";
}


void
ColliThread::finalize()
{
  logger->log_info(name(),"COLLI (Destructor): Entering destructing ...\n");
  delete m_pSearch;
  delete m_pLaser;
  delete m_pMotorInstruct;
  delete m_pSelectDriveMode;
  delete m_pLaserOccGrid;
//  delete driveMode;
  // delete all registered bb-interfaces
  blackboard->close(m_pColliDataObj);
  blackboard->close(m_pColliTargetObj);
  blackboard->close(m_pLaserScannerObj);
  blackboard->close(m_pMopoObj);
  blackboard->close(motor_des);
  logger->log_info(name(),"COLLI (Destructor): Destructing done.\n");

}


/* **************************************************************************** */
/* **************************************************************************** */
/* ******************************  L O O P  *********************************** */
/* **************************************************************************** */
/* **************************************************************************** */

//
// ============================================================================ //
// ============================================================================ //
//                               BBCLIENT LOOP                                  //
//                               *************                                  //
//                                                                              //
//           The desired structure should be something like this                //
//           ===================================================                //
// Update the BB Things                                                         //
// Update the state machine                                                     //
//                                                                              //
// If we are in stop state                                                      //
//    Do stop                                                                   //
// Else if we are in orient state                                               //
//    Do orient                                                                 //
// else if we are in a drive state                                              //
//    Update the grid                                                           //
//    If we are to close to an obstacle                                         //
//       Escape the obstacle                                                    //
//       Get Motor settings for escaping                                        //
//       Set Motor parameters for escaping                                      //
//    else                                                                      //
//       Search for a way                                                       //
//       if we found a way,                                                     //
//          Translate the way in motor things                                   //
//          Set Motor parameters for driving                                    //
//       else                                                                   //
//          do nothing, because this is an error!                               //
//          Set Motor parameters for stopping                                   //
//                                                                              //
// Translate and Realize the motor commands                                     //
// Update the BB Things                                                         //
//                                                                              //
// ============================================================================ //
// ============================================================================ //
//


 #ifdef HAVE_VISUAL_DEBUGGING
void
ColliThread::visualize_cells()
{
  vector<HomPoint > occ_cells;
  vector<HomPoint > near_cells;
  vector<HomPoint > far_cells;
  vector<HomPoint > middle_cells;
  vector<HomPoint > laser_points;
  vector<HomPoint > free_cells;
  for ( int gridY = 0; gridY < m_pLaserOccGrid->getHeight(); gridY++ )
  {
    for ( int gridX = 0; gridX < m_pLaserOccGrid->getWidth(); gridX++ )
    {
      if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_OCCUPIED_ )
      {
        HomPoint p(gridX,gridY);
        occ_cells.push_back(p);
      }
      else if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_NEAR_ )
      {
        HomPoint p(gridX,gridY);
        near_cells.push_back(p);
      }
      else if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_FAR_ )
      {
        HomPoint p(gridX,gridY);
        far_cells.push_back(p);
      }
      else if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_MIDDLE_ )
      {
        HomPoint p(gridX,gridY);
        middle_cells.push_back(p);
      }
      else if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_FREE_ )
      {
        free_cells.push_back(HomPoint(gridX,gridY));
      }
    }
  }
  vector<HomPoint > search_occ_cells;
  OccupancyGrid * socc = m_pSearch->get_astar_grid();
  for ( int gridY = 0; gridY < m_pLaserOccGrid->getHeight(); gridY++ )
  {
    for ( int gridX = 0; gridX < m_pLaserOccGrid->getWidth(); gridX++ )
    {
      if ( socc->getProb( gridX, gridY ) == _COLLI_CELL_OCCUPIED_ )
      {
        search_occ_cells.push_back(HomPoint(gridX,gridY));
      }
    }
  }

  for( int i = 0; i < m_pLaser->GetNumberOfReadings(); i++ )
  {
    float posx = m_pLaser->GetReadingPosX(i);
    float posy = m_pLaser->GetReadingPosY(i);
    HomPoint plaser(posx,posy);
    laser_points.push_back(plaser);
  }

  vector<HomPoint > orig_laser_points;
  m_pLaserScannerObj->read();
  for( unsigned int i = 0; i < m_pLaserScannerObj->maxlenof_distances(); i++ )
  {
    float ori = float(i) * M_PI / 180.;
    float posx = m_pLaserScannerObj->distances(i) * cos(ori);
    float posy = m_pLaserScannerObj->distances(i) * sin(ori);
    HomPoint plaser(posx,posy);
    orig_laser_points.push_back(plaser);
  }

  vector< HomPoint > plan = m_pSearch->GetPlan();
  HomPoint target = m_TargetGridPos;
  HomPoint target_odom = HomPoint(m_pColliTargetObj->dest_x(),m_pColliTargetObj->dest_y());
  float m_des_x = m_ProposedTranslation * cos(m_ProposedRotation);
  float m_des_y = -m_ProposedTranslation * sin(m_ProposedRotation);

  HomPoint motor_des_point(m_des_x,m_des_y);
  float m_x = m_pMopoObj->vx() * cos(m_pMopoObj->omega());
  float m_y = m_pMopoObj->vx() * sin(m_pMopoObj->omega());
  HomPoint motor_real(m_x,m_y);
  vector<HomPoint > astar_found_occ = m_pSearch->get_occ_astar_search();
  vector<HomPoint > seen_states = m_pSearch->get_astar_states();

  HomPoint modTarget = m_pSearch->get_mod_target();
  visthread_->visualize("/base_link",occ_cells,near_cells,far_cells,middle_cells,m_RoboGridPos,m_LaserGridPos,laser_points,plan,motor_des_point,
                       m_pLaserOccGrid->getCellWidth(),m_pLaserOccGrid->getCellHeight(),target,
                       m_OccGridWidth,m_OccGridHeight,motor_real,m_LocalTarget,target_odom,orig_laser_points,
                       search_occ_cells,astar_found_occ,free_cells,seen_states,modTarget);
}
#endif


void
ColliThread::loop()
{
  #ifdef HAVE_VISUAL_DEBUGGING
  visualize_cells();
  #endif
  publish_odom();
  // to be on the sure side of life
  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;
  // Update blackboard data
  UpdateBB();


//  BBPing();
 // BBOperate();
/* TODO
  if ( !( BBAlive( "laser", 4 ) ||
          BBAlive( "sim_robot", 4 ) ||
          BBAlive( "laser_obstacles", 4 ) ) )
    {
      cout << "***** Laser or sim_robot dead!!! --> STOPPING!!!!" << endl;
      m_pMotorInstruct->Drive( 0.0, 0.0 );
      m_pColliDataObj->SetFinal( true );
      m_pColliDataObj->UpdateBB();
      BBOperate();
      escape_count = 0;
      return;
    }
*/
  // THIS IF FOR CHALLENGE ONLY!!!
  //if ( m_pColliTargetObj->GetColliMode() == (int)(OVERRIDE) ) // ** NOT SURE
  if ( m_pColliTargetObj->flags() == (int)(OVERRIDE) )
    {
      std::cout << "BEING OVERRIDDEN!" << endl << endl;
      m_pColliDataObj->set_final( false );
      m_pColliDataObj->write();
      escape_count = 0;
      return;
    }


  if ((int) m_pColliTargetObj->error_code())
    {
      logger->log_error(name(),"Moving is not allowed!\n\n");
      m_pMotorInstruct->Drive( 0.0, 0.0 );
      m_pColliDataObj->set_final(true);
      m_pColliDataObj->write();
      escape_count = 0;
      return;
    }


  // Do only drive, if there is a new (first) target
  if ( ( m_oldTargetX   == m_pColliTargetObj->dest_x() ) &&
       ( m_oldTargetY   == m_pColliTargetObj->dest_y() ) &&
       ( m_oldTargetOri == m_pColliTargetObj->dest_ori() ) )
    {
      m_oldAnglesToTarget.clear();
      for ( unsigned int i = 0; i < 10; i++ )
        m_oldAnglesToTarget.push_back( 0.0 );

      m_ProposedTranslation = 0.0;
      m_ProposedRotation    = 0.0;
      m_pColliDataObj->set_final( true );
      m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

      escape_count = 0;
      // Send motor and colli data away.
      m_pColliDataObj->write();
      return;
    }
  else
    {
      m_oldTargetX   = m_pColliTargetObj->dest_x()   + 1000.0;
      m_oldTargetY   = m_pColliTargetObj->dest_y()   + 1000.0;
      m_oldTargetOri = m_pColliTargetObj->dest_ori() + 1.0;
      m_OldTargetPointX = m_pColliTargetObj->dest_x();
      m_OldTargetPointY = m_pColliTargetObj->dest_y();
    }

  // Update state machine
  UpdateColliStateMachine();

  // nothing is to do
  if (m_ColliStatus == NothingToDo)
    {
      m_pLaserOccGrid->ResetOld();
      m_ProposedTranslation = 0.0;
      m_ProposedRotation    = 0.0;
      m_pColliDataObj->set_final( true );

      m_oldTargetX   = m_pColliTargetObj->dest_x();
      m_oldTargetY   = m_pColliTargetObj->dest_y();
      m_oldTargetOri = m_pColliTargetObj->dest_ori();

      m_pLaserOccGrid->ResetOld();

      escape_count = 0;
    }
  else
    {
      // perform the update of the grid.
      UpdateOwnModules();
      m_pColliDataObj->set_final( false );
      if ( m_pMopoObj->motor_state () == m_pMopoObj->MOTOR_DISABLED )
      {
        m_pMotorInstruct->Drive( 0.0, 0.0 );
      }

      // Check, if one of our positions (robo-, laser-gridpos is not valid) => Danger!
      if ( CheckEscape() == true || escape_count > 0 )
        {

          if ( GetMotorTranslation(motor_des->vx(),motor_des->omega()) == 0.0 &&
               motor_des->omega() == 0.0 )
            {
              m_pLaserOccGrid->ResetOld();
            }

          // ueber denken und testen
          if(m_pColliTargetObj->is_escaping_enabled () == true)
            {
             // SJTODO: ERST wenn ich gestoppt habe, escape mode anwerfen!!!
              if (escape_count > 0)
                {
                  escape_count--;
                }
              else
                {
                  int rnd = (int)((rand())/(float)(RAND_MAX)) * 10; // + 5;
                  escape_count = rnd;
                  logger->log_info(name(),"Escape: new round with %d\n",rnd);
                }
              logger->log_info(name(),"Escape mode, escaping!\n");
              m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x(), m_LocalTarget.y() );
              m_pSelectDriveMode->Update( true );  // <-- this calls the ESCAPE mode!
              m_ProposedTranslation = m_pSelectDriveMode->GetProposedTranslation();
              m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();
            }
          else
            {
              logger->log_error(name(),"Escape mode, but not allowed!\n");
              m_ProposedTranslation = 0.0;
              m_ProposedRotation    = 0.0;
              escape_count = 0;
            }
        }
      else
        { // only orienting to do and moving possible
          if (m_ColliStatus == OrientAtTarget)
            {
              logger->log_info(name(),"colli state: orient at target\n");
              m_ProposedTranslation = 0.0;
              // turn faster if angle-diff is high
              m_ProposedRotation    = 1.0*normalize_mirror_rad( m_pColliTargetObj->dest_ori() -
                                                                GetMotorOri(m_pMopoObj->omega()) );
              // but at least use 0.1 rad/s
              if ( m_ProposedRotation > 0.0 )
                m_ProposedRotation = max(  0.1, (double)m_ProposedRotation );
              else
                m_ProposedRotation = min( -0.1, (double)m_ProposedRotation );

               m_pLaserOccGrid->ResetOld();
            }
          else
            { // search for a path
               m_pSearch->Update( (int)m_RoboGridPos.x(), (int)m_RoboGridPos.y(),
                                 (int)m_TargetGridPos.x(), (int)m_TargetGridPos.y() );
              if ( m_pSearch->UpdatedSuccessful() )
                { //   if path exists,
                  m_vSolution.clear();
                  m_LocalGridTarget = m_pSearch->GetLocalTarget();
                  m_LocalGridTrajec = m_pSearch->GetLocalTrajec();
                  // coordinate transformation from grid coordinates to relative robot coordinates
                  if( !adjust_robopos )
                  {
                    m_LocalTarget = HomPoint( (m_LocalGridTarget.x() - m_RoboGridPos.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTarget.y() - m_RoboGridPos.y())*m_pLaserOccGrid->getCellHeight()/100.0 );
                    m_LocalTrajec = HomPoint( (m_LocalGridTrajec.x() - m_RoboGridPos.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTrajec.y() - m_RoboGridPos.y())*m_pLaserOccGrid->getCellHeight()/100.0 );
                  }
                  else
                  {
                    HomPoint mod_robo = nearest_cell_to_target();
                    m_LocalTarget = HomPoint( (m_LocalGridTarget.x() - mod_robo.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTarget.y() - mod_robo.y())*m_pLaserOccGrid->getCellHeight()/100.0 );
                    m_LocalTrajec = HomPoint( (m_LocalGridTrajec.x() - mod_robo.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTrajec.y() - mod_robo.y())*m_pLaserOccGrid->getCellHeight()/100.0 );
                  }
                  // call appopriate drive mode
                  m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x(), m_LocalTarget.y() );
                  m_pSelectDriveMode->SetLocalTrajec( m_LocalTrajec.x(), m_LocalTrajec.y() );
                  m_pSelectDriveMode->Update();
                  m_ProposedTranslation = m_pSelectDriveMode->GetProposedTranslation();
                  m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();
                }
              else
                { // else stop
                  logger->log_info(name(),"   Drive Mode: Update not successful ---> stopping!\n");
                  m_LocalTarget = HomPoint( 0.0, 0.0 );
                  m_LocalTrajec = HomPoint( 0.0, 0.0 );
                  m_ProposedTranslation = 0.0;
                  m_ProposedRotation    = 0.0;
                  m_pLaserOccGrid->ResetOld();
                }
              m_pColliDataObj->set_x( m_LocalTarget.x() );
              m_pColliDataObj->set_y( m_LocalTarget.y() );
              m_pColliDataObj->set_dest_x( m_LocalTrajec.x() );
              m_pColliDataObj->set_dest_y( m_LocalTrajec.y() );
            }
        }
    }
  cout << "I want to realize " << m_ProposedTranslation << ", " << m_ProposedRotation << endl;
  // Realize drive mode proposal with realization module
  m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

  cout << endl << endl;

  // Send motor and colli data away.
  m_pColliDataObj->write();

}


void
ColliThread::RegisterAtBlackboard()
{
  m_tf_pub_odom = new tf::TransformPublisher(blackboard, "colli odometry");

  m_pMopoObj = blackboard->open_for_reading<MotorInterface>(motor_iface_id.c_str());
  motor_des = blackboard->open_for_writing<MotorInterface>("Motor Caesar");
  m_pLaserScannerObj = blackboard->open_for_reading<Laser360Interface>(laser_iface_id.c_str());
  m_pColliTargetObj = blackboard->open_for_reading<NavigatorInterface>(naviface_id.c_str());
  m_pColliDataObj = blackboard->open_for_writing<NavigatorInterface>("Navigator Temp");


  m_pMopoObj->read();
  m_pLaserScannerObj->read();
  m_pColliTargetObj->read();
  m_pColliDataObj->read();

  m_pColliDataObj->set_final( true );
  m_pColliDataObj->write();
  m_pColliTargetObj->read();

  ninit = blackboard->open_for_writing<NavigatorInterface>(naviface_id.c_str());
}


/** Initialize all modules used by the Colli. */
void
ColliThread::InitializeModules()
{
  laser_frame = m_pLaserScannerObj->frame();
  // FIRST(!): the laserinterface (uses the laserscanner)
  m_pLaser = new Laser( m_pLaserScannerObj, "" );
  m_pLaser->UpdateLaser();
  m_pLaser->transform(tf_listener,laser_frame);

  // SECOND(!): the occupancy grid (it uses the laser)

  // set the cell width and heigth to 5 cm and the grid size to 7.5 m x 7.5 m.
  // this are 750/5 x 750/5 grid cells -> (750x750)/5 = 22500 grid cells

  m_pLaserOccGrid = new CLaserOccupancyGrid( logger, config, m_pLaser, (int) ((m_OccGridWidth*100)/m_OccGridCellWidth),
                                            (int)((m_OccGridHeight*100)/m_OccGridCellHeight),
                                            m_OccGridCellWidth, m_OccGridCellHeight);
  // THIRD(!): the search component (it uses the occ grid (without the laser)
  m_pSearch = new CSearch( logger, config, m_pLaserOccGrid );

  // BEFORE DRIVE MODE: the motorinstruction set
  m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( motor_des, m_pMopoObj,m_ColliFrequency, logger, config );
  //m_pMotorInstruct->SetRecoverEmergencyStop();


  // AFTER MOTOR INSTRUCT: the motor propose values object
  m_pSelectDriveMode = new CSelectDriveMode( motor_des, m_pLaser, m_pColliTargetObj, logger, config );

  // Initialization of colli state machine:
  // Currently nothing is to accomplish
  m_ColliStatus  = NothingToDo;
  m_oldTargetX   = m_pColliTargetObj->dest_x();
  m_oldTargetY   = m_pColliTargetObj->dest_y();
  m_oldTargetOri = m_pColliTargetObj->dest_ori();

  m_OldX   = m_pMopoObj->odometry_position_x();
  m_OldY   = m_pMopoObj->odometry_position_y();
  m_OldOri = GetMotorOri(m_pMopoObj->odometry_orientation());

  m_Updx = m_pMopoObj->odometry_position_x();
  m_Updy = m_pMopoObj->odometry_position_y();
  m_UpdOri = GetMotorOri(m_pMopoObj->odometry_orientation());

  m_vSolution.clear();
}


/** Get the newest values from the blackboard. */
void
ColliThread::UpdateBB()
{
  m_pLaserScannerObj->read();

  m_pMopoObj->read();
  motor_des->set_odometry_position_x(m_pMopoObj->odometry_position_x());
  motor_des->set_odometry_position_y(m_pMopoObj->odometry_position_y());
  motor_des->set_odometry_orientation(m_pMopoObj->odometry_orientation());
  //motor_des->set_motor_state(m_pMopoObj->motor_state());
  motor_des->set_vx(m_pMopoObj->vx());
  motor_des->set_omega(m_pMopoObj->omega());
  motor_des->write();

  motor_des->read();
  update_navi();
  m_pColliTargetObj->read();
  m_pColliDataObj->read();
}


void
ColliThread::update_navi()
{
  if((! ninit->msgq_empty()))
  {
    if( ninit->msgq_first_is<NavigatorInterface::CartesianGotoMessage>() )
    {
      NavigatorInterface::CartesianGotoMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      HomPoint trans = transform_base_to_odom(HomPoint(msgTmp->x(),msgTmp->y()));
      logger->log_info(name(),"transforemed to odom is %f,%f",trans.x(),trans.y());
      ninit->set_dest_x(trans.x());
      ninit->set_dest_y(trans.y());
      ninit->set_dest_ori(msgTmp->orientation());
      ninit->write();
      ninit->msgq_pop();
    }
    else if( ninit->msgq_first_is<NavigatorInterface::PolarGotoMessage>() )
    {
      NavigatorInterface::PolarGotoMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      float new_x = msgTmp->dist() * cos(msgTmp->phi());
      float new_y = msgTmp->dist() * sin(msgTmp->phi());
      HomPoint trans = transform_base_to_odom(HomPoint(new_x,new_y));
      ninit->set_dest_x(trans.x());
      ninit->set_dest_y(trans.y());
      ninit->set_dest_ori(msgTmp->orientation());
      ninit->write();
      ninit->msgq_pop();
    }
  }
  if( ! ninit->msgq_empty() )
  {
    if( ninit->msgq_first_is<NavigatorInterface::SetDriveModeMessage>() )
    {
      NavigatorInterface::SetDriveModeMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_drive_mode(msgTmp->drive_mode());
      ninit->write();
      ninit->msgq_pop();
    }
  }
  if(! ninit->msgq_empty())
  {
    if( ninit->msgq_first_is<NavigatorInterface::SetMaxVelocityMessage>() )
    {
      NavigatorInterface::SetMaxVelocityMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_max_velocity(msgTmp->max_velocity());
      ninit->write();
      ninit->msgq_pop();
    }
  }
  if(! ninit->msgq_empty())
  {
    if( ninit->msgq_first_is<NavigatorInterface::SetSecurityDistanceMessage>() )
    {
      NavigatorInterface::SetSecurityDistanceMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_security_distance(msgTmp->security_distance());
      ninit->write();
      ninit->msgq_pop();
    }
  }
  if(! ninit->msgq_empty())
  {
    if( ninit->msgq_first_is<NavigatorInterface::SetEscapingMessage>() )
    {
      NavigatorInterface::SetEscapingMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_escaping_enabled(msgTmp->is_escaping_enabled());
      ninit->write();
      ninit->msgq_pop();
    }
  }
  //ninit->set_escaping_enabled(true);
  ninit->write();
}


void
ColliThread::UpdateColliStateMachine()
{
  // initialize
  m_ColliStatus = NothingToDo;

  float curPosX = m_pMopoObj->odometry_position_x ();
  float curPosY = m_pMopoObj->odometry_position_y ();
  float curPosO = GetMotorOri(m_pMopoObj->odometry_orientation ());

  float targetX = m_pColliTargetObj->dest_x();
  float targetY = m_pColliTargetObj->dest_y();
  float targetO = m_pColliTargetObj->dest_ori();

  bool  orient = m_pColliTargetObj->dest_ori();
  // Real driving....
  if ( ( orient == true ) &&
       ( sqr( curPosX - targetX ) + sqr( curPosY - targetY ) >= sqr(2.1) ) )
    {
      float ori = m_pColliTargetObj->dest_ori();

      float mult = 0.0;
  /*  NOT USED  //if ( m_pMotorInstruct->GetUserDesiredTranslation() > 0 )
        //      mult =  1.2;
        mult = 0.8;
      else if ( m_pMotorInstruct->GetUserDesiredTranslation() < 0 )
        //      mult = -1.2;
        mult = -0.8;
      else
        mult = 0.0;
*/

      float orientPointX = targetX - ( mult * cos(ori) );
      float orientPointY = targetY - ( mult * sin(ori) );

      m_TargetPointX = orientPointX;
      m_TargetPointY = orientPointY;
      m_ColliStatus = DriveToOrientPoint;
      return;
    }
    else if ( sqr( curPosX - targetX ) + sqr( curPosY - targetY ) > sqr(0.15) )
    {
      m_TargetPointX = targetX;
      m_TargetPointY = targetY;
      m_ColliStatus = DriveToTarget;
      logger->log_info(name(),"colli state: drive to target, distance is %f\n", sqr( curPosX - targetX ) + sqr( curPosY - targetY ));
      return;
    }
  else if ( (orient == true) &&
            ( fabs( normalize_mirror_rad(curPosO - targetO) ) > 0.1 ) )
    {
      logger->log_info(name(),"colli state: orient at target\n");
      m_ColliStatus = OrientAtTarget;
      return;
    }
  else
    {
      logger->log_info(name(),"colli state: nothing to do\n");
      m_ColliStatus = NothingToDo;
      return;
    }

  logger->log_info(name(),"**** COLLI ****: Here I should never never be\n");
  return;

}


/** Calculate all information out of the updated blackboard data.
 * m_RoboGridPos, m_LaserGridPos, m_TargetGridPos have to be updated!
 * the targetPointX and targetPointY were calculated in the collis state machine!
 */
void
ColliThread::UpdateOwnModules()
{
  if ( m_RobocupMode == 1 )  // Robocup mode
    {
      // set the cell size according to the current speed
      m_pLaserOccGrid->setCellWidth( (int)m_OccGridCellWidth );
      m_pLaserOccGrid->setCellHeight( (int)m_OccGridCellHeight );
    }
  else
    {
      // set the cell size according to the current speed
      m_pLaserOccGrid->setCellWidth(  (int)max( (int)m_OccGridCellWidth,
                                                (int)(5*fabs(GetMotorTranslation(motor_des->vx(),motor_des->omega()))+3) ) );
      m_pLaserOccGrid->setCellHeight( (int)max( (int)m_OccGridCellHeight,
                                                (int)(5*fabs(GetMotorTranslation(motor_des->vx(),motor_des->omega()))+3) ) );
    }
    logger->log_info(name(),"cell size is %d,%d",m_pLaserOccGrid->getCellWidth(),m_pLaserOccGrid->getCellHeight());
  // Calculate discrete position of the laser whicj is different for RWI Robots
  int laserpos_x;

  if (isRwiRobot)
    {
      laserpos_x = 0;
    }
  else
    {
      laserpos_x = (int)(m_pLaserOccGrid->getWidth() / 2);
    }
  int laserpos_y = (int)(m_pLaserOccGrid->getHeight() / 2);
  laserpos_x -= (int)( GetMotorTranslation(motor_des->vx(),motor_des->omega())*m_pLaserOccGrid->getWidth() / (2*3.0) );
  laserpos_x  = max ( laserpos_x, 10 );
  laserpos_x  = min ( laserpos_x, (int)(m_pLaserOccGrid->getWidth()-10) );

  int robopos_x = laserpos_x + (int)(motor_distance/(float)m_pLaserOccGrid->getCellWidth());
  int robopos_y = laserpos_y;

  // coordinate transformation for target point
  float aX = m_TargetPointX - m_pMopoObj->odometry_position_x ();
  float aY = m_TargetPointY - m_pMopoObj->odometry_position_y ();
  float targetContX = ( aX*cos( GetMotorOri(m_pMopoObj->odometry_orientation () ) ) + aY*sin( GetMotorOri(m_pMopoObj->odometry_orientation ()) ) );
  float targetContY = (-aX*sin( GetMotorOri(m_pMopoObj->odometry_orientation ()) ) + aY*cos( GetMotorOri(m_pMopoObj->odometry_orientation ()) ) );
  // calculation, where in the grid the target is, thats relative to the motorpos, so add it ;-)
  int targetGridX = (int)( (targetContX * 100.0) / (float)m_pLaserOccGrid->getCellWidth() );
  int targetGridY = (int)( (targetContY * 100.0) / (float)m_pLaserOccGrid->getCellHeight() );

  targetGridX += robopos_x;
  targetGridY += robopos_y;

  // check the target borders. if its out of the occ grid, put it back in by border checking
  // with linear interpolation
  if (targetGridX >= m_pLaserOccGrid->getWidth()-1)
    {
      targetGridY = robopos_y + ((robopos_x - (m_pLaserOccGrid->getWidth()-2))/(robopos_x - targetGridX) * (targetGridY - robopos_y));
      targetGridX = m_pLaserOccGrid->getWidth()-2;
    }
  if (targetGridX < 2)
    {
      targetGridY = robopos_y + ((robopos_x-2)/(robopos_x - targetGridX) * (targetGridY - robopos_y));
      targetGridX = 2;
    }

  if (targetGridY >= m_pLaserOccGrid->getHeight()-1)
    {
      targetGridX = robopos_x + ((robopos_y - (m_pLaserOccGrid->getHeight()-2))/(robopos_y - targetGridY) * (targetGridX - robopos_x));
      targetGridY = m_pLaserOccGrid->getHeight()-2;
    }
  if (targetGridY < 2)
    {
      targetGridX = robopos_x + ((robopos_y-2)/(robopos_y - targetGridY) * (targetGridX - robopos_x));
      targetGridY = 2;
    }

  // update the laser
  m_pLaser->UpdateLaser();
  m_pLaser->transform(tf_listener,laser_frame);
  // Robo increasement for robots
  float m_RoboIncrease = 0.0;

  if ( m_RobocupMode == 1 )  // Robocup mode
    {
      if ( m_pColliTargetObj->security_distance() > 0.0 )
        {
          m_RoboIncrease = m_pColliTargetObj->security_distance();
          logger->log_info(name(),"UpdateOwnModules: Setting EXTERN Robot secure distance = %f . ATTENTION TO THE ROBOT!!!!\n", m_RoboIncrease);
        }
      else
        {
          m_RoboIncrease = 0.0;
        }
    }
  else  // no robocup mode
    {
      if ( m_pColliTargetObj->security_distance() > 0.0 )
        {
          m_RoboIncrease = m_pColliTargetObj->security_distance();
          logger->log_info(name(),"UpdateOwnModules: Setting EXTERN Robot secure distance = %f ATTENTION TO THE ROBOT!!!!\n"
                          , m_RoboIncrease);
        }
      else
        {
          float transinc = max(0.0,fabs( GetMotorTranslation(m_pMopoObj->vx(),m_pMopoObj->omega())/2.0 )-0.7);
          float rotinc   = max(0.0,fabs( m_pMopoObj->omega()/3.5 )-0.7);

          m_RoboIncrease = max( transinc, rotinc );
          m_RoboIncrease = min( m_MaximumRoboIncrease, m_RoboIncrease );
        }
    }

  float xdiff = m_pMopoObj->odometry_position_x() - m_OldX;
  m_OldX = m_pMopoObj->odometry_position_x();
  float ydiff = m_pMopoObj->odometry_position_y() - m_OldY;
  m_OldY = m_pMopoObj->odometry_position_y();
  float oridiff = normalize_mirror_rad( m_pMopoObj->odometry_orientation() - m_OldOri );
  m_OldOri = GetMotorOri(m_pMopoObj->odometry_orientation());

  float relxdiff =  xdiff *  cos( GetMotorOri(m_pMopoObj->odometry_orientation()) ) +
                    ydiff *  sin( GetMotorOri(m_pMopoObj->odometry_orientation()) );
  float relydiff =  xdiff * -sin( GetMotorOri(m_pMopoObj->odometry_orientation()) ) +
                    ydiff *  cos( GetMotorOri(m_pMopoObj->odometry_orientation()) );

  // update the occgrid...
  m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease,GetMotorTranslation(motor_des->vx(),motor_des->omega()),relxdiff, relydiff, oridiff );
  // update the positions
  m_LaserGridPos  = HomPoint( laserpos_x, laserpos_y );
  m_RoboGridPos   = HomPoint( robopos_x, robopos_y );
  m_TargetGridPos = HomPoint( targetGridX, targetGridY );
}


/** Check if we want to escape an obstacle. */
bool
ColliThread::CheckEscape()
{
  if ((float)m_pLaserOccGrid->getProb((int)m_RoboGridPos.x(),(int)m_RoboGridPos.y()) ==  _COLLI_CELL_OCCUPIED_ )
    {
      return true;
    }
  else
    {
      return false;
    }
}


HomPoint
ColliThread::nearest_cell_to_target()
{
  int cell_size = robo_widthX;
  if( robo_widthY < cell_size )
    cell_size = robo_widthY;
  cell_size /= m_pLaserOccGrid->getCellWidth();
  cell_size /= 2;
  int min_roboX = m_RoboGridPos.x();
  int min_roboY = m_RoboGridPos.y();
  float min_dis = sqrt(pow(min_roboX-m_TargetGridPos.x(),2)+pow(min_roboY-m_TargetGridPos.y(),2));
  for( int i = m_RoboGridPos.x() - cell_size; i <= m_RoboGridPos.x() + cell_size; i++ )
  {
    for( int j =  m_RoboGridPos.y() - cell_size; j <= m_RoboGridPos.y() + cell_size; j++ )
    {
      if( ( i >= 0 ) && ( j >= 0 ) && ( i < (int)m_pLaserOccGrid->getWidth() ) && ( j < (int)m_pLaserOccGrid->getHeight()) )
      {
        float dis = sqrt(pow(i-m_TargetGridPos.x(),2)+pow(j-m_TargetGridPos.y(),2));
        if( dis < min_dis )
        {
          min_dis = dis;
          min_roboX = i;
          min_roboY = j;
        }
      }
    }
  }
  return HomPoint(min_roboX,min_roboY);
}


#ifdef HAVE_VISUAL_DEBUGGING
void
ColliThread::set_visualization_thread(ColliVisualizationThreadBase *visthread)
{
  visthread_ = visthread;
  if(visthread_ ) cout << "visualization thread set"<< endl;
}
#endif


void
ColliThread::publish_odom()
{
  tf::Quaternion o_r(-m_pMopoObj->odometry_orientation(), 0, 0);
  tf::Vector3 o_t(m_pMopoObj->odometry_position_x(),-m_pMopoObj->odometry_position_y(), 0);
  tf::Transform o_tr(o_r, o_t);
  Time *o_ts = new Time();
  o_ts = &(o_ts->stamp());
  fawkes::Time o_time(o_ts->get_sec(), o_ts->get_usec());
  m_tf_pub_odom->send_transform(o_tr, o_time, "/odom", "/base_link");
}


HomPoint
ColliThread::transform_odom_to_base(HomPoint point)
{
  HomPoint res;
  try {
    //tf::Stamped<tf::Point> target_odom(tf::Point(point.x(),-point.y(),0.),fawkes::Time(0, 0), "/odom");
    tf::Stamped<tf::Point> target_odom(tf::Point(point.x(),point.y(),0.),fawkes::Time(0, 0), "/odom");
    tf::Stamped<tf::Point> baserel_target;
    tf_listener->transform_point("/base_link", target_odom, baserel_target);
    HomPoint target_trans(baserel_target.x(),baserel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from odom to base_link");
    e.print_trace();
  }
  return res;
}


HomPoint
ColliThread::transform_base_to_odom(HomPoint point)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_base(tf::Point(point.x(),point.y(),0.),fawkes::Time(0, 0), "/base_link");
    tf::Stamped<tf::Point> odomrel_target;
    tf_listener->transform_point("/odom", target_base, odomrel_target);
    HomPoint target_trans(odomrel_target.x(),-odomrel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from baselink to odometry");
    e.print_trace();
  }
  return res;
}


float
ColliThread::GetMotorTranslation(float vtrans, float vori)
{
  float m_vx = vtrans * cos(vori);
  if (  m_vx > 0 )
    return vtrans;
  else
    return -vtrans;
}


float
ColliThread::GetMotorOri(float odom_ori)
{
  return normalize_mirror_rad(odom_ori);
}
