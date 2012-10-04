#include "colli_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
#  include "visualization_thread_base.h"
#endif
ColliThread::ColliThread()
  : Thread("ColliThread", Thread::OPMODE_CONTINUOUS)
{
 #ifdef HAVE_VISUAL_DEBUGGING
  visthread_ = NULL;
 #endif

}
//--------------------------------------------------------------
ColliThread::~ColliThread()
{
}
//-------------------------------------------------------------
void ColliThread::init()
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
    //m_ColliFrequency = (int)(1000.0/(float)config->get_int( "/plugins/colli/Colli_FREQUENCY" ));
    m_ColliFrequency = (int)(100000.0/(float)config->get_int( "/plugins/colli/Colli_FREQUENCY" ));
    //cout << "Colli_FREQUENCY " <<m_ColliFrequency << endl;
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

  for ( unsigned int i = 0; i < 10; i++ )
    m_oldAnglesToTarget.push_back( 0.0 );

  srand( time( NULL ) );

  logger->log_info(name(),"COLLI (Constructor): Entering initialization ...\n");
  
  RegisterAtBlackboard();
  InitializeModules();

//  SetTime( m_ColliFrequency );

 // m_oldTargetX   = m_pColliTargetObj->GetTargetX();
  m_oldTargetX   = m_pColliTargetObj->dest_x();
 // m_oldTargetY   = m_pColliTargetObj->GetTargetY();
  m_oldTargetY   = m_pColliTargetObj->dest_y();
 // m_oldTargetOri = m_pColliTargetObj->GetTargetOri();
  m_oldTargetOri = m_pColliTargetObj->dest_ori();

  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  escape_count = 0;

  // Initialize the alive stuff
  /*BBAlive( "laser", 4 );
  BBAlive( "sim_robot", 4 );
  BBAlive( "laser_obstacles", 4);*/

  logger->log_info(name(),"COLLI (Constructor): Initialization done.\n");


}
//-------------------------------------------------------------
void ColliThread::finalize()
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
  logger->log_info(name(),"COLLI (Destructor): Destructing done.\n");

}
//------------------------------------------------------------------
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
//--------------------------------------------------------------------------------------------
 #ifdef HAVE_VISUAL_DEBUGGING
void ColliThread::visualize_cells()
{
  vector<HomPoint > cells;
  vector<HomPoint > laser_points;
  //logger->log_info(name(),"grid size: %d,%d\n",m_pLaserOccGrid->getWidth(),m_pLaserOccGrid->getHeight());
  for ( int gridY = 0; gridY < m_pLaserOccGrid->getHeight(); gridY++ )
  {
    for ( int gridX = 0; gridX < m_pLaserOccGrid->getWidth(); gridX++ )
    {
      if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_OCCUPIED_ )
      //if ( m_pLaserOccGrid->getProb( gridX, gridY ) == _COLLI_CELL_NEAR_ )
      {
        HomPoint p(gridX,gridY);
        cells.push_back(p); 
      }
    }
  }
  //visthread_->visualize("/bese_laser",cells);
  for( int i = 0; i < m_pLaser->GetNumberOfReadings(); i++ )
  {
    float posx = m_pLaser->GetReadingPosX(i);
    float posy = m_pLaser->GetReadingPosY(i);
    HomPoint plaser(posx,posy);
    laser_points.push_back(plaser);
  }
 //cout << "number of readings "<<m_pLaser->GetNumberOfReadings() << endl;
 //logger->log_info(name(),"cell size is: %d,%d", m_pLaserOccGrid->getCellWidth(),m_pLaserOccGrid->getCellHeight());
 std::vector< HomPoint > plan = m_pSearch->GetPlan();
 HomPoint target(m_pColliTargetObj->dest_x(),m_pColliTargetObj->dest_y());
 
 //logger->log_info(name(),"current velocity is: %f,%f",m_pMopoObj->vx(),m_pMopoObj->omega());
// float m_des_x = m_pMopoObj->vx() * cos(m_pMopoObj->omega());
// float m_des_y = m_pMopoObj->vx() * sin(m_pMopoObj->omega());
 float m_des_x = m_ProposedTranslation * cos(m_ProposedRotation);
 float m_des_y = m_ProposedTranslation * sin(m_ProposedRotation);

 HomPoint motor_des(m_des_x,m_des_y);
 /*float m_x = mopo_obj->vx() * cos(mopo_obj->omega());
 float m_y = mopo_obj->vx() * sin(mopo_obj->omega());*/
 float m_x = m_pMopoObj->vx() * cos(m_pMopoObj->omega());
 float m_y = m_pMopoObj->vx() * sin(m_pMopoObj->omega());
 //logger->log_info(name(),"motor remote velocity is: %f", mopo_obj->vx());
 HomPoint motor_real(m_x,m_y);
 //logger->log_info(name(),"next change must be: %f,%f",m_x,m_y);
 visthread_->visualize("/base_link",cells,m_RoboGridPos,m_LaserGridPos,laser_points,plan,motor_des,
                       m_pLaserOccGrid->getCellWidth(),m_pLaserOccGrid->getCellHeight(),target,
                       m_OccGridWidth,m_OccGridHeight,motor_real);
}
#endif

//--------------------------------------------------------------------------------------------
void ColliThread::loop()
{
  #ifdef HAVE_VISUAL_DEBUGGING
  visualize_cells();
  #endif
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
      //m_pColliDataObj->SetFinal( false );
      m_pColliDataObj->set_final( false );
     // m_pColliDataObj->UpdateBB();
      //BBOperate();
      m_pColliDataObj->write();
      escape_count = 0;
      return;
    }


  //if ( m_pColliTargetObj->GetColliMode() == (int)(MovingNotAllowed) )
  //if ( m_pColliTargetObj->flags() == (int)(MovingNotAllowed) )
  if ((int) m_pColliTargetObj->error_code())
    {
      //BB_DBG(1) << "Moving is not allowed!" << endl << endl;
      logger->log_error(name(),"Moving is not allowed!\n\n");
      m_pMotorInstruct->Drive( 0.0, 0.0 );
      //m_pColliDataObj->SetFinal( true );
      //m_pColliDataObj->UpdateBB();
      //BBOperate();
      m_pColliDataObj->set_final(true);
      m_pColliDataObj->write();
      escape_count = 0;
      return;
    }


  // Do only drive, if there is a new (first) target
  /*if ( ( m_oldTargetX   == m_pColliTargetObj->GetTargetX() ) &&
       ( m_oldTargetY   == m_pColliTargetObj->GetTargetY() ) &&
       ( m_oldTargetOri == m_pColliTargetObj->GetTargetOri() ) )
    {*/
  if ( ( m_oldTargetX   == m_pColliTargetObj->dest_x() ) &&
       ( m_oldTargetY   == m_pColliTargetObj->dest_y() ) &&
       ( m_oldTargetOri == m_pColliTargetObj->dest_ori() ) )
    {
      m_oldAnglesToTarget.clear();
      for ( unsigned int i = 0; i < 10; i++ )
        m_oldAnglesToTarget.push_back( 0.0 );

      m_ProposedTranslation = 0.0;
      m_ProposedRotation    = 0.0;
      //m_pColliDataObj->SetFinal( true );
      m_pColliDataObj->set_final( true );
      m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

      escape_count = 0;
      // Send motor and colli data away.
     // m_pColliDataObj->UpdateBB();
     // BBOperate();
      m_pColliDataObj->write();
      return;
    }
  else
    {
      /*m_oldTargetX   = m_pColliTargetObj->GetTargetX()   + 1000.0;
      m_oldTargetY   = m_pColliTargetObj->GetTargetY()   + 1000.0;
      m_oldTargetOri = m_pColliTargetObj->GetTargetOri() + 1.0;*/
      m_oldTargetX   = m_pColliTargetObj->dest_x()   + 1000.0;
      m_oldTargetY   = m_pColliTargetObj->dest_y()   + 1000.0;
      m_oldTargetOri = m_pColliTargetObj->dest_ori() + 1.0;

    }

  // Update state machine
  UpdateColliStateMachine();

  // nothing is to do
  if (m_ColliStatus == NothingToDo)
    {
      m_pLaserOccGrid->ResetOld();
      m_ProposedTranslation = 0.0;
      m_ProposedRotation    = 0.0;
      //m_pColliDataObj->SetFinal( true );
      m_pColliDataObj->set_final( true );

     /* m_oldTargetX   = m_pColliTargetObj->GetTargetX();
      m_oldTargetY   = m_pColliTargetObj->GetTargetY();
      m_oldTargetOri = m_pColliTargetObj->GetTargetOri();*/
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
      //m_pColliDataObj->SetFinal( false );
      m_pColliDataObj->set_final( false );
    //  m_pColliDataObj->write();
      /* TODO if ( m_pMopoObj->GetAllowMove() == false )
      {
        m_pMopoObj->RecoverEmergencyStop();
        BBOperate();
      }*/
      if ( m_pMopoObj->motor_state () == m_pMopoObj->MOTOR_DISABLED )
      {
        m_pMotorInstruct->Drive( 0.0, 0.0 ); 
      }

      // Check, if one of our positions (robo-, laser-gridpos is not valid) => Danger!
      if ( CheckEscape() == true || escape_count > 0 )
        {

         /* if ( m_pMotorInstruct->GetMotorDesiredTranslation() == 0.0 &&
               m_pMotorInstruct->GetMotorDesiredRotation() == 0.0 )
            {
              m_pLaserOccGrid->ResetOld();
            }*/
          if ( m_pMopoObj->vx() == 0.0 &&
               m_pMopoObj->omega() == 0.0 )
            {
              m_pLaserOccGrid->ResetOld();
            }

          // ueber denken und testen
          //if (m_pColliTargetObj->EscapeAllowed() == true)
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
                  //BB_DBG(0) << "Escape: new round with " << rnd << endl;
                  logger->log_info(name(),"Escape: new round with %d\n",rnd);
                }
              //BB_DBG(0) << "Escape mode, escaping!" << endl;
              logger->log_info(name(),"Escape mode, escaping!\n");
              m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x(), m_LocalTarget.y() );
              m_pSelectDriveMode->Update( true );  // <-- this calls the ESCAPE mode!
              m_ProposedTranslation = m_pSelectDriveMode->GetProposedTranslation();
              m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();
            }
          else
            {
             // BB_DBG(1) << "Escape mode, but not allowed!" << endl;
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
//            m_ProposedRotation    = 1.5*normalize_mirror_rad( m_pColliTargetObj->GetTargetOri() -
             /* m_ProposedRotation    = 1.0*normalize_mirror_rad( m_pColliTargetObj->GetTargetOri() -
                                                                m_pMotorInstruct->GetCurrentOri() );*/
              m_ProposedRotation    = 1.0*normalize_mirror_rad( m_pColliTargetObj->dest_ori() -
                                                                m_pMopoObj->omega() );
              // but at least use 0.1 rad/s
              if ( m_ProposedRotation > 0.0 )
                m_ProposedRotation = max(  0.1, (double)m_ProposedRotation );
              else
                m_ProposedRotation = min( -0.1, (double)m_ProposedRotation );

               m_pLaserOccGrid->ResetOld();
            }
          else
            { // search for a path
               //logger->log_info(name(),"to update grid\n");
               m_pSearch->Update( (int)m_RoboGridPos.x(), (int)m_RoboGridPos.y(),
                                 (int)m_TargetGridPos.x(), (int)m_TargetGridPos.y() );
              if ( m_pSearch->UpdatedSuccessful() )
                { //   if path exists, 
                  logger->log_info(name(),"update successful");
                  m_LocalGridTarget = m_pSearch->GetLocalTarget();
                  m_LocalGridTrajec = m_pSearch->GetLocalTrajec();
               //   logger->log_info(name(),"local grid target: targetx = %f, targety = %f\n",m_LocalGridTarget.x(),m_LocalGridTarget.y());
                 // logger->log_info(name(),"local grid trajectory: trajx = %f, trajy = %f\n",m_LocalGridTrajec.x(),m_LocalGridTarget.y());
                  // coordinate transformation from grid coordinates to relative robot coordinates
                  m_LocalTarget = HomPoint( (m_LocalGridTarget.x() - m_RoboGridPos.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTarget.y() - m_RoboGridPos.y())*m_pLaserOccGrid->getCellHeight()/100.0 );

                  m_LocalTrajec = HomPoint( (m_LocalGridTrajec.x() - m_RoboGridPos.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTrajec.y() - m_RoboGridPos.y())*m_pLaserOccGrid->getCellHeight()/100.0 );

                  // call appopriate drive mode
                  m_pSelectDriveMode->SetLocalTarget( m_LocalTarget.x(), m_LocalTarget.y() );
                  m_pSelectDriveMode->SetLocalTrajec( m_LocalTrajec.x(), m_LocalTrajec.y() );
                  m_pSelectDriveMode->Update();
                  m_ProposedTranslation = m_pSelectDriveMode->GetProposedTranslation();
                  m_ProposedRotation    = m_pSelectDriveMode->GetProposedRotation();
                }
              else
                { // else stop
                  // BB_DBG(1) << "   Drive Mode: Update not successful ---> stopping!" << endl;
                  logger->log_info(name(),"   Drive Mode: Update not successful ---> stopping!\n");
                  m_LocalTarget = HomPoint( 0.0, 0.0 );
                  m_LocalTrajec = HomPoint( 0.0, 0.0 );
                  m_ProposedTranslation = 0.0;
                  m_ProposedRotation    = 0.0;
                  m_pLaserOccGrid->ResetOld();
                }
             /* m_pColliDataObj->SetWPX( m_LocalTarget.X() );
              m_pColliDataObj->SetWPY( m_LocalTarget.Y() );
              m_pColliDataObj->SetCPX( m_LocalTrajec.X() );
              m_pColliDataObj->SetCPY( m_LocalTrajec.Y() );*/
              m_pColliDataObj->set_x( m_LocalTarget.x() );
              m_pColliDataObj->set_y( m_LocalTarget.y() );
              m_pColliDataObj->set_dest_x( m_LocalTrajec.x() );
              m_pColliDataObj->set_dest_y( m_LocalTrajec.y() );
    //          m_pColliDataObj->write();
             // mcolli->draw();
            }
        }
    }
  cout << "I want to realize " << m_ProposedTranslation << ", " << m_ProposedRotation << endl;
  // Realize drive mode proposal with realization module
  m_pMotorInstruct->Drive( m_ProposedTranslation, m_ProposedRotation );

  cout << endl << endl;
 
  // Send motor and colli data away.
  //m_pColliDataObj->UpdateBB();
  //BBOperate();
  m_pColliDataObj->write();

}

//-----------------------------------------------------------
void ColliThread::RegisterAtBlackboard()
{
 /* string brutusStr = "172.16.35.32";
  char* brutus = new char[brutusStr.length()+1];
  strcpy(brutus,brutusStr.c_str());
  try {
    bb_ = new RemoteBlackBoard(brutus, 1910);
    cout << "successfully connected to brutus." << endl;
  } catch (Exception &e) {
    cout << "can not connect to brutus." << endl;
  }*/

  //m_pMopoObj = blackboard->open_for_writing<MotorInterface>("Motor Write");
  m_pMopoObj = blackboard->open_for_reading<MotorInterface>("Motor Brutus");
 // mopo_obj = blackboard->open_for_reading<MotorInterface>("Motor");
 // mopo_obj = bb_->open_for_reading<MotorInterface>("Motor");
  m_pLaserScannerObj = blackboard->open_for_reading<Laser360Interface>("Laser");
  m_pColliTargetObj = blackboard->open_for_reading<NavigatorInterface>("NavigatorTarget");
  m_pColliDataObj = blackboard->open_for_writing<NavigatorInterface>("Navigator Temp"); 


  //mopo_obj->read();
  //m_pMopoObj->write();
  m_pMopoObj->read();
  m_pLaserScannerObj->read();
  m_pColliTargetObj->read();
  m_pColliDataObj->read();
  
  m_pColliDataObj->set_final( true );
  m_pColliDataObj->write(); 
  m_pColliTargetObj->read();
  
  ninit = blackboard->open_for_writing<NavigatorInterface>("NavigatorTarget");
  
 /* ninit = blackboard->open_for_writing<NavigatorInterface>("NavigatorTarget");
  ninit->set_dest_x(2.0);
  ninit->set_dest_y(1.0);
  ninit->set_dest_ori(0.0);
  //ninit->set_security_distance(0.05);
//  ninit->set_colliMode(NavigatorInterface::ModerateAllowBackward);
  ninit->write(); */
}
//--------------------------------------------------------------------------
/// Initialize all modules used by the Colli

void ColliThread::InitializeModules()
{
 
  // FIRST(!): the laserinterface (uses the laserscanner)
  m_pLaser = new Laser( m_pLaserScannerObj, "" );
  m_pLaser->UpdateLaser(); 
  m_pLaser->transform(tf_listener);
  // SECOND(!): the occupancy grid (it uses the laser)

  // set the cell width and heigth to 5 cm and the grid size to 7.5 m x 7.5 m.
  // this are 750/5 x 750/5 grid cells -> (750x750)/5 = 22500 grid cells
 /* m_pLaserOccGrid->setCellWidth(  m_OccGridCellWidth );
  m_pLaserOccGrid->setWidth(  (int)((m_OccGridWidth*100)/m_pLaserOccGrid->getCellWidth()) );
  m_pLaserOccGrid->setCellHeight( m_OccGridCellHeight );
  m_pLaserOccGrid->setHeight( (int)((m_OccGridHeight*100)/m_pLaserOccGrid->getCellHeight()) );*/
  m_pLaserOccGrid = new CLaserOccupancyGrid( logger, config, m_pLaser, (int) ((m_OccGridWidth*100)/m_OccGridCellWidth),
                                            (int)((m_OccGridHeight*100)/m_OccGridCellHeight), 
                                            m_OccGridCellWidth, m_OccGridCellHeight);
  // THIRD(!): the search component (it uses the occ grid (without the laser)
  m_pSearch = new CSearch( logger, config, m_pLaserOccGrid );

  // BEFORE DRIVE MODE: the motorinstruction set
  m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( m_pMopoObj, m_ColliFrequency, logger, config );
  //m_pMotorInstruct->SetRecoverEmergencyStop();


  // AFTER MOTOR INSTRUCT: the motor propose values object
  //m_pSelectDriveMode = new CSelectDriveMode( m_pMotorInstruct, m_pLaser, m_pColliTargetObj, logger );
  m_pSelectDriveMode = new CSelectDriveMode( m_pMopoObj, m_pLaser, m_pColliTargetObj, logger, config ); 
 
  // Initialization of colli state machine:
  // Currently nothing is to accomplish
  m_ColliStatus  = NothingToDo;
  //m_oldTargetX   = m_pColliTargetObj->GetTargetX();
  m_oldTargetX   = m_pColliTargetObj->dest_x();
  //m_oldTargetY   = m_pColliTargetObj->GetTargetY();
  m_oldTargetY   = m_pColliTargetObj->dest_y();
  //m_oldTargetOri = m_pColliTargetObj->GetTargetOri();
  m_oldTargetOri = m_pColliTargetObj->dest_ori();

  //m_OldX   = m_pMotorInstruct->GetCurrentX();
  m_OldX   = m_pMopoObj->odometry_position_x();
  //m_OldY   = m_pMotorInstruct->GetCurrentY();
  m_OldY   = m_pMopoObj->odometry_position_y();
  //m_OldOri = m_pMotorInstruct->GetCurrentOri();
  m_OldOri = m_pMopoObj->odometry_orientation();

}
//-------------------------------------------------------------
/// Get the newest values from the blackboard
void ColliThread::UpdateBB()
{
  //m_pLaserScannerObj->Update();
  m_pLaserScannerObj->read();
  /*mopo_obj->read();
  m_pMopoObj->set_odometry_position_x(mopo_obj->odometry_position_x());
  m_pMopoObj->set_odometry_position_y(mopo_obj->odometry_position_y());
  m_pMopoObj->set_odometry_orientation(mopo_obj->odometry_orientation());
  m_pMopoObj->set_vx(mopo_obj->vx());  
  m_pMopoObj->set_omega(mopo_obj->omega());
  m_pMopoObj->write();*/
  

  m_pMopoObj->read();

  //while((! ninit->msgq_empty()))
  if((! ninit->msgq_empty()))
  {
    if( ninit->msgq_first_is<NavigatorInterface::SetDriveModeMessage>() )
    {
      NavigatorInterface::SetDriveModeMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_drive_mode(msgTmp->drive_mode());
      ninit->write();
      ninit->msgq_pop();
    }
    else if( ninit->msgq_first_is<NavigatorInterface::ObstacleMessage>() )
    {
      NavigatorInterface::ObstacleMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_dest_x(msgTmp->x());
      ninit->set_dest_y(msgTmp->y());
      ninit->set_dest_ori(0.0);
      ninit->write();
      ninit->msgq_pop();      
    }  
  }
 
  //m_pColliTargetObj->Update();
  m_pColliTargetObj->read();
  //m_pColliDataObj->Update();
  m_pColliDataObj->read();
}
//--------------------------------------------------------------------
void ColliThread::UpdateColliStateMachine()
{
  // initialize
  m_ColliStatus = NothingToDo;

  //float curPosX = m_pMotorInstruct->GetCurrentX();
  float curPosX = m_pMopoObj->odometry_position_x ();
  //float curPosY = m_pMotorInstruct->GetCurrentY();
  float curPosY = m_pMopoObj->odometry_position_y ();
  //float curPosO = m_pMotorInstruct->GetCurrentOri();
  float curPosO = m_pMopoObj->odometry_orientation ();

  //float targetX = m_pColliTargetObj->GetTargetX();
  float targetX = m_pColliTargetObj->dest_x();
  //float targetY = m_pColliTargetObj->GetTargetY();
  float targetY = m_pColliTargetObj->dest_y();
  //float targetO = m_pColliTargetObj->GetTargetOri();
  float targetO = m_pColliTargetObj->dest_ori();

  //bool  orient = m_pColliTargetObj->OrientAtTarget();
  bool  orient = m_pColliTargetObj->dest_ori();
  // Real driving....
  if ( ( orient == true ) &&
       ( pow( ( curPosX - targetX ),2) + pow(( curPosY - targetY ),2) >= pow((2.1),2 ) ))
    {
      //float ori = m_pColliTargetObj->GetTargetOri();
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
  else if ( pow( (curPosX - targetX ),2) + pow(( curPosY - targetY ),2) > pow((0.15),2) )  // soll im navigator wegen intercept parametrisierbar sein
    {
      m_TargetPointX = targetX;
      m_TargetPointY = targetY;
      m_ColliStatus = DriveToTarget;
      logger->log_info(name(),"colli state: drive to target\n");
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

//--------------------------------------------------------------------------------------------------------------------
/// Calculate all information out of the updated blackboard data
//  m_RoboGridPos, m_LaserGridPos, m_TargetGridPos have to be updated!
//  the targetPointX and targetPointY were calculated in the collis state machine!
void ColliThread::UpdateOwnModules()
{
  float motor_distance = 19.4;

  if ( m_RobocupMode == 1 )  // Robocup mode
    {
      // set the cell size according to the current speed
      m_pLaserOccGrid->setCellWidth( (int)m_OccGridCellWidth );
      m_pLaserOccGrid->setCellHeight( (int)m_OccGridCellHeight );
    }
  else
    {
      // set the cell size according to the current speed
      /*m_pLaserOccGrid->setCellWidth(  (int)max( (int)m_OccGridCellWidth,
                                                (int)(5*fabs(m_pMotorInstruct->GetMotorDesiredTranslation())+3) ) );
      m_pLaserOccGrid->setCellHeight( (int)max( (int)m_OccGridCellHeight,
                                                (int)(5*fabs(m_pMotorInstruct->GetMotorDesiredTranslation())+3) ) );*/
      m_pLaserOccGrid->setCellWidth(  (int)max( (int)m_OccGridCellWidth,
                                                (int)(5*fabs(m_pMopoObj->vx())+3) ) );
      m_pLaserOccGrid->setCellHeight( (int)max( (int)m_OccGridCellHeight,
                                                (int)(5*fabs(m_pMopoObj->vx())+3) ) );

    }
  
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
  //laserpos_x -= (int)( m_pMotorInstruct->GetMotorDesiredTranslation()*m_pLaserOccGrid->getWidth() / (2*3.0) );
  laserpos_x -= (int)( m_pMopoObj->vx()*m_pLaserOccGrid->getWidth() / (2*3.0) );
  laserpos_x  = max ( laserpos_x, 10 );
  laserpos_x  = min ( laserpos_x, (int)(m_pLaserOccGrid->getWidth()-10) );
  int robopos_x = laserpos_x + (int)(motor_distance/m_pLaserOccGrid->getCellWidth());
  int robopos_y = laserpos_y;
  
  // coordinate transformation for target point
  //float aX = m_TargetPointX - m_pMotorInstruct->GetCurrentX();
  float aX = m_TargetPointX - m_pMopoObj->odometry_position_x ();
  //float aY = m_TargetPointY - m_pMotorInstruct->GetCurrentY();
  float aY = m_TargetPointY - m_pMopoObj->odometry_position_y ();
  //float targetContX = ( aX*cos( m_pMotorInstruct->GetCurrentOri() ) + aY*sin( m_pMotorInstruct->GetCurrentOri() ) );
  float targetContX = ( aX*cos( m_pMopoObj->odometry_orientation ()  ) + aY*sin( m_pMopoObj->odometry_orientation () ) );
  //float targetContY = (-aX*sin( m_pMotorInstruct->GetCurrentOri() ) + aY*cos( m_pMotorInstruct->GetCurrentOri() ) );
  float targetContY = (-aX*sin( m_pMopoObj->odometry_orientation () ) + aY*cos( m_pMopoObj->odometry_orientation () ) );

  // calculation, where in the grid the target is, thats relative to the motorpos, so add it ;-)
  float targetGridX = (int)( (targetContX * 100.0) / (float)m_pLaserOccGrid->getCellWidth() );
  float targetGridY = (int)( (targetContY * 100.0) / (float)m_pLaserOccGrid->getCellHeight() );

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
  //m_pLaser->transform(tf_listener);
  // Robo increasement for robots
  float m_RoboIncrease = 0.0;

  if ( m_RobocupMode == 1 )  // Robocup mode
    {
      //if ( m_pColliTargetObj->GetSecureDistance() > 0.0 )
      if ( m_pColliTargetObj->security_distance() > 0.0 )
        {
          //m_RoboIncrease = m_pColliTargetObj->GetSecureDistance();
          m_RoboIncrease = m_pColliTargetObj->security_distance();
          //BB_DBG(2) << "BBClient( UpdateOwnModules ): Setting EXTERN Robot secure distance = "
                    //<< m_RoboIncrease << ". ATTENTION TO THE ROBOT!!!!" << endl;
          logger->log_info(name(),"BBClient( UpdateOwnModules ): Setting EXTERN Robot secure distance = %f . ATTENTION TO THE ROBOT!!!!\n", m_RoboIncrease);
        }
      else
        {
          m_RoboIncrease = 0.0;
        }
    }
  else  // no robocup mode
    {
      //if ( m_pColliTargetObj->GetSecureDistance() > 0.0 )
      if ( m_pColliTargetObj->security_distance() > 0.0 )
        {
          //m_RoboIncrease = m_pColliTargetObj->GetSecureDistance();
          m_RoboIncrease = m_pColliTargetObj->security_distance();
          /*BB_DBG(1) << "BBClient( UpdateOwnModules ): Setting EXTERN Robot secure distance = "
                    << m_RoboIncrease << ". ATTENTION TO THE ROBOT!!!!" << endl;*/
          logger->log_info(name(),"BBClient( UpdateOwnModules ): Setting EXTERN Robot secure distance = %f ATTENTION TO THE ROBOT!!!!\n"
                          , m_RoboIncrease);
        }
      else
        {
          //      float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.35);
          //      float rotinc   = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentRotation()/3.5 )-0.4);
          //float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.7); 
          float transinc = max(0.0,fabs( m_pMopoObj->vx()/2.0 )-0.7);
          float rotinc   = max(0.0,fabs( m_pMopoObj->omega()/3.5 )-0.7);

          m_RoboIncrease = max( transinc, rotinc );
          m_RoboIncrease = min( m_MaximumRoboIncrease, m_RoboIncrease );
        }
    }

  //float xdiff = m_pMotorInstruct->GetCurrentX() - m_OldX;
  float xdiff = m_pMopoObj->odometry_position_x() - m_OldX;
  //m_OldX = m_pMotorInstruct->GetCurrentX();
  m_OldX = m_pMopoObj->odometry_position_x();
  //float ydiff = m_pMotorInstruct->GetCurrentY() - m_OldY;
  float ydiff = m_pMopoObj->odometry_position_y() - m_OldY;
  //m_OldY = m_pMotorInstruct->GetCurrentY();
  m_OldY = m_pMopoObj->odometry_position_y();
  //float oridiff = normalize_mirror_rad( m_pMotorInstruct->GetCurrentOri() - m_OldOri );
  float oridiff = normalize_mirror_rad( m_pMopoObj->odometry_orientation() - m_OldOri );
  //m_OldOri = m_pMotorInstruct->GetCurrentOri();
  m_OldOri = m_pMopoObj->odometry_orientation();

  /*float relxdiff =  xdiff *  cos( m_pMotorInstruct->GetCurrentOri() ) +
                    ydiff *  sin( m_pMotorInstruct->GetCurrentOri() );*/
  float relxdiff =  xdiff *  cos( m_pMopoObj->odometry_orientation() ) +
                    ydiff *  sin( m_pMopoObj->odometry_orientation() );
  /*float relydiff =  xdiff * -sin( m_pMotorInstruct->GetCurrentOri() ) +
                    ydiff *  cos( m_pMotorInstruct->GetCurrentOri() );*/
  float relydiff =  xdiff * -sin( m_pMopoObj->odometry_orientation() ) +
                    ydiff *  cos( m_pMopoObj->odometry_orientation() );

  // update the occgrid...
  /*m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease,
                                  m_pMotorInstruct->GetMotorDesiredTranslation(),
                                  relxdiff, relydiff, oridiff );*/
  m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease,
                                  m_pMopoObj->vx(),
                                  relxdiff, relydiff, oridiff );
  // update the positions
  m_LaserGridPos  = HomPoint( laserpos_x, laserpos_y );
  //m_LaserGridPos.x = laserpos_x;  m_LaserGridPos.y = laserpos_y;
  m_RoboGridPos   = HomPoint( robopos_x, robopos_y );
  //m_RoboGridPos.x = robopos_x; m_RoboGridPos.y = robopos_y;
  m_TargetGridPos = HomPoint( (int)targetGridX, (int)targetGridY );
  //m_TargetGridPos.x = (int)targetGridX;  m_TargetGridPos.y = (int)targetGridY;
}
//---------------------------------------------------------------------------------------------------------------------
/// Check if we want to escape an obstacle
bool ColliThread::CheckEscape()
{
  //if ((float)m_pLaserOccGrid->getProb((int)m_RoboGridPos.X(),(int)m_RoboGridPos.Y()) == _COLLI_CELL_OCCUPIED_ )
  if ((float)m_pLaserOccGrid->getProb((int)m_RoboGridPos.x(),(int)m_RoboGridPos.y()) ==  _COLLI_CELL_OCCUPIED_ )
    {
      //logger->log_info(name(),"must be escaped\n");
      return true;
    }
  else
    {
      //logger->log_info( name(),"No escape\n");
      return false;
    }
}
//-------------------------------------------------------------------------------------------------------------------
#ifdef HAVE_VISUAL_DEBUGGING
void ColliThread::set_visualization_thread(ColliVisualizationThreadBase *visthread)
{
  visthread_ = visthread;
  if(visthread_ ) cout << "visualization thread set"<< endl;
}
#endif

