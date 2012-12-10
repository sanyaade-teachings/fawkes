#include "colli_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
#  include "visualization_thread_base.h"
//#  include "navigation_thread_base.h"
#endif
ColliThread::ColliThread()
  : Thread("ColliThread", Thread::OPMODE_CONTINUOUS)
{
 #ifdef HAVE_VISUAL_DEBUGGING
  visthread_ = NULL;
//  navthread_ = NULL;
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
  blackboard->close(motor_des);
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
  vector<HomPoint > occ_cells;
  vector<HomPoint > near_cells; 
  vector<HomPoint > far_cells;
  vector<HomPoint > middle_cells;
  vector<HomPoint > laser_points;
  vector<HomPoint > free_cells;
  //logger->log_info(name(),"grid size: %d,%d\n",m_pLaserOccGrid->getWidth(),m_pLaserOccGrid->getHeight());
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
  //CLaserOccupancyGrid* socc = m_pSearch->get_grid();
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

  //visthread_->visualize("/bese_laser",cells);
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
  //cout << endl;

 //cout << "number of readings "<<m_pLaser->GetNumberOfReadings() << endl;
  vector< HomPoint > plan = m_pSearch->GetPlan();
  int non_valid_count = 0;
  for( unsigned int i = 0; i < plan.size(); i++ )
  {
    if (m_pLaserOccGrid->getProb( plan[i].x(), plan[i].y() ) == _COLLI_CELL_OCCUPIED_)
    {
      non_valid_count++;
    }
  }
  logger->log_info(name(),"#of non valid cells in returned plan is: %d\n",non_valid_count);
  //vector< HomPoint > plan = m_vSolution;
  HomPoint target = m_TargetGridPos; 
 // HomPoint target_odom = HomPoint(m_TargetPointX,m_TargetPointY);
  HomPoint target_odom = HomPoint(m_pColliTargetObj->dest_x(),m_pColliTargetObj->dest_y());
 //logger->log_info(name(),"current velocity is: %f,%f",m_pMopoObj->vx(),m_pMopoObj->omega());
// float m_des_x = m_pMopoObj->vx() * cos(m_pMopoObj->omega());
// float m_des_y = m_pMopoObj->vx() * sin(m_pMopoObj->omega());
 float m_des_x = m_ProposedTranslation * cos(m_ProposedRotation);
 float m_des_y = -m_ProposedTranslation * sin(m_ProposedRotation);

 HomPoint motor_des_point(m_des_x,m_des_y);
 /*float m_x = mopo_obj->vx() * cos(mopo_obj->omega());
 float m_y = mopo_obj->vx() * sin(mopo_obj->omega());*/
 float m_x = m_pMopoObj->vx() * cos(m_pMopoObj->omega());
 float m_y = m_pMopoObj->vx() * sin(m_pMopoObj->omega());
 //logger->log_info(name(),"motor remote velocity is: %f", mopo_obj->vx());
 HomPoint motor_real(m_x,m_y);
 vector<HomPoint > astar_found_occ = m_pSearch->get_occ_astar_search();
 vector<HomPoint > seen_states = m_pSearch->get_astar_states();

 /*vector<HomPoint > test_occ_cells;
 vector<HomPoint > test_free_cells;
 vector< HomPoint > test_plan;
 vector<HomPoint > test_orig_laser_points;
 visualize_test(test_occ_cells,test_free_cells,test_plan,test_orig_laser_points);
 free_cells.clear();
 free_cells = test_free_cells;
 occ_cells.clear();
 occ_cells = test_occ_cells; 
 plan.clear();
 plan = test_plan;
 orig_laser_points.clear();
 orig_laser_points = test_orig_laser_points;*/
 visthread_->visualize("/base_link",occ_cells,near_cells,far_cells,middle_cells,m_RoboGridPos,m_LaserGridPos,laser_points,plan,motor_des_point,
                       m_pLaserOccGrid->getCellWidth(),m_pLaserOccGrid->getCellHeight(),target,
                       m_OccGridWidth,m_OccGridHeight,motor_real,m_LocalTarget,target_odom,orig_laser_points,search_occ_cells,astar_found_occ,free_cells,seen_states);
}
#endif
//-------------------------------------------------------------------------------------------
void ColliThread::visualize_test(vector<HomPoint > &test_occ_cells,vector<HomPoint > &test_free_cells,vector< HomPoint > &test_plan,vector<HomPoint > &test_orig_laser_points)
{
  m_pLaserScannerObjTest->read();
  m_pLaserOccGridTest->UpdateOccGrid( 65, 65, 0.,0.,0., 0.,0. );
  logger->log_info(name(),"test cellw: %d, test cellh: %d, test occw %d, test occH %d",m_pLaserOccGridTest->getCellWidth(),m_pLaserOccGridTest->getCellHeight()
                  ,m_pLaserOccGridTest->getWidth(),m_pLaserOccGridTest->getHeight());  
  for ( int gridY = 0; gridY < m_pLaserOccGridTest->getHeight(); gridY++ )
  {
    for ( int gridX = 0; gridX < m_pLaserOccGridTest->getWidth(); gridX++ )
    {
      if ( m_pLaserOccGridTest->getProb( gridX, gridY ) == _COLLI_CELL_OCCUPIED_ )
      {
        HomPoint p(gridX,gridY);
        test_occ_cells.push_back(p);
      }
      else if ( m_pLaserOccGridTest->getProb( gridX, gridY ) == _COLLI_CELL_NEAR_ )
      {
   //     cout << "cell is near: " << gridX <<":" <<gridY <<"\t";
      }
      else if ( m_pLaserOccGridTest->getProb( gridX, gridY ) == _COLLI_CELL_FAR_ )
      {
     //   cout << "cell is far: " << gridX << ":"<<gridY <<"\t";
      }
      else if ( m_pLaserOccGridTest->getProb( gridX, gridY ) == _COLLI_CELL_MIDDLE_ )
      {
       // cout << "cell is middle: " << gridX << ":"<<gridY <<"\t";
      }
      else if ( m_pLaserOccGridTest->getProb( gridX, gridY ) == _COLLI_CELL_FREE_ )
      {
       // cout << "cell is free: " << gridX << ":"<<gridY <<"\t";
        HomPoint p(gridX,gridY);
        test_free_cells.push_back(p);
      }
      //else
      //cout << "cell is not valid: " << gridX << ":"<<gridY <<"\t";
    }
  }
  //cout << endl;
  m_LaserGridPos = HomPoint(65,65);
  logger->log_info(name(),"laser pos is: %f : %f", m_LaserGridPos.x(),m_LaserGridPos.y());
  m_TargetGridPos = HomPoint(90,90);
  m_RoboGridPos = m_LaserGridPos;
  m_pSearchTest->Update(65,65,75,75,m_pLaserOccGridTest);
  test_plan = m_pSearchTest->GetPlan();

  for( unsigned int i = 0; i < m_pLaserScannerObjTest->maxlenof_distances(); i++ )
  {
    float ori = float(i) * M_PI / 180.;
    float posx = m_pLaserScannerObjTest->distances(i) * cos(ori);
    float posy = m_pLaserScannerObjTest->distances(i) * sin(ori);

    HomPoint plaser(posx,posy);
    test_orig_laser_points.push_back(plaser);
  }
  
}
//--------------------------------------------------------------------------------------------
void ColliThread::loop()
{
  #ifdef HAVE_VISUAL_DEBUGGING
  visualize_cells();
  #endif
  transform_odom();
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


  if ((int) m_pColliTargetObj->error_code())
    {
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
          if ( GetMotorTranslation(motor_des->vx(),motor_des->omega()) == 0.0 &&
               motor_des->omega() == 0.0 )
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
               //logger->log_info(name(),"to update grid\n");
               m_pSearch->Update( (int)m_RoboGridPos.x(), (int)m_RoboGridPos.y(),
                                 (int)m_TargetGridPos.x(), (int)m_TargetGridPos.y(),m_pLaserOccGrid );
              if ( m_pSearch->UpdatedSuccessful() )
                { //   if path exists, 
                  logger->log_info(name(),"update successful\n");
                  m_vSolution.clear();
                  m_LocalGridTarget = m_pSearch->GetLocalTarget();
                  m_LocalGridTrajec = m_pSearch->GetLocalTrajec();
                  logger->log_info(name(),"local grid target: targetx = %f, targety = %f\n",m_LocalGridTarget.x(),m_LocalGridTarget.y());
                 // logger->log_info(name(),"local grid trajectory: trajx = %f, trajy = %f\n",m_LocalGridTrajec.x(),m_LocalGridTarget.y());
                  // coordinate transformation from grid coordinates to relative robot coordinates
                  m_LocalTarget = HomPoint( (m_LocalGridTarget.x() - m_RoboGridPos.x())*m_pLaserOccGrid->getCellWidth()/100.0,
                                          (m_LocalGridTarget.y() - m_RoboGridPos.y())*m_pLaserOccGrid->getCellHeight()/100.0 );
                  logger->log_info(name(),"local target: targetx = %f, targety = %f\n",m_LocalTarget.x(),m_LocalTarget.y());
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
  m_tf_pub_odom = new tf::TransformPublisher(blackboard, "colli odometry");

  m_pMopoObj = blackboard->open_for_reading<MotorInterface>("Motor Brutus");
  motor_des = blackboard->open_for_writing<MotorInterface>("Motor Caesar");
  m_pLaserScannerObj = blackboard->open_for_reading<Laser360Interface>("Laser");
  m_pColliTargetObj = blackboard->open_for_reading<NavigatorInterface>("NavigatorTarget");
  m_pColliDataObj = blackboard->open_for_writing<NavigatorInterface>("Navigator Temp"); 


  m_pMopoObj->read();
  m_pLaserScannerObj->read();
  m_pColliTargetObj->read();
  m_pColliDataObj->read();
  
  m_pColliDataObj->set_final( true );
  m_pColliDataObj->write(); 
  m_pColliTargetObj->read();
  
  ninit = blackboard->open_for_writing<NavigatorInterface>("NavigatorTarget");

  m_pLaserScannerObjTest = blackboard->open_for_writing<Laser360Interface>("LaserTest");  
  init_laser();
}
//---------------------------------------------------------------------------
void ColliThread::init_laser()
{
  for( unsigned int i = 0; i < 360; i++ )
  {
    m_pLaserScannerObjTest->set_distances (i,0.98);
  }
  m_pLaserScannerObjTest->write();
  m_pLaserScannerObjTest->read();
 /* for( unsigned int id = 0; id < 360;id++ )
  {
    cout << m_pLaserScannerObjTest->distances(id) << "\t";
  }*/
}
//--------------------------------------------------------------------------
//Initialize all modules used by the Colli

void ColliThread::InitializeModules()
{
    
  // FIRST(!): the laserinterface (uses the laserscanner)
  m_pLaser = new Laser( m_pLaserScannerObj, "" );
  m_pLaser->UpdateLaser(); 
  m_pLaser->transform(tf_listener);

  m_pLaserTest = new Laser( m_pLaserScannerObjTest, "" );
  m_pLaserOccGridTest = new CLaserOccupancyGrid( logger, config, m_pLaserTest, (int) ((m_OccGridWidth*100)/m_OccGridCellWidth),
                                            (int)((m_OccGridHeight*100)/m_OccGridCellHeight),
                                            m_OccGridCellWidth, m_OccGridCellHeight);  
  m_pSearchTest = new CSearch( logger, config, m_pLaserOccGridTest );
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
 // m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( m_pMopoObj, m_ColliFrequency, logger, config );
  m_pMotorInstruct = (CBaseMotorInstruct *)new CQuadraticMotorInstruct( motor_des, m_pMopoObj,m_ColliFrequency, logger, config );
  //m_pMotorInstruct->SetRecoverEmergencyStop();


  // AFTER MOTOR INSTRUCT: the motor propose values object
  //m_pSelectDriveMode = new CSelectDriveMode( m_pMotorInstruct, m_pLaser, m_pColliTargetObj, logger );
  //m_pSelectDriveMode = new CSelectDriveMode( m_pMopoObj, m_pLaser, m_pColliTargetObj, logger, config );
  m_pSelectDriveMode = new CSelectDriveMode( motor_des, m_pLaser, m_pColliTargetObj, logger, config ); 
 
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
  m_OldOri = GetMotorOri(m_pMopoObj->odometry_orientation());

  m_Updx = m_pMopoObj->odometry_position_x();
  m_Updy = m_pMopoObj->odometry_position_y();
  m_UpdOri = GetMotorOri(m_pMopoObj->odometry_orientation());

  m_vSolution.clear();
}
//-------------------------------------------------------------
/// Get the newest values from the blackboard
void ColliThread::UpdateBB()
{
  //m_pLaserScannerObj->Update();
  m_pLaserScannerObj->read();
  
  m_pMopoObj->read();
  motor_des->set_odometry_position_x(m_pMopoObj->odometry_position_x());
  motor_des->set_odometry_position_y(m_pMopoObj->odometry_position_y());
  motor_des->set_odometry_orientation(m_pMopoObj->odometry_orientation());
  motor_des->set_motor_state(m_pMopoObj->motor_state());
  //motor_des->set_vx(m_pMopoObj->vx());
  //motor_des->set_omega(m_pMopoObj->omega());
  motor_des->write();

  motor_des->read();  
  update_navi();
 /* m_Updx = m_pMopoObj->odometry_position_x();
  m_Updy = m_pMopoObj->odometry_position_y();
  m_UpdOri = GetMotorOri(m_pMopoObj->odometry_orientation());*/
   //m_pMopoObj->read();
  //m_pColliTargetObj->Update();
  m_pColliTargetObj->read();
  //m_pColliDataObj->Update();
  m_pColliDataObj->read();
}
//---------------------------------------------------------------------------------------------
void ColliThread::update_navi()
{
  if((! ninit->msgq_empty()))
  {
    if( ninit->msgq_first_is<NavigatorInterface::CartesianGotoMessage>() )
    {
      NavigatorInterface::CartesianGotoMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      ninit->set_dest_x(msgTmp->x());
      ninit->set_dest_y(msgTmp->y());
      ninit->set_dest_ori(msgTmp->orientation());
      ninit->write();
      ninit->msgq_pop();
    }
    else if( ninit->msgq_first_is<NavigatorInterface::PolarGotoMessage>() )
    {
      NavigatorInterface::PolarGotoMessage *msgTmp = ninit->msgq_first_safe(msgTmp);
      float new_x = msgTmp->dist() * cos(msgTmp->phi());
      float new_y = msgTmp->dist() * sin(msgTmp->phi());
      new_x += m_pMopoObj->odometry_position_x();
      new_y += m_pMopoObj->odometry_position_y();
      float tx = ( new_x*cos( m_pMopoObj->odometry_orientation ()  ) + new_y*sin( m_pMopoObj->odometry_orientation () ) );
      float ty = ( new_y*cos( m_pMopoObj->odometry_orientation ()  ) - new_x*sin( m_pMopoObj->odometry_orientation () ) );
      ninit->set_dest_x(tx);
      ninit->set_dest_y(ty);
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
}
//---------------------------------------------------------------------------------------------
void ColliThread::UpdateColliStateMachine()
{
  // initialize
  m_ColliStatus = NothingToDo;

  //float curPosX = m_pMotorInstruct->GetCurrentX();
  float curPosX = m_pMopoObj->odometry_position_x ();
  //float curPosY = m_pMotorInstruct->GetCurrentY();
  float curPosY = m_pMopoObj->odometry_position_y ();
  //float curPosO = m_pMotorInstruct->GetCurrentOri();
  float curPosO = GetMotorOri(m_pMopoObj->odometry_orientation ());

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
       ( sqr( curPosX - targetX ) + sqr( curPosY - targetY ) >= sqr(2.1) ) )
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
                                                (int)(5*fabs(GetMotorTranslation(motor_des->vx(),motor_des->omega()))+3) ) );
      m_pLaserOccGrid->setCellHeight( (int)max( (int)m_OccGridCellHeight,
                                                (int)(5*fabs(GetMotorTranslation(motor_des->vx(),motor_des->omega()))+3) ) );

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
  laserpos_x -= (int)( GetMotorTranslation(motor_des->vx(),motor_des->omega())*m_pLaserOccGrid->getWidth() / (2*3.0) );
  laserpos_x  = max ( laserpos_x, 10 );
  laserpos_x  = min ( laserpos_x, (int)(m_pLaserOccGrid->getWidth()-10) );
  int robopos_x = laserpos_x + (int)(motor_distance/(float)m_pLaserOccGrid->getCellWidth());
  int robopos_y = laserpos_y;
  

  // coordinate transformation for target point
 // float aX = m_TargetPointX - m_pMotorInstruct->GetCurrentX();
  float aX = m_TargetPointX - m_pMopoObj->odometry_position_x ();
  //float aY = m_TargetPointY - m_pMotorInstruct->GetCurrentY();
  float aY = m_TargetPointY - m_pMopoObj->odometry_position_y ();
  //float targetContX = ( aX*cos( m_pMotorInstruct->GetCurrentOri() ) + aY*sin( m_pMotorInstruct->GetCurrentOri() ) );
  float targetContX = ( aX*cos( GetMotorOri(m_pMopoObj->odometry_orientation () ) ) + aY*sin( GetMotorOri(m_pMopoObj->odometry_orientation ()) ) );
  //float targetContY = (-aX*sin( m_pMotorInstruct->GetCurrentOri() ) + aY*cos( m_pMotorInstruct->GetCurrentOri() ) );
  float targetContY = (-aX*sin( GetMotorOri(m_pMopoObj->odometry_orientation ()) ) + aY*cos( GetMotorOri(m_pMopoObj->odometry_orientation ()) ) );
  
/*
  HomPoint target_base = transform_odom(HomPoint(m_TargetPointX,m_TargetPointY));
  float targetContX = target_base.x();
  float targetContY = target_base.y();
*/
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
  m_pLaser->transform(tf_listener);
  // Robo increasement for robots
  float m_RoboIncrease = 0.0;

  if ( m_RobocupMode == 1 )  // Robocup mode
    {
      //if ( m_pColliTargetObj->GetSecureDistance() > 0.0 )
      if ( m_pColliTargetObj->security_distance() > 0.0 )
        {
          //m_RoboIncrease = m_pColliTargetObj->GetSecureDistance();
          m_RoboIncrease = m_pColliTargetObj->security_distance();
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
          logger->log_info(name(),"BBClient( UpdateOwnModules ): Setting EXTERN Robot secure distance = %f ATTENTION TO THE ROBOT!!!!\n"
                          , m_RoboIncrease);
        }
      else
        {
          //      float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.35);
          //      float rotinc   = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentRotation()/3.5 )-0.4);
          //float transinc = max(0.0,fabs( m_pMotorInstruct->GetMotorCurrentTranslation()/2.0 )-0.7); 
          float transinc = max(0.0,fabs( GetMotorTranslation(m_pMopoObj->vx(),m_pMopoObj->omega())/2.0 )-0.7);
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
  m_OldOri = GetMotorOri(m_pMopoObj->odometry_orientation());

  /*float relxdiff =  xdiff *  cos( m_pMotorInstruct->GetCurrentOri() ) +
                    ydiff *  sin( m_pMotorInstruct->GetCurrentOri() );*/
  float relxdiff =  xdiff *  cos( GetMotorOri(m_pMopoObj->odometry_orientation()) ) +
                    ydiff *  sin( GetMotorOri(m_pMopoObj->odometry_orientation()) );
  /*float relydiff =  xdiff * -sin( m_pMotorInstruct->GetCurrentOri() ) +
                    ydiff *  cos( m_pMotorInstruct->GetCurrentOri() );*/
  float relydiff =  xdiff * -sin( GetMotorOri(m_pMopoObj->odometry_orientation()) ) +
                    ydiff *  cos( GetMotorOri(m_pMopoObj->odometry_orientation()) );

  // update the occgrid...
  /*m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease,
                                  m_pMotorInstruct->GetMotorDesiredTranslation(),
                                  relxdiff, relydiff, oridiff );*/
  m_pLaserOccGrid->UpdateOccGrid( laserpos_x, laserpos_y, m_RoboIncrease,GetMotorTranslation(motor_des->vx(),motor_des->omega()),relxdiff, relydiff, oridiff );
  // update the positions
  m_LaserGridPos  = HomPoint( laserpos_x, laserpos_y );
  m_RoboGridPos   = HomPoint( robopos_x, robopos_y );
  m_TargetGridPos = HomPoint( targetGridX, targetGridY );
}
//---------------------------------------------------------------------------------------------------------------------
/// Check if we want to escape an obstacle
bool ColliThread::CheckEscape()
{
  //if ((float)m_pLaserOccGrid->getProb((int)m_RoboGridPos.X(),(int)m_RoboGridPos.Y()) == _COLLI_CELL_OCCUPIED_ )
  if ((float)m_pLaserOccGrid->getProb((int)m_RoboGridPos.x(),(int)m_RoboGridPos.y()) ==  _COLLI_CELL_OCCUPIED_ )
    {
      return true;
    }
  else
    {
      return false;
    }
}
//----------------------------------------------------------------------------------------------------------------------
#ifdef HAVE_VISUAL_DEBUGGING
void ColliThread::set_visualization_thread(ColliVisualizationThreadBase *visthread)
{
  visthread_ = visthread;
  if(visthread_ ) cout << "visualization thread set"<< endl;
}
#endif
//--------------------------------------------------------------------------------------------------------------------
void ColliThread::transform_odom()
{
  tf::Quaternion o_r(-m_pMopoObj->odometry_orientation(), 0, 0);
  tf::Vector3 o_t(m_pMopoObj->odometry_position_x(),-m_pMopoObj->odometry_position_y(), 0);
  tf::Transform o_tr(o_r, o_t);
  Time *o_ts = new Time();
  o_ts = &(o_ts->stamp()); 
  fawkes::Time o_time(o_ts->get_sec(), o_ts->get_usec());
  m_tf_pub_odom->send_transform(o_tr, o_time, "/odom", "/base_link");
}
//-----------------------------------------------------------------------------------------------------------------
HomPoint ColliThread::transform_odom(HomPoint point)
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
//---------------------------------------------------------------------------------------------------------------------
void ColliThread::transform_navi()
{
  try {
    tf::Stamped<tf::Point> centroid(tf::Point(ninit->dest_x(),ninit->dest_y(),0.),
                   fawkes::Time(0, 0), "/odom");
    tf::Stamped<tf::Point> baserel_centroid;
    tf_listener->transform_point("/base_link", centroid, baserel_centroid);
    ninit->set_dest_x(baserel_centroid.x());
    ninit->set_dest_y(baserel_centroid.y());
    ninit->write();
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from odom");
    e.print_trace();
  }

}
//----------------------------------------------------------------------------------------------------------------------
float ColliThread::GetMotorTranslation(float vtrans, float vori)
{
  float m_vx = vtrans * sin(vori);
  if (  m_vx >= 0 )
    return vtrans;
  else
    return -vtrans;
}
//---------------------------------------------------------------------------------------------------------------------
float ColliThread::GetMotorOri(float odom_ori)
{
  return normalize_mirror_rad(odom_ori);
}
