//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$   */
/*                                                                      */
/* Description: This is an abstract drive module interface of Colli-A*  */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the select-drive-mode module.                          */
/*       Call class for all other drive modes.                          */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */



#ifndef _COLLI_SELECT_DRIVE_MODE_CPP_
#define _COLLI_SELECT_DRIVE_MODE_CPP_



#include "select_drive_mode.h"

// INCLUDE HERE YOUR DRIVE MODES!!!
#include "stop_drive_mode.h"
#include "slow_forward_drive_mode.h"
#include "slow_backward_drive_mode.h"
#include "slow_biward_drive_mode.h"
#include "medium_forward_drive_mode.h"
#include "medium_backward_drive_mode.h"
#include "medium_biward_drive_mode.h"
#include "fast_forward_drive_mode.h"
#include "fast_backward_drive_mode.h"
#include "fast_biward_drive_mode.h"
#include "escape_drive_mode.h"

// YOUR CHANGES SHOULD END HERE!!!



using namespace std;


/*CSelectDriveMode::CSelectDriveMode( MotorControl* motor,
				    Laser* laser,
				    bbClients::Colli_Target_Client* target )*/
CSelectDriveMode::CSelectDriveMode(MotorInterface* motor, Laser* laser, NavigatorInterface* target, Logger* logger, Configuration *config)
{
  loggerSelect = logger;
  //BB_DBG(4) << "CSelectDriveMode(Constructor): Entering" << endl;
  loggerSelect->log_info("CSelectDriveMode","CSelectDriveMode(Constructor): Entering\n");
  m_EscapeFlag   = 0;       // no escaping at the beginning
  m_pMotor       = motor;
  m_pLaser       = laser;
  m_pColliTarget = target;
  m_vDriveModeList.clear();

  //BB_DBG(3) << "Creating Drive Mode Objects" << endl;
  loggerSelect->log_info("CSelectDriveMode","Creating Drive Mode Objects\n"); 

  // ============================
  // APPEND YOUR DRIVE MODE HERE!

  // MISC MODES
  // stop drive mode
  m_vDriveModeList.push_back( (CAbstractDriveMode *)new CStopDriveModule( logger, config ) );

  // and here an example of using extra data, e.g. the laser for escape...
  // escape drive mode
  m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapeDriveModule( logger, config, laser ) );
//

  // SLOW MODES
  // slow forward drive mode (have to remember for biward driving!
  CSlowForwardDriveModule* slow_forward = new CSlowForwardDriveModule( logger, config );
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_forward );

  // slow backward drive mode (have to remember for biward driving!
  CSlowBackwardDriveModule* slow_backward = new CSlowBackwardDriveModule( logger, config );
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_backward );

  // slow biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CSlowBiwardDriveModule( logger, config, slow_forward, 
										 slow_backward ) );

  // MEDIUM MODES
  // medium forward drive mode (have to remember for biward driving!
  CMediumForwardDriveModule* medium_forward = new CMediumForwardDriveModule( logger, config );
  m_vDriveModeList.push_back( (CAbstractDriveMode *) medium_forward );
  
  // medium backward drive mode (have to remember for biward driving!
  CMediumBackwardDriveModule* medium_backward = new CMediumBackwardDriveModule( logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) medium_backward );

  // medium biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CMediumBiwardDriveModule( logger, config, medium_forward, 
 										   medium_backward ) );

  // FAST MODES
  // fast forward drive mode (have to remember for biward driving!
  CFastForwardDriveModule* fast_forward = new CFastForwardDriveModule( logger, config );
  m_vDriveModeList.push_back( (CAbstractDriveMode *) fast_forward );

  // fast backward drive mode (have to remember for biward driving!
  CFastBackwardDriveModule* fast_backward = new CFastBackwardDriveModule( logger, config );
  m_vDriveModeList.push_back( (CAbstractDriveMode *) fast_backward );

  // fast biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CFastBiwardDriveModule( logger, config, fast_forward, 
										 fast_backward ) );
  
  // YOUR CHANGES SHOULD END HERE!
  // =============================
  
  //BB_DBG(4) << "CSelectDriveMode(Constructor): Exiting" << endl;
  loggerSelect->log_info("CSelectDriveMode","CSelectDriveMode(Constructor): Exiting\n");
}


CSelectDriveMode::~CSelectDriveMode()
{
  //BB_DBG(4) << "CSelectDriveMode(Destructor): Entering" << endl;
  loggerSelect->log_info("CSelectDriveMode","CSelectDriveMode(Destructor): Entering\n");
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ )
    delete m_vDriveModeList[i];
  //BB_DBG(4) << "CSelectDriveMode(Destructor): Exiting" << endl;
  loggerSelect->log_info("CSelectDriveMode","CSelectDriveMode(Destructor): Exiting\n");
}


void CSelectDriveMode::SetLocalTarget( float localTargetX, float localTargetY )
{
  m_LocalTargetX = localTargetX;
  m_LocalTargetY = localTargetY;
}


void CSelectDriveMode::SetLocalTrajec( float localTrajecX, float localTrajecY )
{
  m_LocalTrajecX = localTrajecX;
  m_LocalTrajecY = localTrajecY;
}


float CSelectDriveMode::GetProposedTranslation()
{
  return m_ProposedTranslation;
}


float CSelectDriveMode::GetProposedRotation()
{
  return m_ProposedRotation;
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/*                               U P D A T E                                      */
/* ****************************************************************************** */
/* ****************************************************************************** */
/*void CSelectDriveMode::Update( bool escape )
{
}*/

void CSelectDriveMode::Update( bool escape )
{
  CAbstractDriveMode * m_pDriveMode = 0;
  m_ProposedTranslation = 0.0;
  m_ProposedRotation = 0.0;

  // choose the correct drive mode!
  //ColliModes desiredMode = MovingNotAllowed;
  //ColliModes desiredMode = SlowForward;
  //ColliModes desiredMode = ModerateAllowBackward;
  ColliModes desiredMode  = (ColliModes)(m_pColliTarget->GetColliMode());
  //loggerSelect->log_info("select drive","colli mode is: %s",desiredMode);
/*  if ( escape == true )
    {
      if ( m_EscapeFlag == 0 && 
	   m_pMotor->GetMotorDesiredTranslation() != 0 &&
	   m_pMotor->GetMotorDesiredRotation() != 0 )
	{
	  desiredMode = MovingNotAllowed; // we have not yet stopped!
	}
      else
	{
	  m_EscapeFlag = 1;               // we have stopped recently, so do escape!
	  desiredMode = ESCAPE;
	}
    }
  else
    {
      m_EscapeFlag = 0;
      desiredMode  = (ColliModes)(m_pColliTarget->GetColliMode());
    }
*/ 
  if ( escape == true )
    {
      if ( m_EscapeFlag == 0 && 
           m_pMotor->vx() != 0 &&
           m_pMotor->omega() != 0 )
        {
          desiredMode = MovingNotAllowed; // we have not yet stopped!
        }
     else
        {
          m_EscapeFlag = 1;               // we have stopped recently, so do escape!
          desiredMode = ESCAPE;
        }
    }
  else
    {
      m_EscapeFlag = 0;
      //desiredMode  = (ColliModes)(m_pColliTarget->GetColliMode());
      // TODO desiredMode  = (ColliModes)(m_pColliTarget->flags()); // ** NOT SURE
    }

  // now search this specific drive mode in the list
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ )
    {
      // error checking
      if ( m_vDriveModeList[i]->GetDriveModeName() == desiredMode &&
	   m_pDriveMode != 0 )
	{
//	  BB_DBG(0) << "Error while selecting drive mode. There is more than one mode "
//		    << "with the same name!!!" << endl << "Stopping!" << endl;
          loggerSelect->log_info("CSelectDriveMode", "Error while selecting drive mode. There is more than one mode with the same name!!!\nStopping!\n");

	  m_pDriveMode = 0;
	  break;
	}

      // drive mode checking
      if ( m_vDriveModeList[i]->GetDriveModeName() == desiredMode &&
	   m_pDriveMode == 0 )
	{
	  m_pDriveMode = m_vDriveModeList[i];
	}
    }


  if ( m_pDriveMode == 0 )  // invalid pointer
    {
      //BB_DBG(0) << "INVALID DRIVE MODE POINTER, stopping!" << endl;
      loggerSelect->log_info("CSelectDriveMode","INVALID DRIVE MODE POINTER, stopping!\n");
      m_ProposedTranslation = 0.0;
      m_ProposedRotation = 0.0;
    }
  else  // valid drive mode!
    {
      // set the values for the drive mode
// **      m_pDriveMode->SetCurrentRoboPos( m_pMotor->GetCurrentX(), m_pMotor->GetCurrentY(),
// **				       m_pMotor->GetCurrentOri() );
      m_pDriveMode->SetCurrentRoboPos( m_pMotor->odometry_position_x(), m_pMotor->odometry_position_y(),
                                       m_pMotor->odometry_orientation() );


// **      m_pDriveMode->SetCurrentRoboSpeed( m_pMotor->GetMotorDesiredTranslation(),
// **					 m_pMotor->GetMotorDesiredRotation() );
      m_pDriveMode->SetCurrentRoboSpeed( m_pMotor->vx(),
                                         m_pMotor->omega() );


// **      m_pDriveMode->SetCurrentTarget( m_pColliTarget->GetTargetX(), m_pColliTarget->GetTargetY(), 
// **				      m_pColliTarget->GetTargetOri() );
      m_pDriveMode->SetCurrentTarget( m_pColliTarget->dest_x(), m_pColliTarget->dest_y(),
                                      m_pColliTarget->dest_ori() );


      m_pDriveMode->SetLocalTarget( m_LocalTargetX, m_LocalTargetY );
      m_pDriveMode->SetLocalTrajec( m_LocalTrajecX, m_LocalTrajecY );
// **      m_pDriveMode->SetCurrentColliMode( m_pColliTarget->OrientAtTarget(), m_pColliTarget->StopOnTarget() );
      m_pDriveMode->SetCurrentColliMode( m_pColliTarget->dest_ori(), !m_pColliTarget->is_escaping_enabled()  ); // ** NOT SURE

      
      // update the drive mode
      m_pDriveMode->Update();
      
      // get the values from the drive mode
      m_ProposedTranslation = m_pDriveMode->GetProposedTranslation();
      m_ProposedRotation    = m_pDriveMode->GetProposedRotation();

      // recheck with targetobj maximum settings
/* **
      if ( m_pColliTarget->GetMaximumTranslation() != 0.0 )
	  if ( fabs( m_ProposedTranslation ) > fabs( m_pColliTarget->GetMaximumTranslation() ) )
	      if ( m_ProposedTranslation > 0.0 )
		  m_ProposedTranslation = m_pColliTarget->GetMaximumTranslation();
	      else
		  m_ProposedTranslation = -m_pColliTarget->GetMaximumTranslation();
      
      if ( m_pColliTarget->GetMaximumRotation() != 0.0 )
	  if ( fabs( m_ProposedRotation ) > fabs( m_pColliTarget->GetMaximumRotation() ) )
	      if ( m_ProposedRotation > 0.0 )
		  m_ProposedRotation = m_pColliTarget->GetMaximumRotation();
	      else
		  m_ProposedRotation = -m_pColliTarget->GetMaximumRotation(); ** */
      if ( m_pColliTarget->max_velocity() != 0.0 )
      {
          if ( fabs( m_ProposedTranslation ) > fabs( m_pColliTarget->max_velocity() ) )
          {
              if ( m_ProposedTranslation > 0.0 )
              {
                  m_ProposedTranslation = m_pColliTarget->max_velocity();
              }
              else
              {
                  m_ProposedTranslation = -m_pColliTarget->max_velocity();
              }
          }
      }
      if ( m_pColliTarget->dest_ori() != 0.0 )
      {
          if ( fabs( m_ProposedRotation ) > fabs( m_pColliTarget->dest_ori() ) )
          {
              if ( m_ProposedRotation > 0.0 )
              {
                  m_ProposedRotation = m_pColliTarget->dest_ori();
              }
              else
              {
                  m_ProposedRotation = -m_pColliTarget->dest_ori();
              }
          }
      }

    }
}



#endif
