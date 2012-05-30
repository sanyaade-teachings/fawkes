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
/* Description: This is escape drive module implementation of Colli-A*  */
/*                                                                      */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the escape drive module. This is called everytime,     */
/*         escaping behaviour is neccessary.                            */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_ESCAPE_DRIVE_MODE_CPP_
#define _COLLI_ESCAPE_DRIVE_MODE_CPP_


//#include <utils/utils.h>
//#include <utils/configfile/configfile.h>
#include "escape_drive_mode.h"


using namespace std;


/** Initialize your local values here.
 */
CEscapeDriveModule::CEscapeDriveModule( Logger* logger, Configuration *config, Laser* laser ) :
  CAbstractDriveMode(logger, config)
{
  loggerEscape = logger;
  //BB_DBG(4) << "CEscapeDriveModule(Constructor): Entering..." << endl;
  loggerEscape->log_info("CEscapeDriveModule","CEscapeDriveModule(Constructor): Entering...\n");
  m_DriveModeName = ESCAPE;
  m_pLaser = laser;
 /* string confFileName = "../cfg/navigation/colli.cfg";
  try 
    {
      ConfigFile * m_pConf = new ConfigFile( confFileName );
      m_MaxTranslation = m_pConf->floating( "CEscapeForwardDriveModule_MAX_TRANS" );
      m_MaxRotation    = m_pConf->floating( "CEscapeForwardDriveModule_MAX_ROT" );
      delete m_pConf;
    }
  catch (...)
    {
      cout << "***** ERROR *****: Could not open: " << confFileName 
	   << " --> ABORTING!" << endl << endl;
      exit( 0 );
    }
*/
//  confFileName = "../cfg/navigation/robot_shape.cfg";
 // m_pRoboShape = new CRoboShape_Colli( (char *)confFileName.c_str(), 2 );
   // ** m_pRoboShape = new CRoboShape_Colli( logger, config, 2) ; // ** Not implemented since roboshape can just be 1 ** //
  if(!config->exists("/plugins/colli/CEscapeDriveModule/EscapeForwardDriveModule_MAX_TRANS") )
  {
    cout << "***** ERROR *****: Could not find: EscapeForwardDriveModule_MAX_TRANS "
         << " --> ABORTING!" << endl << endl;
    return;
  }  
  else
  {
    m_MaxTranslation = config->get_float("/plugins/colli/CEscapeDriveModule/EscapeForwardDriveModule_MAX_TRANS");
  }
 
  if(!config->exists("/plugins/colli/CEscapeDriveModule/CEscapeForwardDriveModule_MAX_ROT") )
  {
    cout << "***** ERROR *****: Could not find: CEscapeForwardDriveModule_MAX_ROT "
         << " --> ABORTING!" << endl << endl;
    return;
  }

  //BB_DBG(4) << "CEscapeDriveModule(Constructor): Exiting..." << endl;
  loggerEscape->log_info("CEscapeDriveModule","CEscapeDriveModule(Constructor): Exiting...\n");
}


/** Destruct your local values here.
 */
CEscapeDriveModule::~CEscapeDriveModule()
{
  //BB_DBG(4) << "CEscapeDriveModule(Destructor): Entering..." << endl;
  //BB_DBG(4) << "CEscapeDriveModule(Destructor): Exiting..." << endl;
  loggerEscape->log_info("CEscapeDriveModule","CEscapeDriveModule(Destructor): Entering...\n");
  loggerEscape->log_info("CEscapeDriveModule","CEscapeDriveModule(Destructor): Exiting...\n");
}



/* ************************************************************************** */
/* ***********************        U P D A T E       ************************* */
/* ************************************************************************** */

/** Calculate here your desired settings. What you desire is checked afterwards to the current
 *    settings of the physical boundaries, but take care also. 
 * 
 *  How you do this is up to you, but be careful, our hardware is expensive!!!!
 *
 *  All values of the other drive modes inherited by the abstract-drive-mode are
 *    non-valid, because search did not succeed or should not have been called!
 *    So do not use them. Instead here you use the m_pLaser!
 *
 *  Afterwards filled should be:
 *
 *     m_ProposedTranslation              --> Desired Translation speed
 *     m_ProposedRotation                 --> Desired Rotation speed
 *
 *  Those values are questioned after an Update() was called.
 */
void CEscapeDriveModule::Update()
{
  // This is only called, if we recently stopped...
  //BB_DBG(0) << "CEscapeDriveModule( Update ): Calculating ESCAPING..." << endl;
  loggerEscape->log_info("EscapeDriveModule","CEscapeDriveModule( Update ): Calculating ESCAPING...\n"); 
  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  FillNormalizedReadings();
  SortNormalizedReadings();

  bool dangerFront = CheckDanger( m_vFront );
  bool dangerBack  = CheckDanger( m_vBack  );

  bool dangerLeft  = ( CheckDanger( m_vLeftFront ) || CheckDanger( m_vLeftBack ) );
  bool dangerRight = ( CheckDanger( m_vRightFront ) || CheckDanger( m_vRightBack ) );

  bool turnLeftAllowed = TurnLeftAllowed();
  bool turnRightAllowed = TurnRightAllowed();

  if (dangerFront)
    //BB_DBG(1) << "DANGER IN FRONT" << endl;
    loggerEscape->log_error("CEscapeDriveModule","DANGER IN FRONT\n");

  if (dangerBack)
    //BB_DBG(1) << "DANGER IN BACK" << endl;
    loggerEscape->log_error("CEscapeDriveModule","DANGER IN BACK\n");
  if (dangerLeft)
    //BB_DBG(1) << "DANGER IN LEFT" << endl;
    loggerEscape->log_error("CEscapeDriveModule","DANGER IN LEFT\n");
  if (dangerRight)
    //BB_DBG(1) << "DANGER IN RIGHT" << endl;
    loggerEscape->log_error("CEscapeDriveModule","DANGER IN RIGHT\n");
  if (!turnLeftAllowed)
    //BB_DBG(1) << "DANGER IF TURNING LEFT!!!" << endl;
    loggerEscape->log_error("CEscapeDriveModule","DANGER IN TURNING LEFT!!!\n");
  if (!turnRightAllowed)
    //BB_DBG(1) << "DANGER IF TURNING RIGHT!!!" << endl;
    loggerEscape->log_error("CEscapeDriveModule","DANGER IN TURNING RIGHT !!!\n");

  if ( dangerFront && dangerBack && turnRightAllowed )
    {
      m_ProposedTranslation = 0.0;
      m_ProposedRotation = m_MaxRotation;
      return;
    }

  if ( dangerFront && dangerBack && turnLeftAllowed )
    {
      m_ProposedTranslation = 0.0;
      m_ProposedRotation = -m_MaxRotation;
      return;
    }

  if (!dangerFront && dangerBack)
    {
      m_ProposedTranslation = m_MaxTranslation;
      
      if ( (turnRightAllowed) && (m_LocalTargetY >= m_RoboY) )
	{
	  m_ProposedRotation =  m_MaxRotation;
	}
      else if ( (turnLeftAllowed) && (m_LocalTargetY <= m_RoboY) )
	{
	  m_ProposedRotation = -m_MaxRotation;
	}
    }

  if (dangerFront && !dangerBack)
    {
      m_ProposedTranslation = -m_MaxTranslation;

      if ( (turnRightAllowed) && (m_LocalTargetY >= m_RoboY) )
	{
	  m_ProposedRotation =  m_MaxRotation;
	}
      else if ( (turnLeftAllowed) && (m_LocalTargetY <= m_RoboY) )
	{
	  m_ProposedRotation = -m_MaxRotation;
	}
    }

  if ( !dangerFront && !dangerBack )
    {
      // entscheide ueber die zielkoordinaten welche richtung einzuschlagen ist
      if ( m_TargetX > m_RoboX )
	{
	  m_ProposedTranslation = m_MaxTranslation;
    	}
      else
	{
	  m_ProposedTranslation = -m_MaxTranslation;
	}

      if ( (turnRightAllowed) && (m_LocalTargetY >= m_RoboY) )
	{
	  m_ProposedRotation =  m_MaxRotation;
	}
      else if ( (turnLeftAllowed) && (m_LocalTargetY <= m_RoboY) )
	{
	  m_ProposedRotation = -m_MaxRotation;
	}

      return;
    }
}


/* ************************************************************************** */
/* ***********************     Private Methods      ************************* */
/* ************************************************************************** */

void CEscapeDriveModule::FillNormalizedReadings()
{
  m_vNormalizedReadings.clear();

  for ( int i = 0; i < m_pLaser->GetNumberOfReadings(); i++ )
    {
      float rad    = normalize_rad( m_pLaser->GetRadiansForReading( i ) );
      float sub    = m_pRoboShape->GetRobotLengthforRad( rad );
      float length = m_pLaser->GetReadingLength( i );
      m_vNormalizedReadings.push_back( length - sub );
    }
}


void CEscapeDriveModule::SortNormalizedReadings()
{
  m_vFront.clear();
  m_vBack.clear();
  m_vLeftFront.clear();
  m_vLeftBack.clear();
  m_vRightFront.clear();
  m_vRightBack.clear();

  int pipe = 0;
  int i = 0;
  float rad    = normalize_rad( m_pLaser->GetRadiansForReading( i ) );
  while ( i < m_pLaser->GetNumberOfReadings() )
    {
      if ( (pipe == 0) && !m_pLaser->IsPipe( rad ) )
	m_vFront.push_back( m_vNormalizedReadings[i] );

      else if ( (pipe == 1) && !m_pLaser->IsPipe( rad ) && (rad < M_PI_2) )
	m_vRightFront.push_back( m_vNormalizedReadings[i] );
      else if ( (pipe == 1) && !m_pLaser->IsPipe( rad ) && (rad > M_PI_2) )
	m_vRightBack.push_back( m_vNormalizedReadings[i] );

      else if ( (pipe == 2) && !m_pLaser->IsPipe( rad ) )
	m_vBack.push_back( m_vNormalizedReadings[i] );

      else if ( (pipe == 3) && !m_pLaser->IsPipe( rad ) && (rad > 3*M_PI_2) )
	m_vLeftFront.push_back( m_vNormalizedReadings[i] );
      else if ( (pipe == 3) && !m_pLaser->IsPipe( rad ) && (rad < 3*M_PI_2) )
	m_vLeftBack.push_back( m_vNormalizedReadings[i] );

      else if ( (pipe == 4) && !m_pLaser->IsPipe( rad ) )
	m_vFront.push_back( m_vNormalizedReadings[i] );
	
      rad = m_pLaser->GetRadiansForReading( ++i );

      if ( m_pLaser->IsOnlyPipe( rad ) )
	{
	  ++pipe;
	  while (m_pLaser->IsOnlyPipe( rad ))
	    {
	      rad = m_pLaser->GetRadiansForReading( ++i );
	    }
	}
    }
}


bool CEscapeDriveModule::CheckDanger( vector< float > readings )
{
  // if something is smaller than 5 cm, you have to flee.......
  for ( unsigned int i = 0; i < readings.size(); i++ )
    if ( readings[i] < 0.06 )
      return true;
  return false;
}


bool CEscapeDriveModule::TurnLeftAllowed()
{
  for ( unsigned int i = 0; i < m_vFront.size(); i++ )
    if ( m_vFront[i] < 0.12 )
      return false;

  for ( unsigned int i = 0; i < m_vRightFront.size(); i++ )
    if ( m_vRightFront[i] < 0.06 )
      return false;
    
  for ( unsigned int i = 0; i < m_vBack.size(); i++ )
    if ( m_vBack[i] < 0.07 )
      return false;

  for ( unsigned int i = 0; i < m_vLeftBack.size(); i++ )
    if ( m_vLeftBack[i] < 0.13 )
      return false;

  return true;
}



bool CEscapeDriveModule::TurnRightAllowed()
{
  for ( unsigned int i = 0; i < m_vFront.size(); i++ )
    if ( m_vFront[i] < 0.12 )
      return false;

  for ( unsigned int i = 0; i < m_vLeftFront.size(); i++ )
    if ( m_vLeftFront[i] < 0.06 )
      return false;
    
  for ( unsigned int i = 0; i < m_vBack.size(); i++ )
    if ( m_vBack[i] < 0.07 )
      return false;

  for ( unsigned int i = 0; i < m_vRightBack.size(); i++ )
    if ( m_vRightBack[i] < 0.13 )
      return false;

  return true;
}


#endif
