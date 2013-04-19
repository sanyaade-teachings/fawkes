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
  ������������������������������������������������������������������������������
  �                                                                            �
  �                                            ####   ####           .-""-.    �
  �       # #                             #   #    # #    #         /[] _ _\   �
  �       # #                                 #    # #             _|_o_LII|_  �
  � ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ �
  � #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| �
  � #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  �
  � #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  �
  � '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  �
  �                                                               /__|    |__\ �
  �                                                                            �
  ������������������������������������������������������������������������������
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$     */
/*                                                                      */
/* Description: This is fast forward drive module implementation of Colli-A*    */
/*                                                                      */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the fast forward drive module.                       */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_FAST_FORWARD_DRIVE_MODE_CPP_
#define _COLLI_FAST_FORWARD_DRIVE_MODE_CPP_


//#include <utils/utils.h>
//#include <utils/configfile/configfile.h>
#include "fast_forward_drive_mode.h"


using namespace std;


/** Initialize your local values here.
 */
CFastForwardDriveModule::CFastForwardDriveModule( Logger* logger, Configuration *config ) :
  CAbstractDriveMode( logger, config )
{
  loggerFastFor = logger;
  loggerFastFor->log_info("CFastForwardDriveModule","CFastForwardDriveModule(Constructor): Entering...\n");
  m_DriveModeName = FastForward;
/*
  string confFileName = "../cfg/robocup/colli.cfg";
  try 
    {
      ConfigFile * m_pConf = new ConfigFile( confFileName );
      m_MaxTranslation = m_pConf->floating( "CFastDriveModule_MAX_TRANS" );
      m_MaxRotation    = m_pConf->floating( "CFastDriveModule_MAX_ROT" );
      delete m_pConf;
    }
  catch (...)
    {
      BB_DBG(0) << "***** ERROR *****: Could not open: " << confFileName 
		<< " --> ABORTING!" << endl << endl;
      exit( 0 );
    }
*/
  if(!config->exists("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_TRANS") )
  {
    cout << "***** ERROR *****: Could not find: CFastDriveModule_MAX_TRANS "
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_MaxTranslation = config->get_float("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_TRANS");
    //cout << "CFastDriveModule_MAX_TRANS: " << m_MaxTranslation << endl;
  }

  if(!config->exists("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_ROT") )
  {
    cout << "***** ERROR *****: Could not find: CFastDriveModule_MAX_ROT "
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    m_MaxRotation = config->get_float("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_ROT");
    //cout << "CFastDriveModule_MAX_ROT: " << m_MaxRotation << endl;
  }
  loggerFastFor->log_info("CFastForwardDriveModule","CFastForwardDriveModule(Constructor): Exiting...\n");
}


/** Destruct your local values here.
 */
CFastForwardDriveModule::~CFastForwardDriveModule()
{
  loggerFastFor->log_info("CFastForwardDriveModule","CFastForwardDriveModule(Destructor): Entering...\n");
  m_DriveModeName = MovingNotAllowed;
  loggerFastFor->log_info("CFastForwardDriveModule","CFastForwardDriveModule(Destructor): Exiting...\n");
}






/** Calculate by given variables a new rotation to give for the motor to minimize curvature.
 *
 *  DOC.: This here is the most complicated part in realizing the colli. By the given information
 *        I have to calculate a rotation I want to achieve in an optimal way.
 *        Here this is solved in an interesting way:
 *        First, check how long the curvature is, we want to drive to the target. This is done by
 *        approximating the size of the triangle around this curvature given by collision and 
 *        and targetpoint and the angle between those both. Afterwards the time we have to drive
 *        with constant speed is calculated. Now we have the time we want to turn the angle. By
 *        multiplying this with a high constant (here 4), we rotate faster than we want to, but
 *        so the curvatures are not so extraordinary big. Afterwards there is an proportional
 *        part added, so we have a control factor here. (P-Regler ;-) )
 *
 *  @return A desired rotation.
 */
float CFastForwardDriveModule::FastForward_Curvature( float dist_to_target, float dist_to_trajec, float alpha, 
							  float trans_0, float rot_0 )
{
  return 1.5*alpha;
}


/** Calculate by given variables a new translation to give for the motor to 
 *    minimize distance to the target.
 *
 *  DOC.: This here is a fairly easy routine after the previous one ;-). It calculates
 *        to a proposed rotation the translation part. This means, in relation to
 *        distances to target and trajec and the current values we calculate a new one.
 *
 *  @return A desired translation.
 */
float CFastForwardDriveModule::FastForward_Translation ( float dist_to_target, float dist_to_front, float alpha,
							 float trans_0, float rot_0, float rot_1 )
{
  float trans_1 = 0.0;
  
  if ( fabs( rot_1 ) >= 0.0 && fabs( rot_1 ) <= 0.3 )
    {
      trans_1 = LinInterpol( fabs( rot_1 ), 0.3, 0.0, 2.5, m_MaxTranslation+0.1 );
    }
  else if ( fabs( rot_1 ) > 0.3 && fabs( rot_1 ) <= 0.8 )
    {
      trans_1 = LinInterpol( fabs( rot_1 ), 0.8, 0.3, 2.0, 2.5 );
    }
  else if ( fabs( rot_1 ) > 0.8 && fabs( rot_1 ) <= 1.3 )
    {
      trans_1 = LinInterpol( fabs( rot_1 ), 1.3, 0.8, 1.0, 2.0 );
    }
  else if ( fabs( rot_1 ) > 1.3 && fabs( rot_1 ) <= 2.0 )
    {
      trans_1 = LinInterpol( fabs( rot_1 ), 2.0, 1.3, 0.4, 1.0 );
    }
  else if ( fabs( rot_1 ) > 2.0 )
    {
      trans_1 = LinInterpol( fabs( rot_1 ), M_PI, 2.0, 0.0, 0.4 );
    }
  else
    {
      loggerFastFor->log_error("CFastForwardDriveModule","************ FAST DRIVE MODES:::NOT DEFINED STATE!!!!! STOPPING\n");
      trans_1 = 0;
    }
  
  
  // test the borders (no agressive behaviour!)
  if ( trans_1 < 0.0 ) trans_1 = 0.0;
  if ( trans_1 > m_MaxTranslation ) trans_1 = m_MaxTranslation;
  
  // OLD STUFF
  //   // check stopping on target and compare distances with choosen velocities
  //   if ( fabs( dist_to_target - dist_to_front ) < 0.2 )
  //     {
  //       if (m_StopAtTarget == true)
  // 	trans_1 = min( trans_1, dist_to_target*1.5 );
  //       else
  // 	; // do not stop, so drive behind the target with full power
  //     }
  //   else
  //     {
  //       trans_1 = min( trans_1, dist_to_front );
  //     }

  // NEW STUFF
  float trans_target = 10000.0;
  float trans_front  = 10000.0;
  
  if ( m_StopAtTarget == true )
    {
      trans_target = GuaranteeTransStop( dist_to_target, trans_0, trans_1 );
    }
  
  // And the next collision point
  if ( dist_to_front < dist_to_target )
    {
      trans_front = GuaranteeTransStop( dist_to_front, trans_0, trans_1 );
    }
  // NEW STUFF END HERE
  
  trans_1 = min( trans_1, min( trans_target, trans_front ) );
  
  return trans_1;
}




/* ************************************************************************** */
/* ***********************        U P D A T E       ************************* */
/* ************************************************************************** */

/** Calculate here your desired settings. What you desire is checked afterwards to the current
 *    settings of the physical boundaries, but take care also. 
 * 
 *  How you do this is up to you, but be careful, our hardware is expensive!!!!
 * 
 *  Available are:  
 *
 *     m_TargetX, m_TargetY, m_TargetOri  --> current Target to drive to
 *     m_RoboX, m_RoboY, m_RoboOri        --> current Robot coordinates
 *     m_RoboTrans, m_RoboRot             --> current Motor values
 *     
 *     m_LocalTargetX, m_LocalTargetY     --> our local target found by the search component we want to reach      
 *     m_LocalTrajecX, m_LocalTrajecY     --> The point we would collide with, if we would drive WITHOUT Rotation
 *
 *     m_OrientAtTarget                   --> Do we have to orient ourself at the target?
 *     m_StopAtTarget                     --> Do we have to stop really ON the target?
 *
 *  Afterwards filled should be:
 *
 *     m_ProposedTranslation              --> Desired Translation speed
 *     m_ProposedRotation                 --> Desired Rotation speed
 *
 *  Those values are questioned after an Update() was called.
 */
void CFastForwardDriveModule::Update()
{
  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  float dist_to_target  = sqrt(  sqr(m_LocalTargetX) + sqr(m_LocalTargetY) );
  float alpha  = atan2( m_LocalTargetY, m_LocalTargetX );
  float dist_to_trajec  = sqrt(  sqr(m_LocalTrajecX) + sqr(m_LocalTrajecY) );

  m_ProposedRotation = FastForward_Curvature( dist_to_target, dist_to_trajec, 
					      alpha, m_RoboTrans, m_RoboRot );
  
  if ( fabs( alpha ) > M_PI_2 )
    {
      m_ProposedTranslation = 0.0;
    }
  else
    {
      m_ProposedTranslation = FastForward_Translation( dist_to_target, dist_to_trajec, alpha,
						       m_RoboTrans, m_RoboRot, m_ProposedRotation );
    }


  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target < 0.1 )
    {
      m_ProposedTranslation = 0.0;
      m_ProposedRotation    = 0.0;
    }
  else
    {
      m_ProposedTranslation = min ( m_ProposedTranslation, m_MaxTranslation );
      m_ProposedTranslation = max ( m_ProposedTranslation, (float)0.0 );

      if (m_ProposedRotation >  m_MaxRotation)
	m_ProposedRotation =  m_MaxRotation;

      if (m_ProposedRotation < -m_MaxRotation)
	m_ProposedRotation = -m_MaxRotation;

      if ( m_StopAtTarget == false && dist_to_target < 2.0 )
	{
	  // Reduziere die rotationsgeschwindigkeiten, damit keine wilden lenkmanoever kommen
	  if ( m_ProposedRotation > 0.6 )
	    m_ProposedRotation =  0.6;
	  else if ( m_ProposedRotation < -0.6 )
	    m_ProposedRotation = -0.6;
	  else
	    ;
	}

    }
}



#endif
