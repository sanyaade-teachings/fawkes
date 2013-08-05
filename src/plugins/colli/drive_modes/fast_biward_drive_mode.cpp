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
/* Description: This is fast biward drive module implementation of Colli-A*    */
/*                                                                      */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the fast biward drive module.                        */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */

#include "fast_biward_drive_mode.h"

using namespace std;

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  /** Initialize your local values here.
   */
  CFastBiwardDriveModule::CFastBiwardDriveModule(Logger* logger, Configuration *config,
      CFastForwardDriveModule* forward_module, CFastBackwardDriveModule* backward_module)
      : CAbstractDriveMode(logger, config)
  {
    loggerFastBi = logger;
    loggerFastBi->log_info("CFastBiwardDriveModule", "CFastBiwardDriveModule(Constructor): Entering...\n");
    m_DriveModeName = FastAllowBackward;
    m_pFastForwardDriveModule = forward_module;
    m_pFastBackwardDriveModule = backward_module;

    m_CountForward = 1;
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
     }*/
    if (!config->exists("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_TRANS")) {
      cout << "***** ERROR *****: Could not find: CFastDriveModule_MAX_TRANS " << " --> ABORTING!" << endl << endl;
      return;
    } else {
      m_MaxTranslation = config->get_float("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_TRANS");
      //   cout << "CFastDriveModule_MAX_TRANS: " << m_MaxTranslation << endl;
    }

    if (!config->exists("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_ROT")) {
      cout << "***** ERROR *****: Could not find: CFastDriveModule_MAX_ROT " << " --> ABORTING!" << endl << endl;
      return;
    } else {
      m_MaxRotation = config->get_float("/plugins/colli/CFastForwardDriveModule/CFastDriveModule_MAX_ROT");
      // cout << "CFastDriveModule_MAX_ROT: " << m_MaxRotation << endl;
    }
    loggerFastBi->log_info("CFastBiwardDriveModule", "CFastBiwardDriveModule(Constructor): Exiting...");
  }

  /** Destruct your local values here.
   */
  CFastBiwardDriveModule::~CFastBiwardDriveModule()
  {
    loggerFastBi->log_info("CFastBiwardDriveModule", "CFastBiwardDriveModule(Destructor): Entering...\n");
    m_DriveModeName = MovingNotAllowed;
    loggerFastBi->log_info("CFastBiwardDriveModule", "CFastBiwardDriveModule(Destructor): Exiting...\n");
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
  void
  CFastBiwardDriveModule::Update()
  {
    m_ProposedTranslation = 0.0;
    m_ProposedRotation = 0.0;

    // Our drive mode (choose between forward and backward)
    CAbstractDriveMode * driveMode = 0;

    // Search the correct drive mode
    float angle_to_target = atan2(m_LocalTargetY, m_LocalTargetX);

    if (m_CountForward == 1 && fabs(angle_to_target) > M_PI_2 + 0.1) {
      m_CountForward = -1;
    } else if (m_CountForward == 1) {
      m_CountForward = 1;
    } else if (m_CountForward == -1 && fabs(angle_to_target) < M_PI_2 - 0.1) {
      m_CountForward = 1;
    } else if (m_CountForward == -1) {
      m_CountForward = -1;
    } else {
      //cout << "Undefined state" << endl;
      loggerFastBi->log_error("CFastBiwardDriveModule", "Undefined state\n");
      m_CountForward = 0;
    }

    if (m_CountForward == 1) {
      driveMode = m_pFastForwardDriveModule;
    } else {
      driveMode = m_pFastBackwardDriveModule;
    }

    // set the current info to the drive mode
    driveMode->SetCurrentRoboPos(m_RoboX, m_RoboY, m_RoboOri);
    driveMode->SetCurrentRoboSpeed(m_RoboTrans, m_RoboRot);
    driveMode->SetCurrentTarget(m_TargetX, m_TargetY, m_TargetOri);
    driveMode->SetLocalTarget(m_LocalTargetX, m_LocalTargetY);
    driveMode->SetLocalTrajec(m_LocalTrajecX, m_LocalTrajecY);
    driveMode->SetCurrentColliMode(m_OrientAtTarget, m_StopAtTarget);

    // update the drive mode
    driveMode->Update();

    // get the values from the drive mode
    m_ProposedTranslation = driveMode->GetProposedTranslation();
    m_ProposedRotation = driveMode->GetProposedRotation();

  }

} // namespace fawkes
