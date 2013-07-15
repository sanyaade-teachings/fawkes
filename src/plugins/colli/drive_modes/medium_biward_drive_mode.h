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
/* $Id$       */
/*                                                                      */
/* Description: This is the medium biward drive module interface of Colli-A*     */
/*                                                                      */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the medium biwarddrive module.                         */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_MEDIUM_BIWARD_DRIVE_MODE_H_
#define _COLLI_MEDIUM_BIWARD_DRIVE_MODE_H_


#include "abstract_drive_mode.h"
#include "medium_forward_drive_mode.h"
#include "medium_backward_drive_mode.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CMediumBiwardDriveModule : public CAbstractDriveMode
{
public:

  /** Constructor.
   */
  CMediumBiwardDriveModule( Logger* logger, Configuration *config, CMediumForwardDriveModule*  forward_module,
          CMediumBackwardDriveModule* backward_module );


  /** Destructor. Does nothing, because nothing was created in this module.
   */
  ~CMediumBiwardDriveModule();


  /** This Routine is called. Afterwards the m_proposedTranslation and
   *    m_proposedRotation have to be filled. Here they are
   *    set to zero.
   */
  virtual void Update();


private:

  float m_MaxTranslation, m_MaxRotation;

  CMediumForwardDriveModule*   m_pMediumForwardDriveModule;
  CMediumBackwardDriveModule*  m_pMediumBackwardDriveModule;

  int   m_CountForward;

  Logger* loggerMedBi;
};

} // namespace fawkes

#endif
