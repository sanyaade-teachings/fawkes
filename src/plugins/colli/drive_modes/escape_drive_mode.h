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
/* Description: This is the escape drive module interface of Colli-A*   */
/*                                                                      */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the escape drive module. This is called everytime,     */
/*       an escape is necessary.                                        */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_ESCAPE_DRIVE_MODE_H_
#define _COLLI_ESCAPE_DRIVE_MODE_H_


#include <vector>
//#include <utils/roboshape/roboshape_colli.h>
#include "../robo-utils/rob/robo_laser.h"
#include "abstract_drive_mode.h"
#include <interfaces/Laser360Interface.h>
#include "../robo-utils/roboshape_colli.h"
using namespace fawkes;
using namespace std;
/** Class Escape-Drive-Module. This module is called, if an escape is
 *    neccessary. It should try to maximize distance to the disturbing
 *    obstacle.
 */
class CEscapeDriveModule : public CAbstractDriveMode
{
public:

    /** Constructor. Sets only the drive mode name to "MovingNotAllowed"
     */
    CEscapeDriveModule( Logger* logger, Configuration *config, Laser* laser );

  
    /** Destructor. Does nothing, because nothing was created in this module.
     */
    ~CEscapeDriveModule();


    /** This Routine is called. Afterwards the m_proposedTranslation and 
     *    m_proposedRotation have to be filled. Here they are
     *    set to zero.
     */
    virtual void Update();


private:

    /// our pointer to the laserinterface.... lets escape ;-)
    Laser*             m_pLaser;
    CRoboShape_Colli*  m_pRoboShape;

    /// Readings without robolength in it
    vector< float > m_vNormalizedReadings;
    vector< float > m_vFront, m_vBack;
    vector< float > m_vLeftFront,  m_vLeftBack;
    vector< float > m_vRightFront, m_vRightBack;


    /// absolute values are the maximum values. do not act faster!
    float m_MaxTranslation;
    float m_MaxRotation;


    void FillNormalizedReadings();
    void SortNormalizedReadings();

    bool CheckDanger( vector< float > readings );
    bool TurnLeftAllowed();
    bool TurnRightAllowed();

    Logger* loggerEscape;
};


#endif
