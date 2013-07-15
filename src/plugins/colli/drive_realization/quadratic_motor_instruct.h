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
/* $Id$ */
/*                                                                      */
/* Description: This is the quadratic motor instructor for Colli-A*     */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This module is a class for validity checks of drive            */
/*         commands and sets those things with respect to the physical  */
/*         borders of the robot.                                        */
/*       For this purpose the two functions CalculateRotation and       */
/*         CalculateTranslation are implemented quadratically ;-)       */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_QUADRATIC_MOTORINSTRUCT_H_
#define _COLLI_QUADRATIC_MOTORINSTRUCT_H_

//#include "drive_realization/base_motor_instruct.h"
#include "base_motor_instruct.h"

#include <config/config.h>
#include <logging/logger.h>

#include <math.h>
/** A linear motor instruction class. Tries to reach the desired values
 *    from the current values by a linear function.
 */
class CQuadraticMotorInstruct: public CBaseMotorInstruct
//class CQuadraticMotorInstruct
{
public:
  
  ///
  CQuadraticMotorInstruct( MotorInterface* motor, MotorInterface* motor_cmd,float frequency, Logger *logger, Configuration *config );
  
  ///
 // virtual ~CQuadraticMotorInstruct();
 ~CQuadraticMotorInstruct();
  
  
private:
  
  // implementation of the virtual functions of the base class
  

  /// linear implementation of velocity constraints
  float CalculateRotation( float currentRotation, float desiredRotation, 
			   float time_factor );
  
  /// linear implementation of velocity constraints
  float CalculateTranslation( float currentTranslation, float desiredTranslation, 
			      float time_factor );


  /// maximum acceleration and deceleration values for translation and rotation
  float basic_trans_acc;
  float basic_trans_dec;
  float basic_rot_acc;
  float basic_rot_dec;
  Logger *loggerQuad;  

  inline float sqr( float x )
  {
    return (x*x);
  }
  inline int sqr( int x )
  {
    return (x*x);
  }
 
  inline double sqr( double x )
  {
    return (x*x);
  }

  inline unsigned long sqr( unsigned long x )
  {
    return (x*x);
  }

};


#endif
