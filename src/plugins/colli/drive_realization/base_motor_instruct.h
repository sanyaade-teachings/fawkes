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
/*
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
*/
/* ******************************************************************** */
/*                                                                      */
/* $Id$  */
/*                                                                      */
/* Description: This is the basic abstract motor instructor for Colli-A* */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This module is the base class for validity checks of drive     */
/*         commands and sets those things with respect to the physical  */
/*         borders of the robot.                                        */
/*       For this purpose the two functions CalculateRotation and       */
/*         CalculateTranslation have to be implemented in the derived   */
/*         class.                                                       */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_BASE_MOTORINSTRUCT_H_
#define _COLLI_BASE_MOTORINSTRUCT_H_


/*#include <interfaces/mopo_client.h>
#include <utils/system/timestamp.h>
#include <data_utils/rob/robo_motorcontrol.h>
#include <data_utils/rob/robo_laser.h>*/

#include <interfaces/MotorInterface.h>
#include <logging/logger.h>
#include <utils/time/time.h>
#include <blackboard/remote.h>

#include <iostream>
#include <string.h>
#include <string>
#include <cstring>
#include <math.h>
#include <time.h>
#include <vector>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

using namespace fawkes;
using namespace std;

/** The Basic of a Motorinstructor.
 */
class CBaseMotorInstruct
{
public:
    
  ///
  //CBaseMotorInstruct( bbClients::Mopo_Client* motor, float frequency );
  CBaseMotorInstruct( MotorInterface *motor, float frequency ,Logger *logger);
  ///
  //virtual ~CBaseMotorInstruct();
  ~CBaseMotorInstruct();

  /** Try to realize the proposed values
   *    with respect to the maximum allowed values.
   */
  void Drive( float proposedTrans, float proposedRot );


  /** Executes a soft stop with respect to CalculateTranslation
   *    and CalculateRotation.
   */
  void ExecuteStop( );


private:

  //////////////////////////////////////////////////////////////////////
  // VARIABLES
  //////////////////////////////////////////////////////////////////////

  float m_execTranslation, m_execRotation;
  float m_desiredTranslation, m_desiredRotation;
  float m_currentTranslation, m_currentRotation;
  float m_Frequency;

  Logger *loggerTmp;
  MotorInterface *motor_if;
  MotorInterface  *motor_if_read;
  BlackBoard *bb_if;

  Time *m_OldTimestamp;

  //////////////////////////////////////////////////////////////////////
  // METHODS
  //////////////////////////////////////////////////////////////////////
    
  // setCommand sets the executable commands and sends them
  void SetCommand( );
    

  // calculates rotation speed
  // has to be implemented in its base classes!
  // is for the smoothness of rotation transitions.
  // calculate maximum allowed rotation change between proposed and desired values
  virtual float CalculateRotation( float currentRotation, float desiredRotation, 
				   float time_factor ) = 0;


  // calculates translation speed.
  // has to be implemented in its base classes!
  // is for the smoothness of translation transitions.
  // calculate maximum allowed translation change between proposed and desired values
  virtual float CalculateTranslation( float currentTranslation, float desiredTranslation, 
				      float time_factor ) = 0;
};


// ** util function, not completely supported in fawkes ** //
inline  float TimeDiff(Time *currentTime,Time *m_OldTimestamp)
{
  long timediffSec = currentTime->in_sec() - m_OldTimestamp->in_sec();
  long timediffUsec = currentTime->in_usec() - m_OldTimestamp->in_usec();
  if( timediffUsec < 0 )
    {
      timediffSec--;
      timediffUsec += 1000000;
    }
  else if( timediffUsec >= 1000000 )
    {
      timediffSec++;
      timediffUsec -= 1000000;
    }

  float timediff = (float)timediffSec +( ((float)timediffUsec)/1000000.0 );
  return timediff;
}

/* ************************************************************************** */
/* ********************  BASE IMPLEMENTATION DETAILS  *********************** */
/* ************************************************************************** */


// Constructor. Initializes all constants and the local pointers.
inline CBaseMotorInstruct::CBaseMotorInstruct( MotorInterface * motor, float frequency, Logger *logger )
{
  m_OldTimestamp = new Time();
  m_OldTimestamp = &(m_OldTimestamp->stamp());
  motor_if = motor;
  loggerTmp = logger;
  loggerTmp->log_info("BaseMotorInstruct","CBaseMotorInstruct(Constructor): Entering\n");
  // init all members, zero, just to be on the save side
  m_desiredTranslation = m_desiredRotation = 0.0;     
  m_currentTranslation = m_currentRotation = 0.0;     
  m_execTranslation    = m_execRotation    = 0.0;
  // ** m_OldTimestamp.Stamp();
  m_Frequency = frequency;
// ** in order to send messages to motor plugin
  char *host = (char *)"localhost";
  unsigned short int port = 1910;
  bb_if = new RemoteBlackBoard(host, port);
  motor_if_read = bb_if->open_for_reading<MotorInterface>("Motor");
// **
  loggerTmp->log_info("BaseMotorInstruct","CBaseMotorInstruct(Constructor): Exiting\n");
}



// Destructor
inline CBaseMotorInstruct::~CBaseMotorInstruct()
{
  loggerTmp->log_info("BaseMotorInstruct","CBaseMotorInstruct(Destructor): Entering\n");
  bb_if->close(motor_if_read);
  loggerTmp->log_info("BaseMotorInstruct","CBaseMotorInstruct(Destructor): Exiting\n");
}



// setcommand. Puts the command to the motor.
inline void CBaseMotorInstruct::SetCommand(  ) 
{
/*
  // This case here should be removed after Alex's RPC does work.
  // SJ TODO!!!
  if ( !(GetMovingAllowed()) )
    SetRecoverEmergencyStop();
  // SJ TODO!!!
*/
  // Translation borders
  float vx,vy,omega;
  vy = 0.0;
  if ( fabs(m_execTranslation) < 0.05 )
  {
    //SetDesiredTranslation( 0.0 );
    motor_if->set_vx(0.0);
    vx = 0.0;
  }
  else
  {
    if ( fabs(m_execTranslation) > 3.0 )
    {
      if ( m_execTranslation > 0.0 )
      {
	//SetDesiredTranslation( 3.0 );
        motor_if->set_vx(3.0);
        vx = 3.0;
      }
      else
      {
	//SetDesiredTranslation( -3.0 );
        motor_if->set_vx(-3.0);
        vx = -3.0;
      }
    }
    else
    {
      //SetDesiredTranslation( m_execTranslation );
      motor_if->set_vx( m_execTranslation );
      vx = m_execTranslation;
    }
  }
  // Rotation borders
  if ( fabs(m_execRotation) < 0.01 )
  {
    //SetDesiredRotation( 0.0 );
    motor_if->set_omega(0.0);
    omega = 0.0;
  }
  else
  {
    if ( fabs(m_execRotation) > 2*M_PI )
    {
      if ( m_execRotation > 0.0 )
      {
	//SetDesiredRotation( 2*M_PI );
        motor_if->set_omega(2*M_PI);
        omega = 2*M_PI;
      }
      else
      {
	//SetDesiredRotation( -2*M_PI );
        motor_if->set_omega(-2*M_PI);
        omega = -2*M_PI;
      }
    }
    else
    {
      //SetDesiredRotation( m_execRotation );
      motor_if->set_omega( m_execRotation );
      omega = m_execRotation;
    }
  }
  motor_if->write();
  /*motor_if_read->read();
  motor_if->set_odometry_position_x(motor_if_read->odometry_position_x());
  motor_if->set_odometry_position_y(motor_if_read->odometry_position_y());  
  motor_if->set_odometry_orientation(motor_if_read->odometry_orientation());*/
  // Send the commands to the motor. No controlling afterwards done!!!!
  //SendCommand();
  //motor_if->read();
  //MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(motor_if->vx(),motor_if->vy(),motor_if->omega());
  //loggerTmp->log_info("drive realization","vx %f, vy %f, omega %f",vx,vy,omega );
   MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(vx,vy,omega);
  //MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(1.5,0.0,0.0); // ** for test if it works remotely ** //
   motor_if_read->msgq_enqueue(msg);
  
  //motor_if->set_motor_state(motor_if->DRIVE_MODE_TRANS_ROT);
   //cout << "motor velocity: " << motor_if->vx() << endl;
  //cout << "motor rotation: "<< motor_if->omega() << endl;
}



// Set a drive command with respect to the physical constraints of the robot.
inline void CBaseMotorInstruct::Drive( float proposedTrans, float proposedRot )
{
  // initializing driving values (to be on the sure side of life)
  m_execTranslation = 0.0;
  m_execRotation = 0.0;
/*
  // timediff storie to realize how often one was called
  Timestamp currentTime;
  currentTime.Stamp();
  float timediff = currentTime - m_OldTimestamp;
  float time_factor = ( (timediff*1000.0) / m_Frequency);

  if (time_factor < 0.5) 
    {
      BB_DBG(1) << "CBaseMotorInstruct( Drive ): "
		<< "Blackboard timing(case 1) strange, time_factor is " 
		<< time_factor << endl;
    }
  else if (time_factor > 2.0)
    {
      BB_DBG(1) << "CBaseMotorInstruct( Drive ): "
	        << "Blackboard timing(case 2) strange, time_factor is " 
		<< time_factor << endl;    
    }
  else
    {
      ;
    }


  m_OldTimestamp = currentTime; */
  
  Time *currentTime = new Time();
  currentTime = &(currentTime->stamp());
  //float timediff = currentTime - m_OldTimestamp;

  float timediff = TimeDiff(currentTime,m_OldTimestamp);
//  loggerTmp->log_info("CBaseMotorInstruct","timediff is: %f , frequency is: %f \n",timediff,m_Frequency);
  // ** for now, just for test ** //
  float time_factor = ( (timediff*1000.0) / m_Frequency );

/*  if(time_factor < 0.5)
    {
      loggerTmp->log_info("CBaseMotorInstruct","CBaseMotorInstruct( Drive ): Blackboard timing(case 1) strange, time_factor is %f\n",time_factor); 
    }
  else if (time_factor > 2.0)
    {
      loggerTmp->log_info("CBaseMotorInstruct","CBaseMotorInstruct( Drive ): Blackboard timing(case 2) strange, time_factor is %f\n",time_factor);
    }
  else
    {
      ;
    }*/
  // getting current performed values
  //m_currentRotation    = GetMotorDesiredRotation();
  //m_currentTranslation = GetMotorDesiredTranslation();
  m_currentRotation    = motor_if->omega();
  m_currentTranslation = motor_if->vx();

  // calculate maximum allowed rotation change between proposed and desired values
  m_desiredRotation = proposedRot;
  // ** TODO for test time_factor ** //
  time_factor = 1.0;
  m_execRotation    = CalculateRotation( m_currentRotation, m_desiredRotation, time_factor );
//  cout << "rotation calculated: " << m_execRotation << endl;
  // calculate maximum allowed translation change between proposed and desired values
  m_desiredTranslation = proposedTrans;
  m_execTranslation    = CalculateTranslation( m_currentTranslation, m_desiredTranslation, time_factor );
//  cout << "translation calculated: " << m_execTranslation << endl;
 //  send the command to the motor
  SetCommand( );
}



// Does execute a stop command if it is called several times
inline void CBaseMotorInstruct::ExecuteStop()
{
  //m_currentTranslation = GetMotorDesiredTranslation();
  //m_currentRotation    = GetMotorDesiredRotation();
  m_currentTranslation = motor_if->vx();
  m_currentRotation = motor_if->omega();

  // with respect to the physical borders do a stop to 0.0, 0.0.
  Drive( 0.0, 0.0 );
  SetCommand( );
}



#endif
