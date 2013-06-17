/***************************************************************************
 *  robotino-sim_thread.h - Thread simulate the Robotino in Gazebo by sending needed informations to the Robotino-plugin in Gazebo and recieving sensordata from Gazebo
 *
 *  Created: Fr 3. Mai 21:20:08 CEST 2013
 *  Copyright  2013  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_ROBOTINO_SIM_THREAD_H_
#define __PLUGINS_ROBOTINO_SIM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>

namespace fawkes {
  class MotorInterface;
  class BatteryInterface;
  class RobotinoSensorInterface;
}

class RobotinoSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  RobotinoSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
 private:
  //Publisher to send messages to gazebo
  gazebo::transport::PublisherPtr stringPub;
  gazebo::transport::PublisherPtr motorMovePub;

  //Suscribers to recieve messages from gazebo
  gazebo::transport::SubscriberPtr gyroSub;

  //Handler functions for incoming messages
  void OnGyroMsg(ConstVector3dPtr &msg);

  //provided interfaces
  fawkes::BatteryInterface        *batt_if_;
  fawkes::RobotinoSensorInterface *sens_if_;
  fawkes::MotorInterface         *motor_if_;
  
  //Helper variables:
  //motorMovements last sent to gazebo
  float vx, vy, vomega;

  //Helper functions:
  void sendMotorMove();
};

#endif