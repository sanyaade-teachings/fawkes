
/***************************************************************************
 *  move8.h - Simple plugin to make the Robotino drive an eight using the MotorInterface
 *
 *  Created: Sat May 18 02:11:53 2013
 *  Copyright  2012 Frederik Zwilling
 *
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

#ifndef __PLUGINS_PLUGIN_TEMPLATE_THREAD_H_
#define __PLUGINS_PLUGIN_TEMPLATE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>

#include <string>

namespace fawkes {
  class MotorInterface;
  class RobotinoSensorInterface;
}

class Move8Thread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ClockAspect
{

 public:
  Move8Thread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void stop();
  void send_transrot(float vx, float vy, float omega);
  void driveByTime();
  float getInverse(float fl);
  bool inToleranceOf(float angle, float aim);
  bool obstacleInRange();
  fawkes::Time started;
  float angle;
  float startAngle;
  float inverseStartAngle;
  int stateOf8; //0-3s

 private:
  fawkes::MotorInterface     *motor_if_;
  fawkes::RobotinoSensorInterface *sensor;
  
};


#endif
