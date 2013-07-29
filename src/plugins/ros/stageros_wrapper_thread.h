
/***************************************************************************
 *  StagerosWrapper_thread.h - Wrapper for the 2D Simulator stage in ROS Environment
 *
 *  Created: Mon July 29 16:34:39 2013
 *  Copyright  2013  Sebastian Reuter
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

#ifndef __PLUGINS_StagerosWrapper_THREAD_H_
#define __PLUGINS_StagerosWrapper_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>
#include <interfaces/Laser360Interface.h>
#include <core/threading/mutex.h>
#include <utils/time/time.h>

#include <list>
#include <queue>

#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>

class StagerosWrapperThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect
{
 public:
  StagerosWrapperThread();
  virtual ~StagerosWrapperThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  void laser_scan_message_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
  void conditional_close(fawkes::Interface *interface) throw();
  std::string topic_name(const char *if_id, const char *suffix);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  ros::Subscriber __sub;
  sensor_msgs::LaserScan  __laser_msg;
  fawkes::Laser360Interface *__laser_if;

};

#endif
