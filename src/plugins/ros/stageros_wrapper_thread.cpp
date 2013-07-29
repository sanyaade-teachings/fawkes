
/***************************************************************************
 *  laserscan_thread.cpp - Thread to exchange laser scans
 *
 *  Created: Tue May 29 19:41:18 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include "stageros_wrapper_thread.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>

#include <ros/this_node.h>
#include <sensor_msgs/LaserScan.h>

#include <fnmatch.h>

using namespace fawkes;

/** @class StagerosWrapperThread "pcl_thread.h"
 * Thread to exchange point clouds between Fawkes and ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
StagerosWrapperThread::StagerosWrapperThread()
  : Thread("StagerosWrapperThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

/** Destructor. */
StagerosWrapperThread::~StagerosWrapperThread()
{
}



void
StagerosWrapperThread::init()
{

  //subscribe ROS Topic "scan"
  try {
    __sub = rosnode->subscribe("base_scan", 100, &StagerosWrapperThread::laser_scan_message_cb, this);
  } catch (Exception& e) {
    e.append("%s initialization failed, could not register ros laser subscriber", name());
    logger->log_error(name(), e);
    throw;
  }

  // try to open laser interface for writing
  try {
    __laser_if = blackboard->open_for_writing<Laser360Interface>("base_scan");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open laser interface for writing", name());
    logger->log_error(name(), e);
    throw;
  }

}

void
StagerosWrapperThread::finalize()
{
  
  // unregister ros subscriber
  try {
    __sub.shutdown();
  } catch (Exception& e) {
    logger->log_error(name(), "Unregistering ros laser subscriber failed!");
    logger->log_error(name(), e);
  }

  // close BlackBoard laser interface
  try {
    blackboard->close(__laser_if);
    //__laser_if->clear();
  } catch (Exception& e) {
    logger->log_error(name(), "Closing laser interface failed!");
    logger->log_error(name(), e);
  }
}


void
StagerosWrapperThread::loop()
{
}


/** Callback function for ROS laser scan message subscription.
 * @param msg incoming message
 */
void
StagerosWrapperThread::laser_scan_message_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  float distances[360];

	for (unsigned int i = 0; i < 360; ++i) {
		float a_rad = deg2rad(i);
		if ((a_rad < msg->angle_min) || (a_rad > msg->angle_max)) {
			distances[i] = 0.;
		} else {
            		// get closest ray from message
            		int idx = (int)roundf((a_rad - msg->angle_min) / msg->angle_increment);
            		distances[i] = msg->ranges[idx];
          	}
        }

  __laser_if->set_distances(distances);  
  __laser_if->write();
}
