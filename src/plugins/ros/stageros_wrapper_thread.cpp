
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

#include <utils/math/angle.h>
#include <math.h>

#include <ros/this_node.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/types.h>


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

  //subscribe ROS Topic "base_scan"
  try {
    __laser_sub = rosnode->subscribe("base_scan", 100, &StagerosWrapperThread::laser_scan_message_cb, this);
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

  //subscribe ROS Topic "odom"
  try {
    __odom_sub = rosnode->subscribe("odom", 100, &StagerosWrapperThread::odom_message_cb, this);
  } catch (Exception& e) {
    e.append("%s initialization failed, could not register odom subscriber", name());
    logger->log_error(name(), e);
    throw;
  }

  // try to open motor interface for writing
  try {
    __motor_if = blackboard->open_for_writing<MotorInterface>("Robotino");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open Motor interface for writing", name());
    logger->log_error(name(), e);
    throw;
  }

  //publish ROS Topic "twist"
  try {
    __cmd_vel_pub = rosnode->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  } catch (Exception& e) {
    e.append("%s initialization failed, could not register cmd_vel publisher", name());
    logger->log_error(name(), e);
    throw;
  }
}

void
StagerosWrapperThread::finalize()
{
  
  // unregister ros laser subscriber
  try {
    __laser_sub.shutdown();
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

  // unregister ros odom subscriber
  try {
    __odom_sub.shutdown();
  } catch (Exception& e) {
    logger->log_error(name(), "Unregistering ros odom subscriber failed!");
    logger->log_error(name(), e);
  }

  // close BlackBoard laser interface
  try {
    blackboard->close(__motor_if);
  } catch (Exception& e) {
    logger->log_error(name(), "Closing motor interface failed!");
    logger->log_error(name(), e);
  }

  // unregister ros odom subscriber
  try {
    __cmd_vel_pub.shutdown();
  } catch (Exception& e) {
    logger->log_error(name(), "Unregistering ros cmd_vel publisher failed!");
    logger->log_error(name(), e);
  }
}


void
StagerosWrapperThread::loop()
{

  //process incoming Velocity Commands from Fawkes
  __motor_if->read();
  while( ! __motor_if->msgq_empty() ){

	if( fawkes::MotorInterface::TransRotMessage *msg = __motor_if->msgq_first_safe(msg)){ 
		publish_cmd_vel_message(msg);
  	}
  }

}


/** Callback function for ROS laser scan message subscription.
 * @param msg incoming message
 */
void
StagerosWrapperThread::laser_scan_message_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  float distances[360];

	for (unsigned int i = 0; i < 360; ++i) {
		/************ no need for this snippet as we use 1°-inkrements in the Laser360-Interface*********************** 
		float a_rad = deg2rad(i);
		if ((a_rad < msg->angle_min) || (a_rad > msg->angle_max)) {
			distances[i] = 0.;
		} else {
            		// get closest ray from message
            		int idx = (int)roundf((a_rad - msg->angle_min) / msg->angle_increment);
            		distances[i] = msg->ranges[idx];
          	}
		*/
		
		// dirty hardcoded stuff follows
		
		if ((0 <= i ) && (i <= 90 )){
			distances[90-i] = msg->ranges[i];
		}
		else if((270 <= i ) && (i < 360)){
			distances[i] = msg->ranges[450-i];			
		}
		else {
			distances[i] = 0.0;
		}

        }
  __laser_if->set_distances(distances);  
  __laser_if->write();
}

void
StagerosWrapperThread::odom_message_cb(const nav_msgs::Odometry::ConstPtr &msg){
   
  const float odomX = (*msg).pose.pose.position.x;
  const float odomY = (*msg).pose.pose.position.y;

  // convert quaternion to angle 
  float q_x = (*msg).pose.pose.orientation.x;
  float q_y = (*msg).pose.pose.orientation.y;
  float q_z = (*msg).pose.pose.orientation.z;
  float q_w = (*msg).pose.pose.orientation.w;
  const float odomPhi = asin( 2*q_x*q_y + 2*q_z*q_w);

  __motor_if->set_odometry_position_x( odomX / 1000.f );
  __motor_if->set_odometry_position_y( odomY / 1000.f );
  __motor_if->set_odometry_orientation( deg2rad( odomPhi ) );
  __motor_if->write();
}

void
StagerosWrapperThread::publish_cmd_vel_message(const fawkes::MotorInterface::TransRotMessage *msg){

  // write msg on interface
  __motor_if->set_vx( msg->vx() );
  __motor_if->set_vy( msg->vy() );
  __motor_if->set_omega( msg->omega() );
  __motor_if->write();

  // push msg into ros-format
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = msg->vx();
  twist_msg.linear.y = msg->vy();
  twist_msg.angular.z = msg->omega();

  // publish ros msg
  __cmd_vel_pub.publish(twist_msg);


}
