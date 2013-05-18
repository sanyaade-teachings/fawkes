/***************************************************************************
 *  move8_thread.cpp - Simple plugin to make the Robotino drive an eight using the MotorInterface
 *
 *  Created: Sat May 18 02:14:36 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "move8_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/RobotinoSensorInterface.h>

#include <cmath>

#define FORWARD_SPEED 0.5f
#define SIDEWARD_SPEED 0.0f
#define ROTATIONAL_SPEED 1.5f
#define ROUND_TIME 6.3f
#define DEVIATION 0.05f
#define PI 3.1415926f

using namespace fawkes;

/** @class PluginTemplateThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
Move8Thread::Move8Thread() :
		Thread("Move8Thread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void Move8Thread::init() {

	logger->log_info(name(), "Move8 starts up");
	motor_if_ = blackboard->open_for_reading<MotorInterface>("Robotino");
	sensor = blackboard->open_for_reading<RobotinoSensorInterface>("Robotino");
	started = clock->now();
	sensor->read();
	startAngle = sensor->gyro_angle();
	inverseStartAngle = getInverse(startAngle);
	stateOf8 = 0;
}

bool Move8Thread::prepare_finalize_user() {
	stop();
	return true;
}

float Move8Thread::getInverse(float fl) {
	float tmp = fl + PI;
	if (tmp > PI) {
		tmp -= PI * 2;
	}
	return tmp;
}

void Move8Thread::finalize() {
	blackboard->close(motor_if_);
	float ended = clock->elapsed(&started);
	logger->log_info(name(), "Abgelaufene Zeit seid start: %f", ended);
}

void Move8Thread::send_transrot(float vx, float vy, float omega) {
	MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(
			vx, vy, omega);
	motor_if_->msgq_enqueue(msg);
}

void Move8Thread::stop() {
	send_transrot(0., 0., 0.);
}

//Work in this method
void Move8Thread::loop() {
	sensor->read();
	angle = sensor->gyro_angle();

	if (obstacleInRange()) {
		//Obstacle detected
		//turn
		if(stateOf8 == 0 || stateOf8 == 1 )
			send_transrot(0.0, 0.0, ROTATIONAL_SPEED);
		else
			send_transrot(0.0, 0.0, -ROTATIONAL_SPEED);

		return;
	} else {
		switch (stateOf8) {
		case 0: {
			send_transrot(FORWARD_SPEED, SIDEWARD_SPEED, ROTATIONAL_SPEED);
			//wait for inverseAngle, driving right
			if (inToleranceOf(angle, inverseStartAngle)) {
				stateOf8++;
			}
			break;
		}
		case 1: {
			send_transrot(FORWARD_SPEED, SIDEWARD_SPEED, ROTATIONAL_SPEED);
			//wait for inverseAngle, driving right
			if (inToleranceOf(angle, startAngle)) {
				stateOf8++;
			}
			break;
		}
		case 2: {
			send_transrot(FORWARD_SPEED, -SIDEWARD_SPEED, -ROTATIONAL_SPEED);
			//wait for inverseAngle, driving right
			if (inToleranceOf(angle, inverseStartAngle)) {
				stateOf8++;
			}
			break;
		}
		case 3: {
			send_transrot(FORWARD_SPEED, -SIDEWARD_SPEED, -ROTATIONAL_SPEED);
			//wait for inverseAngle, driving right
			if (inToleranceOf(angle, startAngle)) {
				stateOf8 = 0;
			}
			break;
		}
		}
	}

}

bool Move8Thread::inToleranceOf(float angle, float aim) {
	if (aim + DEVIATION > angle && aim - DEVIATION < angle) {
		return true;
	}
	//Angle could be close to pi or -pi
	aim += 2 * PI;
	if (aim + DEVIATION > angle && aim - DEVIATION < angle) {
		return true;
	}
	aim -= 4 * PI;
	if (aim + DEVIATION > angle && aim - DEVIATION < angle) {
		return true;
	}
	return false;
}

void Move8Thread::driveByTime() {
	if (clock->elapsed(&started) < ROUND_TIME) {
		send_transrot(FORWARD_SPEED, SIDEWARD_SPEED, ROTATIONAL_SPEED);
		logger->log_info(name(), "Driving madly!!");
	} else if (clock->elapsed(&started) < ROUND_TIME * 2) {
		send_transrot(FORWARD_SPEED, -SIDEWARD_SPEED, -ROTATIONAL_SPEED);
	}

	else {
		started = clock->now();
	}
}

//sensor read required before
bool Move8Thread::obstacleInRange() {
	sensor->read();
	//distances the robotino holds to obstacles
	float neededSpace[] = {0.3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
	float* distances = sensor->distance();
	for (int i = 0; i < 9; i++) {
		if (distances[i] != 0 && distances[i] < neededSpace[i]) {
			logger->log_info(name(), "Move8 stoped, Obstacle seen at index %i, sensor value: %f",
					i, distances[i]);
			return true;
		}
	}
	return false;
}

