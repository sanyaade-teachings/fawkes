
/***************************************************************************
 *  qa_projection.cpp - QA for Projection Filter
 *
 *  Created: Wed Dec 30 12:00:00 2009
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

/// @cond QA

#include "../filters/projection.h"
#include <geometry/hom_polar.h>
#include <utils/time/tracker.h>
#include <utils/math/angle.h>
#include <utils/math/coord.h>

#include <cstdio>
#include <cstring>
#include <unistd.h>

using namespace fawkes;

int
main(int argc, char **argv)
{
  int data_size = 360;
  float* readings = new float[data_size];

  for (int i = 0; i < data_size; ++i) {
    readings[i] = 0.0f;
  }
  //readings[0] = 2.934f;
  readings[0] = 1.50f;
  readings[90] = 1.50f;
  readings[135] = 1.50f;
  readings[270] = 1.50f;
  readings[180] = 1.50f;
  readings[40] = 1.50f; // this one is too low (Z_THRESHOLD)
  readings[39] = 0.40f; // this one is in the robot

  std::vector<float*> in;
  in.push_back(readings);

  for (std::vector<float*>::const_iterator it = in.begin();
       it != in.end(); ++it) {
    float* inbuf = *it;
    for (int i = 0; i < data_size; ++i) {
      if (inbuf[i] != 0.0) {
        const float angle = static_cast<float>(i);
        const float length = inbuf[i];
        const HomPolar p = HomPolar(length, deg2rad(angle));
        printf("IN  %lf / %lf (%.2f, %.2f, %.2f)\n",
               angle, length, p.x(), p.y(), p.z());
      }
    }
  }

  const bool LEFT = false;
  const LaserProjectionDataFilter::Rotation LASER_ROT(0.0f, -90.0f, (LEFT ? 90.0f : -90.0f));
  const LaserProjectionDataFilter::Rotation FIXTURE_ROT((LEFT ? -39.0f : 39.0f), 0.0f, (LEFT ? -39.0f : 39.0f));
  const LaserProjectionDataFilter::Translation TRANS(0.08f, (LEFT ? 0.20f : -0.20f), 1.17f);
  const LaserProjectionDataFilter::Rectangle ROBOT_RECT(-0.07, 0.31, -0.20f, 0.20f);
  const float Z_THRESHOLD = -0.05;
  LaserProjectionDataFilter filter(LASER_ROT, FIXTURE_ROT, TRANS, ROBOT_RECT, Z_THRESHOLD , data_size, in);
  filter.filter();

  const std::vector<float*> out = filter.get_out_vector();
  for (std::vector<float*>::const_iterator it = out.begin();
       it != out.end(); ++it) {
    float* outbuf = *it;
    for (int i = 0; i < data_size; ++i) {
      if (outbuf[i] != 0.0f) {
        const float new_angle = static_cast<float>(i);
        const float new_length = outbuf[i];
        const HomPolar p = HomPolar(new_length, deg2rad(new_angle));
        printf("OUT %lf / %lf (%.2f, %.2f, %.2f)\n",
               new_angle, new_length, p.x(), p.y(), p.z());
      }
    }
  }

  delete[] readings;
  return 0;
}

/// @endcond
