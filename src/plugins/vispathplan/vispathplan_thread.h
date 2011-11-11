
/***************************************************************************
 *  vispathplan_thread.h - Visualization for pathplan via rviz
 *
 *  Created: Fri Nov 11 21:17:24 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_VISPATHPLAN_VISPATHPLAN_THREAD_H_
#define __PLUGINS_VISPATHPLAN_VISPATHPLAN_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <plugins/ros/aspect/ros.h>

#include <ros/publisher.h>

class PathplanVisualizationThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect
{
 public:
  PathplanVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  size_t last_id_num_;
  ros::Publisher vispub_;
};


#endif
