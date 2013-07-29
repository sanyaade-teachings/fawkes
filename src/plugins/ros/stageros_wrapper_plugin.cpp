
/***************************************************************************
 *  stage_laser_plugin.cpp - Exchange simulated laser scans between Fawkes and ROS
 *
 *  Created: Sun July 29 19:30:47 2012
 *  Copyright  2013  Sebastian Reuter
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

#include <core/plugin.h>

#include "stageros_wrapper_thread.h"

using namespace fawkes;

/** Plugin exchange laser scans between Fawkes and ROS.
 * @author Tim Niemueller
 */
class StagerosWrapperPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  StagerosWrapperPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new StagerosWrapperThread());
  }
};

PLUGIN_DESCRIPTION("Wraps the 2D Simulation Stage running in ROS Environment")
EXPORT_PLUGIN(StagerosWrapperPlugin)
