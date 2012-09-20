
/***************************************************************************
 *  gazeboscene_plugin.cpp - Fawkes Gazebo Scene Reconstruction Plugin
 *
 *  Created: Mon Aug 27 09:53:54 2012
 *  Author  Bastian Klingen
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

#include "gazeboscene_plugin.h"
#include "gazeboscene_thread.h"

using namespace fawkes;

/** @class GazeboScenePlugin "gazeboscene_plugin.h"
 * Gazebo Scene Reconstruction Plugin.
 * This plugin uses an existing MongoDB Log created by the MongoLog
 * plugin to reconstruct the scene inside the Gazebo Simulator.
 *
 * @author Bastian Klingen
 */

/** Constructor.
 * @param config Fawkes configuration
 */
GazeboScenePlugin::GazeboScenePlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new GazeboSceneThread());
}


PLUGIN_DESCRIPTION("Gazebo Scene Reconstruction from MongoDB")
EXPORT_PLUGIN(GazeboScenePlugin)
