
/***************************************************************************
 *  gazeboscene_plugin.h - Fawkes Gazebo Scene Reconstruction Plugin
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

#ifndef __PLUGINS_GAZEBOSCENE_GAZEBOSCENE_PLUGIN_H_
#define __PLUGINS_GAZEBOSCENE_GAZEBOSCENE_PLUGIN_H_

#include <core/plugin.h>

class GazeboScenePlugin : public fawkes::Plugin
{
 public:
  GazeboScenePlugin(fawkes::Configuration *config);
};

#endif
