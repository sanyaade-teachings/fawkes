
/***************************************************************************
 *  colli_plugin.cpp - Fawkes Colli Plugin
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013  AllemaniACs
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

#include "colli_thread.h"

#ifdef HAVE_VISUAL_DEBUGGING
#include "visualization_thread.h"
//#include "navigation_thread.h"
#endif

using namespace fawkes;

class ColliPlugin : public fawkes::Plugin
{
 public:
  ColliPlugin(Configuration *config) : Plugin(config)
  {
    // thread_list.push_back(new ColliThread());
    ColliThread *collithr = new ColliThread();
    thread_list.push_back(collithr);
    /* #ifdef HAVE_VISUAL_DEBUGGING
    ColliNavigationThread *navthr = new ColliNavigationThread();
    collithr->set_navigation_thread(navthr);
    thread_list.push_back(navthr);
    #endif*/
    #ifdef HAVE_VISUAL_DEBUGGING
    ColliVisualizationThread *visthr = new ColliVisualizationThread();
    collithr->set_visualization_thread(visthr);
    thread_list.push_back(visthr);
    #endif
  }
};

PLUGIN_DESCRIPTION("plugin for collision avoidance")
EXPORT_PLUGIN(ColliPlugin)
