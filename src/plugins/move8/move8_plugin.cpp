
/***************************************************************************
 *  move8_plugin.cpp - Mi 23. Mai 17:44:14 CEST 2012
 *
 *  Created: Sat May 18 02:20:05 2013
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

#include <core/plugin.h>

#include "move8_thread.h"

using namespace fawkes;

/** Template! Makes the robotino move forward for 3 seconds
 * @author Daniel Ewert
 */
class Move8Plugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  Move8Plugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new Move8Thread());
  }
};

PLUGIN_DESCRIPTION("Move faehrt die 8")
EXPORT_PLUGIN(Move8Plugin)
