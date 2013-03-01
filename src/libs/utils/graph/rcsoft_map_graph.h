
/***************************************************************************
 *  map_graph.h - Map graph for storing pathplan information
 *
 *  Created: Tue Jun 30 09:25:09 2009 (RoboCup 2009, Graz)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: rcsoft_map_graph.h 2710 2009-06-30 12:47:20Z tim $
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

#ifndef __UTILS_GRAPH_RCSOFT_MAP_GRAPH_H_
#define __UTILS_GRAPH_RCSOFT_MAP_GRAPH_H_

#include <utils/graph/topological_map_graph.h>

namespace xmlpp {
  class DomParser;
  class Node;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

extern TopologicalMapGraph *  load_rcsoft_graph(std::string filename);

} // end of namespace fawkes

#endif
