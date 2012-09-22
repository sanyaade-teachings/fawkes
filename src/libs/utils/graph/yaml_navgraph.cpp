
/***************************************************************************
 *  yaml_navgraph.cpp - Nav graph stored in a YAML file
 *
 *  Created: Fri Sep 21 18:37:16 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include <utils/graph/yaml_navgraph.h>
#include <utils/graph/topological_map_graph.h>
#include <core/exception.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Read topological map node from YAML iterator.
 * @param n iterator to node representing a topological map graph node
 * @param node node to fill
 */
static void
operator >> (const YAML::Iterator& n, TopologicalMapNode &node) {
  std::string name;
  n.first() >> name;

  const YAML::Node &yamlnode = n.second();

  if (yamlnode.Type() != YAML::NodeType::Map) {
    throw Exception("Node %s is not a map!?", name.c_str());
  }

  try {
    if (yamlnode["pos"].size() != 2) {
      throw Exception("Invalid position for node %s, "
                      "must be list of [x,y] coordinates", name.c_str());
    }
    float x, y;
    yamlnode["pos"][0] >> x;
    yamlnode["pos"][1] >> y;

    node.set_x(x);
    node.set_y(y);
  } catch (YAML::Exception &e) {
    throw fawkes::Exception("Failed to parse node: %s", e.what());
  }

  try {
    const YAML::Node &props = yamlnode["properties"];
    std::map<std::string, std::string> properties;

    YAML::Iterator p;
    for (p = props.begin(); p != props.end(); ++p) {
      if (p->Type() == YAML::NodeType::Scalar) {
        std::string key;
        *p >> key;
        node.set_property(key, "true");
      } else if (p->Type() == YAML::NodeType::Map) {
        for (YAML::Iterator i = p->begin(); i != p->end(); ++i) {
          std::string key, value;
          i.first() >> key;
          i.second() >> value;
          node.set_property(key, value);
        }
      } else {
        throw Exception("Invalid property for node '%s'", name.c_str());
      }
    }
    
  } catch (YAML::Exception &e) {
    //printf("Parsing failure: %s\n", e.what());
  } // ignored

  node.set_name(name);
}

/** Read topological map edge from YAML iterator.
 * @param n iterator to node representing a topological map graph edge
 * @param edge edge to fill
 */
static void
operator >> (const YAML::Iterator& n, TopologicalMapEdge &edge) {
  if (n->Type() != YAML::NodeType::Sequence || n->size() != 2) {
    throw Exception("Invalid edge");
  }
  std::string from, to;
  (*n)[0] >> from;
  (*n)[1] >> to;

  edge.set_from(from);
  edge.set_to(to);

  if (n->Tag() == "tag:fawkesrobotics.org,navgraph/dir") {
    edge.set_directed(true);
  }
}



/** Load topological map graph stored in RCSoft format.
 * @param filename path to the file to read
 * @return topological map graph read from file
 * @exception Exception thrown on any error to read the graph file
 */
TopologicalMapGraph *
load_yaml_navgraph(std::string filename)
{
  std::ifstream fin(filename.c_str());
  YAML::Parser parser(fin);

  YAML::Node doc;

  if (! parser.GetNextDocument(doc)) {
    throw fawkes::Exception("Failed to read YAML file %s", filename.c_str());
  }

  std::string graph_name;
  doc["graph-name"] >> graph_name;

  TopologicalMapGraph *graph = new TopologicalMapGraph(graph_name);


  YAML::Iterator n;
  for (n = doc["nodes"].begin(); n != doc["nodes"].end(); ++n) {
    std::string name;
    n.first() >> name;
    TopologicalMapNode node;
    n >> node;
    graph->add_node(node);
  }

  YAML::Iterator e;
  const YAML::Node &edges = doc["connections"];
  for (e = edges.begin(); e != edges.end(); ++e) {
    TopologicalMapEdge edge;
    e >> edge;
    graph->add_edge(edge);
  }

  graph->calc_reachability();
  return graph;
}

} // end of namespace fawkes
