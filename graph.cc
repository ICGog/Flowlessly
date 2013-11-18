#include "graph.h"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>

using boost::algorithm::is_any_of;
using boost::lexical_cast;
using boost::token_compress_on;

void Graph::allocateGraphMemory(uint32_t num_nodes, uint32_t num_arcs) {
  node_supply = new uint32_t[num_nodes + 1];
  arcs = new map<uint32_t, Arc>[num_nodes + 1];
}

void Graph::readData(const string& graph_file_path) {
  FILE* graph_file = NULL;
  if ((graph_file = fopen(graph_file_path.c_str(), "r")) == NULL) {
    LOG(ERROR) << "Failed to open graph file: " << graph_file_path;
    return;
  }
  char line[100];
  uint32_t line_num = 0;
  vector<string> vals;
  while (!feof(graph_file)) {
    if (fscanf(graph_file, "%[^\n]%*[\n]", &line[0]) > 0) {
      line_num++;
      boost::split(vals, line, is_any_of(" "), token_compress_on);
      if (vals[0].compare("a") == 0) {
        uint32_t src_node = lexical_cast<uint32_t>(vals[1]);
        uint32_t dst_node = lexical_cast<uint32_t>(vals[2]);
        int32_t arc_min_flow = lexical_cast<uint32_t>(vals[3]);
        uint32_t arc_capacity = lexical_cast<uint32_t>(vals[4]);
        int32_t arc_cost = lexical_cast<uint32_t>(vals[5]);
        arcs[src_node][dst_node] = Arc(arc_capacity, arc_min_flow, arc_cost);
        arcs[dst_node][src_node] = Arc(0, -arc_min_flow, -arc_cost);
      } else if (vals[0].compare("n") == 0) {
        uint32_t node_id = lexical_cast<uint32_t>(vals[1]);
        node_supply[node_id] = lexical_cast<uint32_t>(vals[2]);
      } else if (vals[0].compare("p") == 0) {
        num_nodes = lexical_cast<uint32_t>(vals[2]);
        num_arcs = lexical_cast<uint32_t>(vals[3]);
        allocateGraphMemory(num_nodes, num_arcs);
      } else if (vals[0].compare("c") == 0) {
        // Comment line. Ignore it.
      } else {
        LOG(ERROR) << "The file doesn't respect the DIMACS format on line: "
                   << line_num;
      }
    }
  }
  fclose(graph_file);
}

uint32_t Graph::get_num_nodes() {
  return num_nodes;
}

uint32_t Graph::get_num_arcs() {
  return num_arcs;
}

map<uint32_t, Arc>* Graph::get_arcs() {
  return arcs;
}
