#include "graph.h"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>

using boost::algorithm::is_any_of;
using boost::lexical_cast;
using boost::token_compress_on;

void Graph::allocateGraphMemory(uint32_t num_nodes, uint32_t num_arcs) {
  arcs.resize(num_nodes + 1);
  nodes_demand.resize(num_nodes + 1);
}

void Graph::readGraph(const string& graph_file_path) {
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
        arcs[src_node][dst_node] =
          new Arc(arc_capacity, arc_min_flow, arc_cost, false);
        arcs[dst_node][src_node] = new Arc(0, -arc_min_flow, -arc_cost, true);
      } else if (vals[0].compare("n") == 0) {
        uint32_t node_id = lexical_cast<uint32_t>(vals[1]);
        nodes_demand[node_id] = lexical_cast<int32_t>(vals[2]);
        if (nodes_demand[node_id] > 0) {
          source_nodes.push_back(node_id);
        } else if (nodes_demand[node_id] < 0) {
          sink_nodes.push_back(node_id);
        }
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

void Graph::writeGraph(const string& out_graph_file) {
  int32_t min_cost = 0;
  FILE *graph_file = NULL;
  if ((graph_file = fopen(out_graph_file.c_str(), "w")) == NULL) {
    LOG(ERROR) << "Could no open graph file for writing: " << out_graph_file;
  }
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->flow > 0) {
        fprintf(graph_file, "f %u %u %d\n",
                node_id, it->first, it->second->flow);
        min_cost += it->second->flow * it->second->cost;
      }
    }
  }
  fprintf(graph_file, "s %d\n", min_cost);
  fclose(graph_file);
}

void Graph::logGraph() {
  int32_t min_cost = 0;
  LOG(INFO) << "src dst flow cap cost";
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      LOG(INFO) << "f " << node_id << " " << it->first << " "
                << it->second->flow << " " << it->second->cap << " "
                << it->second->cost;
      if (it->second->flow > 0) {
        min_cost += it->second->flow * it->second->cost;
      }
    }
  }
  LOG(INFO) << "s " << min_cost;
}

uint32_t Graph::get_num_nodes() {
  return num_nodes;
}

uint32_t Graph::get_num_arcs() {
  return num_arcs;
}

const vector<map<uint32_t, Arc*> >& Graph::get_arcs() const {
  return arcs;
}

const vector<uint32_t>& Graph::get_source_nodes() const {
  if (added_sink_and_source) {
    return single_source_node;
  } else {
    return source_nodes;
  }
}

const vector<uint32_t>& Graph::get_sink_nodes() const {
  if (added_sink_and_source) {
    return single_sink_node;
  } else {
    return sink_nodes;
  }
}

const vector<int32_t>& Graph::get_nodes_demand() const {
  return nodes_demand;
}

bool Graph::hasSinkAndSource() {
  return added_sink_and_source;
}

void Graph::addSinkAndSource() {
  added_sink_and_source = true;
  num_nodes += 2;
  arcs.resize(num_nodes + 1);
  nodes_demand.resize(num_nodes + 1);
  single_source_node.push_back(num_nodes - 1);
  single_sink_node.push_back(num_nodes);
  for (vector<uint32_t>::iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    arcs[num_nodes - 1][*it] =
      new Arc(nodes_demand[*it], 0, 0, false);
    arcs[*it][num_nodes - 1] = new Arc(0, 0, 0, true);
    nodes_demand[num_nodes - 1] += nodes_demand[*it];
  }
  for (vector<uint32_t>::iterator it = sink_nodes.begin();
       it != sink_nodes.end(); ++it) {
    arcs[num_nodes][*it] = new Arc(0, 0, 0, true);
    arcs[*it][num_nodes] =
      new Arc(-nodes_demand[*it], 0, 0, false);
    nodes_demand[num_nodes] += nodes_demand[*it];
  }
}

void Graph::removeSinkAndSource() {
  added_sink_and_source = false;
  num_nodes -= 2;
  arcs.pop_back();
  arcs.pop_back();
  nodes_demand.pop_back();
  nodes_demand.pop_back();
  single_source_node.pop_back();
  single_sink_node.pop_back();
  for (vector<uint32_t>::iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    arcs[*it].erase(num_nodes + 1);
  }
  for (vector<uint32_t>::iterator it = sink_nodes.begin();
       it != sink_nodes.end(); ++it) {
    arcs[*it].erase(num_nodes + 2);
  }
}
