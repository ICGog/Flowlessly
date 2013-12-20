#include "graph.h"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <stack>

namespace flowlessly {

  using boost::algorithm::is_any_of;
  using boost::lexical_cast;
  using boost::token_compress_on;

  void Graph::allocateGraphMemory(uint32_t num_nodes) {
    arcs.resize(num_nodes + 1);
    admisible_arcs.resize(num_nodes + 1);
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
          int32_t arc_min_flow = lexical_cast<int32_t>(vals[3]);
          uint32_t arc_capacity = lexical_cast<uint32_t>(vals[4]);
          int64_t arc_cost = lexical_cast<int64_t>(vals[5]);
          Arc* arc = new Arc(src_node, dst_node, arc_capacity, arc_cost, NULL);
          Arc* reverse_arc = new Arc(dst_node, src_node, 0, -arc_cost, arc);
          arc->set_reverse_arc(reverse_arc);
          arcs[src_node][dst_node] = arc;
          arcs[dst_node][src_node] = reverse_arc;
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
          allocateGraphMemory(num_nodes);
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

  bool Graph::checkFlow(const string& flow_file_path) {
    FILE* flow_file = NULL;
    if ((flow_file = fopen(flow_file_path.c_str(), "r")) == NULL) {
      LOG(ERROR) << "Failed to open flow file: " << flow_file_path;
      return false;
    }
    uint32_t line_num = 0;
    char line[100];
    vector<string> vals;
    int64_t claimed_min_cost;
    while (!feof(flow_file)) {
      if (fscanf(flow_file, "%[^\n]%*[\n]", &line[0]) > 0) {
        ++line_num;
        boost::split(vals, line, is_any_of(" "), token_compress_on);
        if (vals[0].compare("f") == 0) {
          uint32_t src_node = lexical_cast<uint32_t>(vals[1]);
          uint32_t dst_node = lexical_cast<uint32_t>(vals[2]);
          int32_t arc_flow = lexical_cast<int32_t>(vals[3]);
          arcs[src_node][dst_node]->cap =
            arcs[src_node][dst_node]->initial_cap - arc_flow;
          nodes_demand[src_node] -= arc_flow;
          arcs[dst_node][src_node]->cap = arc_flow;
          nodes_demand[dst_node] += arc_flow;
        } else if (vals[0].compare("s") == 0) {
          claimed_min_cost = lexical_cast<int64_t>(vals[1]);
        } else if (vals[0].compare("c") == 0) {
          // Comment line. Ignore it.
        } else {
          LOG(ERROR) << "The file doesn't respect the DIMACS format on line: "
                     << line_num;
        }
      }
    }
    fclose(flow_file);
    // Check all the demand has been drained.
    int64_t actual_min_cost = 0;
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      if (nodes_demand[node_id] != 0) {
        LOG(ERROR) << "The demand has not been drained";
        return false;
      }
      for (map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
           it != arcs[node_id].end(); ++it) {
        if (it->second->cap < it->second->initial_cap) {
          claimed_min_cost -= (it->second->initial_cap - it->second->cap) *
            it->second->cost;
          actual_min_cost += (it->second->initial_cap - it->second->cap) *
            it->second->cost;
        }
      }
    }
    LOG(INFO) << "Min cost of the flow graph: " << actual_min_cost;
    if (claimed_min_cost != 0) {
      LOG(ERROR) << "The claimed min cost does not represent the cost of the "
                 << "flow graph";
      return false;
    }
    return true;
  }

  void Graph::writeGraph(const string& out_graph_file, int64_t scale_down) {
    int64_t min_cost = 0;
    FILE *graph_file = NULL;
    if ((graph_file = fopen(out_graph_file.c_str(), "w")) == NULL) {
      LOG(ERROR) << "Could no open graph file for writing: " << out_graph_file;
    }
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
      map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
        if (it->second->cap < it->second->initial_cap) {
          int32_t flow = it->second->initial_cap - it->second->cap;
          fprintf(graph_file, "f %u %u %d\n",
                  node_id, it->first, flow);
          min_cost += flow * it->second->cost;
        }
      }
    }
    fprintf(graph_file, "s %jd\n", min_cost / scale_down);
    fclose(graph_file);
  }

  void Graph::logGraph() {
    int64_t min_cost = 0;
    LOG(INFO) << "src dst flow cap cost";
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
      map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
        int32_t flow = it->second->initial_cap - it->second->cap;
        LOG(INFO) << "f " << node_id << " " << it->first << " "
                  << flow << " " << it->second->initial_cap << " "
                  << it->second->cost;
        if (flow > 0) {
          min_cost += flow * it->second->cost;
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

  vector<map<uint32_t, Arc*> >& Graph::get_arcs() {
    return arcs;
  }

  vector<map<uint32_t, Arc*> >& Graph::get_admisible_arcs() {
    return admisible_arcs;
  }

  list<Arc*>& Graph::get_fixed_arcs() {
    return fixed_arcs;
  }

  vector<uint32_t>& Graph::get_source_nodes() {
    if (added_sink_and_source) {
      return single_source_node;
    } else {
      return source_nodes;
    }
  }

  vector<uint32_t>& Graph::get_sink_nodes() {
    if (added_sink_and_source) {
      return single_sink_node;
    } else {
      return sink_nodes;
    }
  }

  vector<int32_t>& Graph::get_nodes_demand() {
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
      Arc* arc = new Arc(num_nodes - 1, *it, nodes_demand[*it], 0, NULL);
      Arc* reverse_arc = new Arc(*it, num_nodes - 1, 0, 0, arc);
      arc->set_reverse_arc(reverse_arc);
      arcs[num_nodes - 1][*it] = arc;
      arcs[*it][num_nodes - 1] = reverse_arc;
      nodes_demand[num_nodes - 1] += nodes_demand[*it];
      nodes_demand[*it] = 0;
    }
    for (vector<uint32_t>::iterator it = sink_nodes.begin();
         it != sink_nodes.end(); ++it) {
      Arc* arc = new Arc(num_nodes, *it, 0, 0, NULL);
      Arc* reverse_arc = new Arc(*it, num_nodes, -nodes_demand[*it], 0, arc);
      arc->set_reverse_arc(reverse_arc);
      arcs[num_nodes][*it] = arc;
      arcs[*it][num_nodes] = reverse_arc;
      nodes_demand[num_nodes] += nodes_demand[*it];
      nodes_demand[*it] = 0;
    }
  }

  void Graph::removeSinkAndSource() {
    map<uint32_t, Arc*>::iterator it = arcs[num_nodes - 1].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[num_nodes - 1].end();
    for (; it != end_it; ++it) {
      nodes_demand[it->first] += it->second->initial_cap - it->second->cap;
    }
    for (vector<uint32_t>::iterator it = source_nodes.begin();
         it != source_nodes.end(); ++it) {
      arcs[*it].erase(num_nodes - 1);
    }
    for (vector<uint32_t>::iterator it = sink_nodes.begin();
         it != sink_nodes.end(); ++it) {
      Arc* arc = arcs[*it][num_nodes];
      nodes_demand[*it] = -arc->cap;
      arcs[*it].erase(num_nodes);
    }
    added_sink_and_source = false;
    num_nodes -= 2;
    arcs.pop_back();
    arcs.pop_back();
    nodes_demand.pop_back();
    nodes_demand.pop_back();
    single_source_node.pop_back();
    single_sink_node.pop_back();
  }

  // Construct a topological order of the admisible graph.
  bool Graph::orderTopologically(vector<int64_t>& potentials,
                                 vector<uint32_t>& ordered) {
    vector<uint32_t>& source_nodes = get_source_nodes();
    vector<bool> has_in_vertex(num_nodes + 1, false);
    stack<uint32_t> to_visit;
    // 0 - node not visited.
    // 1 - node visited but didn't finished yet all its subtrees.
    // 2 - node visited completly.
    vector<uint8_t> marked(num_nodes + 1, 0);
    // Push the nodes with a 0 in-degree.
    // TODO(ionel): Think how it can be optimized.
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      map<uint32_t, Arc*>::const_iterator it = admisible_arcs[node_id].begin();
      map<uint32_t, Arc*>::const_iterator end_it =
        admisible_arcs[node_id].end();
      for (; it != end_it; ++it) {
        has_in_vertex[it->first] = true;
      }
    }
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      if (!has_in_vertex[node_id]) {
        to_visit.push(node_id);
      }
    }
    while (!to_visit.empty()) {
      uint32_t node_id = to_visit.top();
      to_visit.pop();
      // If marked temporarly then we have a cycle.
      if (marked[node_id] == 1) {
        // Return partial result.
        return false;
      } else {
        marked[node_id] = 1;
        ordered.push_back(node_id);
        map<uint32_t, Arc*>::const_iterator it =
          admisible_arcs[node_id].begin();
        map<uint32_t, Arc*>::const_iterator end_it =
          admisible_arcs[node_id].end();
        for (; it != end_it; ++it) {
          if (marked[it->first] == 0) {
            to_visit.push(it->first);
          }
        }
        marked[node_id] = 2;
      }
    }
    return true;
  }

}
