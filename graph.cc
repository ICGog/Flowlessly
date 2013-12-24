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
    potential.resize(num_nodes + 1);
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
            source_nodes.insert(node_id);
          } else if (nodes_demand[node_id] < 0) {
            sink_nodes.insert(node_id);
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

  bool Graph::checkEpsOptimality(int64_t eps) {
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      for (map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
           it != arcs[node_id].end(); ++it) {
        if (it->second->cap > 0 && it->second->cost + potential[node_id] -
            potential[it->first] < -eps) {
          return false;
        }
      }
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

  void Graph::logAdmisibleGraph() {
    LOG(INFO) << "src dst residual_cap reduced_cost";
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      map<uint32_t, Arc*>::iterator it = admisible_arcs[node_id].begin();
      map<uint32_t, Arc*>::iterator end_it = admisible_arcs[node_id].end();
      for (; it != end_it; ++it) {
        int64_t reduced_cost = it->second->cost + potential[node_id] -
          potential[it->first];
        LOG(INFO) << node_id << " " << it->first << " " << it->second->cap
                  << " " << reduced_cost;
      }
    }
  }

  void Graph::logResidualGraph() {
    LOG(INFO) << "src dst residual_cap cost";
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      for (map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
           it != arcs[node_id].end(); ++it) {
        if (it->second->cap > 0) {
          LOG(INFO) << node_id << " " << it->first << " "
                    << it->second->cap << " " << it->second->cost;
        }
      }
    }
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

  set<uint32_t>& Graph::get_source_nodes() {
    if (added_sink_and_source) {
      return single_source_node;
    } else {
      return source_nodes;
    }
  }

  set<uint32_t>& Graph::get_sink_nodes() {
    if (added_sink_and_source) {
      return single_sink_node;
    } else {
      return sink_nodes;
    }
  }

  vector<int32_t>& Graph::get_nodes_demand() {
    return nodes_demand;
  }

  vector<int64_t>& Graph::get_potential() {
    return potential;
  }

  bool Graph::hasSinkAndSource() {
    return added_sink_and_source;
  }

  void Graph::addSinkAndSource() {
    added_sink_and_source = true;
    num_nodes += 2;
    arcs.resize(num_nodes + 1);
    nodes_demand.resize(num_nodes + 1);
    single_source_node.insert(num_nodes - 1);
    single_sink_node.insert(num_nodes);
    for (set<uint32_t>::iterator it = source_nodes.begin();
         it != source_nodes.end(); ++it) {
      Arc* arc = new Arc(num_nodes - 1, *it, nodes_demand[*it], 0, NULL);
      Arc* reverse_arc = new Arc(*it, num_nodes - 1, 0, 0, arc);
      arc->set_reverse_arc(reverse_arc);
      arcs[num_nodes - 1][*it] = arc;
      arcs[*it][num_nodes - 1] = reverse_arc;
      nodes_demand[num_nodes - 1] += nodes_demand[*it];
      nodes_demand[*it] = 0;
    }
    for (set<uint32_t>::iterator it = sink_nodes.begin();
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
    for (set<uint32_t>::iterator it = source_nodes.begin();
         it != source_nodes.end(); ++it) {
      arcs[*it].erase(num_nodes - 1);
    }
    for (set<uint32_t>::iterator it = sink_nodes.begin();
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
    single_source_node.clear();
    single_sink_node.clear();
  }

  // Construct a topological order of the admisible graph.
  bool Graph::orderTopologically(vector<uint32_t>& ordered) {
    stack<uint32_t> to_visit;
    // 0 - node not visited.
    // 1 - node visited but didn't finished yet all its subtrees.
    // 2 - node visited completly.
    vector<uint8_t> marked(num_nodes + 1, 0);
    list<uint32_t> ordered_list;
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      if (marked[node_id] == 0) {
        if (!visitTopologically(node_id, marked, ordered_list)) {
          return false;
        }
      }
    }
    for (list<uint32_t>::iterator it = ordered_list.begin();
         it != ordered_list.end(); ++it) {
      ordered.push_back(*it);
    }
    return true;
  }

  bool Graph::visitTopologically(uint32_t node_id, vector<uint8_t>& marked,
                                 list<uint32_t>& ordered) {
    if (marked[node_id] == 1) {
      return false;
    } else {
      marked[node_id] = 1;
      map<uint32_t, Arc*>::const_iterator it =
        admisible_arcs[node_id].begin();
      map<uint32_t, Arc*>::const_iterator end_it =
        admisible_arcs[node_id].end();
      for (; it != end_it; ++it) {
        if (marked[it->first] != 2 &&
            !visitTopologically(it->first, marked, ordered)) {
          return false;
        }
      }
      marked[node_id] = 2;
      ordered.push_front(node_id);
    }
    return true;
  }

  void Graph::removeNode(uint32_t node_id) {
    // Delete arcs in which node_id appears from the list of fixed
    // arcs.

    // TODO(ionel): Implement this. Currently we have to go over the entire
    // fixed_arc list. This is to inefficient.

    // Delete arcs for the arcs and admisible arcs.
    for (map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
         it != arcs[node_id].end(); ) {
      Arc* arc = it->second;
      if (arc->cap < arc->initial_cap) {
        // Forward arc.
        nodes_demand[arc->src_node_id] += arc->initial_cap - arc->cap;
        nodes_demand[arc->dst_node_id] -= arc->initial_cap - arc->cap;
      } else {
        // Reverse arc.
        nodes_demand[arc->src_node_id] -= arc->cap;
        nodes_demand[arc->dst_node_id] += arc->cap;
      }
      map<uint32_t, Arc*>::iterator to_erase_it = it;
      ++it;
      arcs[node_id].erase(to_erase_it);
      arcs[it->first].erase(node_id);
      admisible_arcs[node_id].erase(it->first);
      admisible_arcs[it->first].erase(node_id);
      --num_arcs;
      delete arc->reverse_arc;
      delete arc;
    }
    // Delete node from sink or source nodes set.
    if (nodes_demand[node_id] > 0) {
      source_nodes.erase(node_id);
    } else if (nodes_demand[node_id] < 0) {
      sink_nodes.erase(node_id);
    }
    deleted_nodes.push_back(node_id);
    nodes_demand[node_id] = 0;
    potential[node_id] = 0;
  }

  void Graph::removeNodes(vector<uint32_t>& node_ids) {
    for (vector<uint32_t>::iterator it = node_ids.begin();
         it != node_ids.end(); ++it) {
      removeNode(*it);
    }
  }

  uint32_t Graph::addNode(uint32_t node_id, int32_t node_demand,
                          int64_t node_potential,
                          vector<Arc*>& arcs_from_node) {
    uint32_t new_node_id;
    if (deleted_nodes.empty()) {
      ++num_nodes;
      new_node_id = num_nodes;
      potential.push_back(node_potential);
      nodes_demand.push_back(node_demand);
      arcs.push_back(map<uint32_t, Arc*>());
      admisible_arcs.push_back(map<uint32_t, Arc*>());
    } else {
      new_node_id = deleted_nodes.front();
      deleted_nodes.pop_front();
      potential[new_node_id] = node_potential;
      nodes_demand[new_node_id] = node_demand;
    }
    // TODO(ionel): Update fixed arcs.
    for (vector<Arc*>::iterator it = arcs_from_node.begin();
         it != arcs_from_node.end(); ++it) {
      Arc* arc = (*it);
      arc->src_node_id = new_node_id;
      arc->reverse_arc->dst_node_id = new_node_id;
      arcs[new_node_id][arc->dst_node_id] = arc;
      arcs[arc->dst_node_id][new_node_id] = arc->reverse_arc;
      ++num_arcs;
      int64_t reduced_cost = arc->cost + potential[arc->src_node_id] -
        potential[arc->dst_node_id];
      if (reduced_cost < 0 && arc->cap > 0) {
        admisible_arcs[arc->src_node_id][arc->dst_node_id] = arc;
      }
      if (-reduced_cost < 0 && arc->reverse_arc->cap > 0) {
        admisible_arcs[arc->dst_node_id][arc->src_node_id] = arc->reverse_arc;
      }
    }
    if (nodes_demand[new_node_id] > 0) {
      source_nodes.insert(new_node_id);
    } else if (nodes_demand[new_node_id] < 0) {
      sink_nodes.insert(new_node_id);
    }
    return new_node_id;
  }

}
