#include "cycle_cancelling.h"

#include "utils.h"

#include <limits>

namespace flowlessly {

  using namespace std;

  //    Establish a feasible flow x in the network
  //    while ( Gx contains a negative cycle ) do
  //        identify a negative cycle W
  //        mr = min(r(i,j)) where (i,j) is part of W
  //        augment mr units of flow along the cycle W
  //        update Gx
  void CycleCancelling::cycleCancelling(bool has_flow) {
    if (!has_flow) {
      if (!graph_.hasSinkAndSource()) {
        graph_.addSinkAndSource();
      }
      maxFlow(graph_);
      graph_.removeSinkAndSource();
    }
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    BellmanFord(graph_, graph_.get_source_nodes(), distance, predecessor);
    bool removed_cycle = removeNegativeCycles(distance, predecessor);
    while (removed_cycle) {
      fill(distance.begin(), distance.end(), numeric_limits<int64_t>::max());
      fill(predecessor.begin(), predecessor.end(), 0);
      BellmanFord(graph_, graph_.get_source_nodes(), distance, predecessor);
      removed_cycle = removeNegativeCycles(distance, predecessor);
    }
  }

  bool CycleCancelling::removeNegativeCycles(const vector<int64_t>& distance,
                                             const vector<uint32_t>& predecessor) {
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
      for (map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
           it != end_it; ++it) {
        if (it->second->cap > 0 &&
            distance[node_id] + it->second->cost < distance[it->first]) {
          // Found negative cycle.
          augmentFlow(predecessor, node_id, it->first);
          return true;
        }
      }
    }
    return false;
  }

  void CycleCancelling::augmentFlow(const vector<uint32_t>& predecessor,
                                    uint32_t src_node, uint32_t dst_node) {
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    vector<bool> seen(num_nodes, false);
    int32_t min_flow = numeric_limits<int32_t>::max();
    uint32_t cur_node = src_node;
    // Detect the node where the cycle ends.
    for (; !seen[cur_node];
         seen[cur_node] = true, cur_node = predecessor[cur_node]);
    dst_node = cur_node;
    // Compute the minimum residual in the cycle.
    do {
      Arc* arc = arcs[predecessor[cur_node]][cur_node];
      min_flow = min(min_flow, arc->cap);
      cur_node = predecessor[cur_node];
    } while (cur_node != dst_node);
    do {
      Arc* arc = arcs[predecessor[cur_node]][cur_node];
      arc->cap -= min_flow;
      arc->reverse_arc->cap += min_flow;
      nodes_demand[predecessor[cur_node]] -= min_flow;
      nodes_demand[cur_node] += min_flow;
      cur_node = predecessor[cur_node];
    } while (cur_node != dst_node);
  }

}
