#include "cycle_cancelling.h"

#include "utils.h"

#include <limits>

namespace flowlessly {

  using namespace std;

  // Applies the Cycle cancelling algorithm to compute the min cost flow.
  // The complexity is O(F * M + N * M^2 * C * U)
  // NOTE: It changes the graph.
  void CycleCancelling::cycleCancelling() {
    //    Establish a feasible flow x in the network
    //    while ( Gx contains a negative cycle ) do
    //        identify a negative cycle W
    //        mr = min(r(i,j)) where (i,j) is part of W
    //        augment mr units of flow along the cycle W
    //        update Gx
    if (!graph_.hasSinkAndSource()) {
      graph_.addSinkAndSource();
    }
    maxFlow(graph_);
    graph_.removeSinkAndSource();
    graph_.logGraph();
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    BellmanFord(graph_, graph_.get_source_nodes(), distance, predecessor);
    logCosts(distance, predecessor);
    bool removed_cycle = removeNegativeCycles(distance, predecessor);
    graph_.logGraph();
    while (removed_cycle) {
      fill(distance.begin(), distance.end(), numeric_limits<int64_t>::max());
      fill(predecessor.begin(), predecessor.end(), 0);
      BellmanFord(graph_, graph_.get_source_nodes(), distance, predecessor);
      logCosts(distance, predecessor);
      removed_cycle = removeNegativeCycles(distance, predecessor);
      graph_.logGraph();
    }
  }

  bool CycleCancelling::removeNegativeCycles(vector<int64_t>& distance,
                                             vector<uint32_t>& predecessor) {
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
      map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
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

  void CycleCancelling::augmentFlow(vector<uint32_t>& predecessor,
                                    uint32_t src_node, uint32_t dst_node) {
    LOG(INFO) << "Negative cycle closed by: (" << src_node << ", "
              << dst_node << ")";
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<bool> seen(num_nodes, false);
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
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
      LOG(INFO) << "Negative cycle: (" << predecessor[cur_node] << ", "
                << cur_node << ")";
      cur_node = predecessor[cur_node];
    } while (cur_node != dst_node);
    LOG(INFO) << "Augmenting negative cycle with flow: " << min_flow;
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
