#include "successive_shortest.h"

#include "utils.h"

#include <limits>

namespace flowlessly {

  using namespace std;

  void SuccessiveShortest::reduceCost(const vector<int64_t>& distance) {
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
      for (map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
           it != end_it; ++it) {
        Arc* arc = it->second;
        if (arc->cap > 0) {
          arc->cost += distance[node_id] - distance[it->first];
        } else {
          arc->cost = 0;
        }
      }
    }
  }

  void SuccessiveShortest::successiveShortestPath() {
    //    Transform network G by adding source and sink
    //    Initial flow x is zero
    //        while ( Gx contains a path from s to t ) do
    //        Find any shortest path P from s to t
    //        Augment current flow x along P
    //        update Gx
    if (!graph_.hasSinkAndSource()) {
      graph_.addSinkAndSource();
    }
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    const set<uint32_t> source_node = graph_.get_source_nodes();
    // Works with the assumption that there's only a sink and a source node.
    const uint32_t sink_node = *(graph_.get_sink_nodes().begin());
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    do {
      fill(distance.begin(), distance.end(), numeric_limits<int64_t>::max());
      fill(predecessor.begin(), predecessor.end(), 0);
      BellmanFord(graph_, source_node, distance, predecessor);
      if (distance[sink_node] < numeric_limits<int64_t>::max()) {
        int32_t min_flow = numeric_limits<int32_t>::max();
        for (uint32_t cur_node = sink_node; cur_node != *(source_node.begin());
             cur_node = predecessor[cur_node]) {
          Arc* arc = arcs[predecessor[cur_node]][cur_node];
          min_flow = min(min_flow, arc->cap);
        }
        for (uint32_t cur_node = sink_node; cur_node != *(source_node.begin());
             cur_node = predecessor[cur_node]) {
          Arc* arc = arcs[predecessor[cur_node]][cur_node];
          arc->cap -= min_flow;
          arc->reverse_arc->cap += min_flow;
          nodes_demand[predecessor[cur_node]] -= min_flow;
          nodes_demand[cur_node] += min_flow;
        }
      }
    } while (distance[sink_node] < numeric_limits<int64_t>::max());
    graph_.removeSinkAndSource();
  }

  void SuccessiveShortest::successiveShortestPathPotentials() {
    //    Transform network G by adding source and sink
    //    Initial flow x is zero
    //    Use Bellman-Ford's algorithm to establish potentials PI
    //    Reduce Cost ( PI )
    //    while ( Gx contains a path from s to t ) do
    //        Find any shortest path P from s to t
    //        Reduce Cost ( PI )
    //        Augment current flow x along P
    //        update Gx
    if (!graph_.hasSinkAndSource()) {
      graph_.addSinkAndSource();
    }
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    // Works with the assumption that there's only a source and sink node.
    const set<uint32_t>& source_node = graph_.get_source_nodes();
    const uint32_t sink_node = *(graph_.get_sink_nodes().begin());
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    BellmanFord(graph_, source_node, distance, predecessor);
    reduceCost(distance);
    do {
      fill(distance.begin(), distance.end(), numeric_limits<int64_t>::max());
      fill(predecessor.begin(), predecessor.end(), 0);
      DijkstraOptimized(graph_, source_node, distance, predecessor);
      if (distance[sink_node] < numeric_limits<int64_t>::max()) {
        reduceCost(distance);
        int32_t min_flow = numeric_limits<int32_t>::max();
        for (uint32_t cur_node = sink_node; cur_node != *(source_node.begin());
             cur_node = predecessor[cur_node]) {
          Arc* arc = arcs[predecessor[cur_node]][cur_node];
          min_flow = min(min_flow, arc->cap);
        }
        for (uint32_t cur_node = sink_node; cur_node != *(source_node.begin());
             cur_node = predecessor[cur_node]) {
          Arc* arc = arcs[predecessor[cur_node]][cur_node];
          arc->cap -= min_flow;
          arc->reverse_arc->cap += min_flow;
          nodes_demand[predecessor[cur_node]] -= min_flow;
          nodes_demand[cur_node] += min_flow;
        }
      }
    } while (distance[sink_node] < numeric_limits<int64_t>::max());
    graph_.removeSinkAndSource();
  }

}
