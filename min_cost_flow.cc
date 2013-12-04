#include "min_cost_flow.h"

#include <algorithm>
#include <boost/heap/binomial_heap.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <limits>
#include <queue>
#include <stdint.h>

using namespace boost::heap;
using namespace std;

void MinCostFlow::logCosts(const vector<int32_t>& distance,
                             const vector<uint32_t>& predecessor) {
  LOG(INFO) << "Logging graph costs";
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    LOG(INFO) << node_id << " " << distance[node_id] << " "
              << predecessor[node_id] << endl;
  }
}

void MinCostFlow::BellmanFord(const vector<uint32_t>& source_nodes,
                              vector<int32_t>& distance,
                              vector<uint32_t>& predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  for (vector<uint32_t>::const_iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    distance[*it] = 0;
  }
  for (uint32_t iter = 1; iter < num_nodes; ++iter) {
    bool relaxed = false;
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      if (distance[node_id] < numeric_limits<int32_t>::max()) {
        map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
        map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
        for (; it != end_it; ++it) {
          if (it->second->cap - it->second->flow > 0 &&
              distance[node_id] + it->second->cost < distance[it->first]) {
            distance[it->first] = distance[node_id] + it->second->cost;
            predecessor[it->first] = node_id;
            relaxed = true;
          }
        }
      }
    }
    if (!relaxed) {
      break;
    }
  }
}

void MinCostFlow::augmentFlow(vector<int32_t>& distance,
                              vector<uint32_t>& predecessor,
                              uint32_t src_node, uint32_t dst_node) {
  LOG(INFO) << "Negative cycle closed by: (" << src_node << ", "
            << dst_node << ")";
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<bool> seen(num_nodes, false);
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  int32_t min_flow = numeric_limits<int32_t>::max();
  uint32_t cur_node = src_node;
  for (; !seen[cur_node];
       seen[cur_node] = true, cur_node = predecessor[cur_node]);
  dst_node = cur_node;
  do {
    Arc* arc = arcs[predecessor[cur_node]][cur_node];
    if (arc->cap - arc->flow < min_flow) {
      min_flow = arc->cap - arc->flow;
    }
    LOG(INFO) << "Negative cycle: (" << predecessor[cur_node] << ", "
              << cur_node << ")";
    cur_node = predecessor[cur_node];
  } while (cur_node != dst_node);
  LOG(INFO) << "Augmenting negative cycle with flow: " << min_flow;
  do {
    arcs[predecessor[cur_node]][cur_node]->flow += min_flow;
    arcs[cur_node][predecessor[cur_node]]->flow -= min_flow;
    cur_node = predecessor[cur_node];
  } while (cur_node != dst_node);
}

bool MinCostFlow::removeNegativeCycles(vector<int32_t>& distance,
                                       vector<uint32_t>& predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cap - it->second->flow > 0 &&
          distance[node_id] + it->second->cost < distance[it->first]) {
        // Found negative cycle.
        augmentFlow(distance, predecessor, node_id, it->first);
        return true;
      }
    }
  }
  return false;
}

void MinCostFlow::DijkstraSimple(const vector<uint32_t>& source_nodes,
                                 vector<int32_t>& distance,
                                 vector<uint32_t>& predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<bool> node_used(num_nodes, false);
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  for (vector<uint32_t>::const_iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    distance[*it] = 0;
  }
  for (uint32_t iter = 1; iter < num_nodes - 1; ++iter) {
    uint32_t min_node_distance = numeric_limits<int32_t>::max();
    uint32_t min_node_id = 0;
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      if (!node_used[node_id] && distance[node_id] <= min_node_distance) {
        min_node_distance = distance[node_id];
        min_node_id = node_id;
      }
    }
    node_used[min_node_id] = true;
    map<uint32_t, Arc*>::iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[min_node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cap - it->second->flow > 0 &&
          distance[min_node_id] + it->second->cost < distance[it->first]) {
        distance[it->first] = distance[min_node_id] + it->second->cost;
        predecessor[it->first] = min_node_id;
      }
    }
  }
}


void MinCostFlow::DijkstraOptimized(const vector<uint32_t>& source_nodes,
                                    vector<int32_t>& distance,
                                    vector<uint32_t>& predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  vector<bool> visited(num_nodes, false);
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > > dist_heap;
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > >::handle_type
                handles[num_nodes];
  for (vector<uint32_t>::const_iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    distance[*it] = 0;
    visited[*it] = true;
    handles[*it] = dist_heap.push(make_pair(0, *it));
  }
  while (!dist_heap.empty()) {
    pair<int32_t, uint32_t> min_dist = dist_heap.top();
    int32_t min_cost = min_dist.first;
    uint32_t min_node_id = min_dist.second;
    LOG(INFO) << min_node_id;
    dist_heap.pop();
    map<uint32_t, Arc*>::iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[min_node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cap - it->second->flow > 0 &&
          distance[min_node_id] + it->second->cost < distance[it->first]) {
        distance[it->first] = distance[min_node_id] + it->second->cost;
        if (!visited[it->first]) {
          visited[it->first] = true;
          predecessor[it->first] = min_node_id;
          handles[it->first] =
            dist_heap.push(make_pair(distance[it->first], it->first));
        } else {
          predecessor[it->first] = min_node_id;
          dist_heap.increase(handles[it->first],
                             make_pair(distance[it->first], it->first));
        }
      }
    }
  }
}

// Computes Max Flow over the graph using the Ford-Fulkerson algorithm.
// The Complexity of the algorithm is O(E * F). Where F is the max flow value.
// NOTE: This changes the graph.
void MinCostFlow::maxFlow() {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  vector<int32_t> nodes_descriptor = graph_.get_nodes_descriptor();
  vector<int32_t> visited(num_nodes, 0);
  vector<uint32_t> predecessor(num_nodes, 0);
  uint32_t source_node = graph_.get_source_id();
  uint32_t sink_node = graph_.get_sink_id();
  bool has_path = true;
  while (has_path) {
    has_path = false;
    queue<uint32_t> to_visit;
    fill(visited.begin(), visited.end(), 0);
    fill(predecessor.begin(), predecessor.end(), 0);
    to_visit.push(source_node);
    visited[source_node] = nodes_descriptor[source_node];
    while (!to_visit.empty() && !has_path) {
      uint32_t cur_node = to_visit.front();
      LOG(INFO) << "Max flow node popped: " << cur_node;
      to_visit.pop();
      map<uint32_t, Arc*>::iterator it = arcs[cur_node].begin();
      map<uint32_t, Arc*>::iterator end_it = arcs[cur_node].end();
      for (; it != end_it; ++it) {
        if (!visited[it->first] && it->second->cap - it->second->flow > 0) {
          visited[it->first] = min(it->second->cap - it->second->flow,
                                   visited[cur_node]);
          to_visit.push(it->first);
          predecessor[it->first] = cur_node;
          if (it->first == sink_node) {
            has_path = true;
            int32_t min_aux_flow = visited[it->first];
            for (uint32_t cur_node = it->first; predecessor[cur_node] > 0;
                 cur_node = predecessor[cur_node]) {
              arcs[predecessor[cur_node]][cur_node]->flow += min_aux_flow;
              arcs[cur_node][predecessor[cur_node]]->flow -= min_aux_flow;
              LOG(INFO) << "Flow path: (" << predecessor[cur_node] << ", "
                        << cur_node << ") " << min_aux_flow;
            }
            break;
          }
        }
      }
    }
    LOG(INFO) << "The graph after another iteration of max flow.";
    graph_.logGraph();
  }
}

// Applies the Cycle cancelling algorithm to compute the min cost flow.
// The complexity is O(F * M + N * M^2 * C * U)
// NOTE: It changes the graph.
void MinCostFlow::cycleCancelling() {
  //    Establish a feasible flow x in the network
  //    while ( Gx contains a negative cycle ) do
  //        identify a negative cycle W
  //        mr = min(r(i,j)) where (i,j) is part of W
  //        augment mr units of flow along the cycle W
  //        update Gx
  if (!graph_.hasSinkAndSource()) {
    graph_.addSinkAndSource();
  }
  maxFlow();
  graph_.removeSinkAndSource();
  graph_.logGraph();
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  BellmanFord(graph_.get_supply_nodes(), distance, predecessor);
  logCosts(distance, predecessor);
  bool removed_cycle = removeNegativeCycles(distance, predecessor);
  graph_.logGraph();
  while (removed_cycle) {
    fill(distance.begin(), distance.end(), numeric_limits<int32_t>::max());
    fill(predecessor.begin(), predecessor.end(), 0);
    BellmanFord(graph_.get_supply_nodes(), distance, predecessor);
    logCosts(distance, predecessor);
    removed_cycle = removeNegativeCycles(distance, predecessor);
    graph_.logGraph();
  }
}

void MinCostFlow::successiveShortestPath() {
  //    Transform network G by adding source and sink
  //    Initial flow x is zero
  //        while ( Gx contains a path from s to t ) do
  //        Find any shortest path P from s to t
  //        Augment current flow x along P
  //        update Gx
  if (!graph_.hasSinkAndSource()) {
    graph_.addSinkAndSource();
  }
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  vector<uint32_t> source_node(1, graph_.get_source_id());
  uint32_t sink_node = graph_.get_sink_id();
  do {
    fill(distance.begin(), distance.end(), numeric_limits<int32_t>::max());
    fill(predecessor.begin(), predecessor.end(), 0);
    BellmanFord(source_node, distance, predecessor);
    if (distance[sink_node] < numeric_limits<int32_t>::max()) {
      uint32_t min_flow = numeric_limits<int32_t>::max();
      for (uint32_t cur_node = sink_node; cur_node != source_node[0];
           cur_node = predecessor[cur_node]) {
        Arc* arc = arcs[predecessor[cur_node]][cur_node];
        if (arc->cap - arc->flow < min_flow) {
          min_flow = arc->cap - arc->flow;
        }
      }
      for (uint32_t cur_node = sink_node; cur_node != source_node[0];
           cur_node = predecessor[cur_node]) {
        arcs[predecessor[cur_node]][cur_node]->flow += min_flow;
        arcs[cur_node][predecessor[cur_node]]->flow -= min_flow;
      }
    }
  } while (distance[sink_node] < numeric_limits<int32_t>::max());
}

void MinCostFlow::reduceCost(vector<int32_t>& potential) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      Arc* arc = it->second;
      if (arc->cap - arc->flow > 0) {
        arc->cost += potential[node_id] - potential[it->first];
      } else {
        arc->cost = 0;
      }
    }
  }
}

void MinCostFlow::successiveShortestPathPotentials() {
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
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  vector<uint32_t> source_node(1, graph_.get_source_id());
  uint32_t sink_node = graph_.get_sink_id();
  BellmanFord(source_node, distance, predecessor);
  reduceCost(distance);
  uint32_t it_cnt = 0;
  do {
    it_cnt++;
    cout << "Iteration: " << it_cnt << endl;
    fill(distance.begin(), distance.end(), numeric_limits<int32_t>::max());
    fill(predecessor.begin(), predecessor.end(), 0);
    graph_.logGraph();
    DijkstraOptimized(source_node, distance, predecessor);
    logCosts(distance, predecessor);
    if (distance[sink_node] < numeric_limits<int32_t>::max()) {
      reduceCost(distance);
      uint32_t min_flow = numeric_limits<int32_t>::max();
      for (uint32_t cur_node = sink_node; cur_node != source_node[0];
           cur_node = predecessor[cur_node]) {
        Arc* arc = arcs[predecessor[cur_node]][cur_node];
        if (arc->cap - arc->flow < min_flow) {
          min_flow = arc->cap - arc->flow;
        }
      }
      for (uint32_t cur_node = sink_node; cur_node != source_node[0];
           cur_node = predecessor[cur_node]) {
        arcs[predecessor[cur_node]][cur_node]->flow += min_flow;
        arcs[cur_node][predecessor[cur_node]]->flow -= min_flow;
      }
    }
  } while (distance[sink_node] < numeric_limits<int32_t>::max());
}

void MinCostFlow::push(Arc* arc) {
  // Excess flow.
  // TODO(ionel): Compute excess flow.
  //  int32_t flow_to_send = min(excess_flow, arc->flow);
}

void MinCostFlow::relabel(vector<int32_t>& potentials, uint32_t node_id) {
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
  map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
  int32_t max_p_node = numeric_limits<int32_t>::min();
  /*
  for (; it != end_it; ++it) {
    max_p_node = max(max_p_node,
                     potentials[it->first] - it->second->cost - eps);
  }
  */
  potentials[node_id] = max_p_node;
}

void MinCostFlow::discharge(uint32_t node_id) {
}

/*
void MinCostFlow::firstActive() {
  stack<uint32_t> active_stack;
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    // Add the node to the stack if active.
    if (excess_flow[node_id] > 0) {
      active_stack.push(node_id);
    }
  }
  while (!active_stack.empty()) {
    bool relabeled = discharge(active_stack.top());
    if (!relabeled) {
      active_stack.pop();
    }
  }
}
*/

void MinCostFlow::refine(vector<int32_t>& potentials) {
  // Saturate arcs with negative reduced cost.
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cap + potentials[node_id] - potentials[it->first] < 0) {
        it->second->flow = it->second->cap;
      }
    }
  }

}

void MinCostFlow::costScaling() {
  //    eps = max arc cost
  //    potential(v) = 0
  //    Establish a feasible flow x in the network
  //    while eps >= 1/n do
  //      (e, f, p) = refine(e, f p)
  int32_t alpha = 2;
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  vector<int32_t> potentials(num_nodes, 0);
  // Get max cost arc.
  int32_t max_cost_arc = numeric_limits<int32_t>::min();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cost > max_cost_arc) {
        max_cost_arc;
      }
    }
  }
  if (!graph_.hasSinkAndSource()) {
    graph_.addSinkAndSource();
  }
  maxFlow();
  graph_.removeSinkAndSource();
  for (uint32_t eps = max_cost_arc; eps >= 1; eps /= alpha) {
    refine(potentials);
  }
}
