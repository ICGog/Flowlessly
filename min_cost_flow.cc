#include "min_cost_flow.h"

#include <algorithm>
#include <boost/heap/binomial_heap.hpp>
#include <cmath>
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
  const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  for (vector<uint32_t>::const_iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    distance[*it] = 0;
  }
  bool relaxed = true;
  for (uint32_t iter = 1; iter < num_nodes && relaxed; ++iter) {
    relaxed = false;
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
  }
}

void MinCostFlow::augmentFlow(vector<int32_t>& distance,
                              vector<uint32_t>& predecessor,
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
    if (arc->cap - arc->flow < min_flow) {
      min_flow = arc->cap - arc->flow;
    }
    LOG(INFO) << "Negative cycle: (" << predecessor[cur_node] << ", "
              << cur_node << ")";
    cur_node = predecessor[cur_node];
  } while (cur_node != dst_node);
  LOG(INFO) << "Augmenting negative cycle with flow: " << min_flow;
  do {
    Arc* arc = arcs[predecessor[cur_node]][cur_node];
    arc->flow += min_flow;
    arc->reverse_arc->flow -= min_flow;
    nodes_demand[predecessor[cur_node]] -= min_flow;
    nodes_demand[cur_node] += min_flow;
    cur_node = predecessor[cur_node];
  } while (cur_node != dst_node);
}

bool MinCostFlow::removeNegativeCycles(vector<int32_t>& distance,
                                       vector<uint32_t>& predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
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
  const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  // Works with the assumption that all the elements of distance are
  // already set to INF.
  for (vector<uint32_t>::const_iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    distance[*it] = 0;
  }
  for (uint32_t iter = 1; iter < num_nodes - 1; ++iter) {
    uint32_t min_node_distance = numeric_limits<int32_t>::max();
    uint32_t min_node_id = 0;
    // Get the closest unused vertex.
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      if (!node_used[node_id] && distance[node_id] <= min_node_distance) {
        min_node_distance = distance[node_id];
        min_node_id = node_id;
      }
    }
    node_used[min_node_id] = true;
    map<uint32_t, Arc*>::const_iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[min_node_id].end();
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
  const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  vector<bool> visited(num_nodes, false);
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > > dist_heap;
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > >::handle_type
                handles[num_nodes];
  // Works with the assumption that all the elements of distance are
  // already set to INF.
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
    map<uint32_t, Arc*>::const_iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[min_node_id].end();
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

// Computes max flow over the graph using the Ford-Fulkerson algorithm.
// The Complexity of the algorithm is O(E * F). Where F is the max flow value.
// NOTE: This method changes the graph.
void MinCostFlow::maxFlow() {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
  vector<int32_t> visited(num_nodes, 0);
  vector<uint32_t> predecessor(num_nodes, 0);
  // Works with the assumption that there is only a sink and a source node.
  uint32_t source_node = graph_.get_source_nodes()[0];
  uint32_t sink_node = graph_.get_sink_nodes()[0];
  bool has_path = true;
  while (has_path) {
    has_path = false;
    queue<uint32_t> to_visit;
    fill(visited.begin(), visited.end(), 0);
    fill(predecessor.begin(), predecessor.end(), 0);
    to_visit.push(source_node);
    visited[source_node] = nodes_demand[source_node];
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
              Arc* arc = arcs[predecessor[cur_node]][cur_node];
              arc->flow += min_aux_flow;
              arc->reverse_arc->flow -= min_aux_flow;
              nodes_demand[predecessor[cur_node]] -= min_aux_flow;
              nodes_demand[cur_node] += min_aux_flow;
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
  BellmanFord(graph_.get_source_nodes(), distance, predecessor);
  logCosts(distance, predecessor);
  bool removed_cycle = removeNegativeCycles(distance, predecessor);
  graph_.logGraph();
  while (removed_cycle) {
    fill(distance.begin(), distance.end(), numeric_limits<int32_t>::max());
    fill(predecessor.begin(), predecessor.end(), 0);
    BellmanFord(graph_.get_source_nodes(), distance, predecessor);
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
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  // Works with the assumption that there's only a sink and a source node.
  vector<uint32_t> source_node = graph_.get_source_nodes();
  uint32_t sink_node = graph_.get_sink_nodes()[0];
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
        Arc* arc = arcs[predecessor[cur_node]][cur_node];
        arc->flow += min_flow;
        arc->reverse_arc->flow -= min_flow;
        nodes_demand[predecessor[cur_node]] -= min_flow;
        nodes_demand[cur_node] += min_flow;
      }
    }
  } while (distance[sink_node] < numeric_limits<int32_t>::max());
}

void MinCostFlow::reduceCost(vector<int32_t>& potential) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
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
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
  // Works with the assumption that there's only a source and sink node.
  vector<uint32_t>& source_node = graph_.get_source_nodes();
  uint32_t sink_node = graph_.get_sink_nodes()[0];
  BellmanFord(source_node, distance, predecessor);
  reduceCost(distance);
  uint32_t iteration_cnt = 0;
  do {
    iteration_cnt++;
    cout << "Iteration: " << iteration_cnt << endl;
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
        Arc* arc = arcs[predecessor[cur_node]][cur_node];
        arc->flow += min_flow;
        arc->reverse_arc->flow -= min_flow;
        nodes_demand[predecessor[cur_node]] -= min_flow;
        nodes_demand[cur_node] += min_flow;
      }
    }
  } while (distance[sink_node] < numeric_limits<int32_t>::max());
}

void MinCostFlow::discharge(queue<uint32_t>& active_nodes,
                            vector<int32_t>& potentials,
                            vector<int32_t>& nodes_demand, int32_t eps) {
  uint32_t node_id = active_nodes.front();
  active_nodes.pop();
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  while (nodes_demand[node_id] > 0) {
    bool has_neg_cost_arc = false;
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      LOG(INFO) << "Cost: (" << node_id << ", " << it->first << "): "
                << it->second->cost + potentials[node_id] - potentials[it->first];
      if (it->second->cost + potentials[node_id] - potentials[it->first] < 0) {
        if (it->second->cap - it->second->flow > 0) {
          has_neg_cost_arc = true;
          // Push flow.
          pushes_cnt++;
          int32_t min_flow = min(nodes_demand[node_id],
                                 it->second->cap - it->second->flow);
          LOG(INFO) << "Pushing flow " << min_flow << " on (" << node_id
                    << ", " << it->first << ")";
          it->second->flow += min_flow;
          arcs[it->first][node_id]->flow -= min_flow;
          nodes_demand[node_id] -= min_flow;
          // If node doesn't have any excess then it will be activated.
          if (nodes_demand[it->first] <= 0) {
            active_nodes.push(it->first);
          }
          nodes_demand[it->first] += min_flow;
        }
      }
    }
    if (!has_neg_cost_arc) {
      // Relabel vertex.
      relabel_cnt++;
      potentials[node_id] -= eps;
      LOG(INFO) << "Potential of " << node_id << " : " << potentials[node_id];
    }
  }
}

void MinCostFlow::refine(vector<int32_t>& potentials, int32_t eps) {
  // Saturate arcs with negative reduced cost.
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
  // Saturate all the arcs with negative cost.
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cost + potentials[node_id] - potentials[it->first] < 0) {
        nodes_demand[node_id] -= it->second->cap - it->second->flow;
        nodes_demand[it->first] += it->second->cap - it->second->flow;
        arcs[it->first][node_id]->flow -= it->second->cap - it->second->flow;
        it->second->flow = it->second->cap;
      }
    }
  }
  graph_.logGraph();
  queue<uint32_t> active_nodes;
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    if (nodes_demand[node_id] > 0) {
      active_nodes.push(node_id);
    }
  }
  while (!active_nodes.empty()) {
    discharge(active_nodes, potentials, nodes_demand, eps);
  }
}

// Scales up costs by alpha * num_nodes
// It returns the maximum cost.
int32_t MinCostFlow::scaleUpCosts() {
  uint32_t num_nodes = graph_.get_num_nodes();
  const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  int32_t max_cost_arc = numeric_limits<int32_t>::min();
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      it->second->cost *= FLAGS_alpha_scaling_factor * num_nodes;
      if (it->second->cost > max_cost_arc) {
        max_cost_arc = it->second->cost;
      }
    }
  }
  return max_cost_arc;
}

void MinCostFlow::costScaling() {
  //    eps = max arc cost
  //    potential(v) = 0
  //    Establish a feasible flow x in the network
  //    while eps >= 1/n do
  //      (e, f, p) = refine(e, f p)
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<int32_t> potentials(num_nodes, 0);
  if (!graph_.hasSinkAndSource()) {
    graph_.addSinkAndSource();
  }
  maxFlow();
  graph_.removeSinkAndSource();
  graph_.logGraph();
  relabel_cnt = 0;
  pushes_cnt = 0;
  for (int32_t eps = scaleUpCosts() / FLAGS_alpha_scaling_factor; eps >= 1;
       eps = eps < FLAGS_alpha_scaling_factor && eps > 1 ?
         1 : eps / FLAGS_alpha_scaling_factor) {
    graph_.logGraph();
    refine(potentials, eps);
  }
  LOG(ERROR) << "Num relables: " << relabel_cnt;
  LOG(ERROR) << "Num pushes: " << pushes_cnt;
}

void MinCostFlow::globalPotentialsUpdate(vector<int32_t>& potential,
                                         int32_t eps) {
  uint32_t num_nodes = graph_.get_num_nodes();
  // Variable used to denote an empty bucket.
  uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
  uint32_t bucket_end = num_nodes + 1;
  vector<int32_t> rank(num_nodes + 1, 0);
  vector<uint32_t> bucket(max_rank + 1, 0);
  vector<uint32_t> bucket_prev(num_nodes + 1, 0);
  vector<uint32_t> bucket_next(num_nodes + 1, 0);
  vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  uint32_t num_active_nodes = 0;
  // Initialize buckets.
  for (uint32_t cur_rank = 0; cur_rank < max_rank; ++cur_rank) {
    bucket[cur_rank] = bucket_end;
  }
  // Put nodes with negative excess in bucket[0].
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    if (nodes_demand[node_id] < 0) {
      rank[node_id] = 0;
      bucket_next[node_id] = bucket[0];
      bucket_prev[bucket[0]] = node_id;
      bucket[0] = node_id;
    } else {
      rank[node_id] = max_rank;
      if (nodes_demand[node_id] > 0) {
        num_active_nodes++;
      }
    }
  }
  // Return if there are no active nodes.
  if (!num_active_nodes) {
    return;
  }
  int32_t bucket_index = 0;
  for ( ; num_active_nodes > 0 && bucket_index < max_rank; ++bucket_index) {
    while (bucket[bucket_index] != bucket_end) {
      uint32_t node_id = bucket[bucket_index];
      bucket[bucket_index] = bucket_next[node_id];
      map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
      map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
        Arc* rev_arc = it->second->reverse_arc;
        if (rev_arc->cap - rev_arc->flow > 0 &&
            bucket_index < rank[it->first]) {
          int32_t k = floor((rev_arc->cost + potential[it->first] -
                             potential[node_id]) / eps) + 1 + bucket_index;
          int32_t old_rank = rank[it->first];
          if (k < rank[it->first]) {
            rank[it->first] = k;
            // Remove node from the old bucket.
            if (old_rank < max_rank) {
              // Check if node is first element.
              if (bucket[old_rank] == it->first) {
                bucket[old_rank] = bucket_next[it->first];
              } else {
                uint32_t prev = bucket_prev[it->first];
                uint32_t next = bucket_next[it->first];
                bucket_next[prev] = next;
                bucket_prev[next] = prev;
              }
            }
            // Insert into the new bucket.
            bucket_next[it->first] = bucket[k];
            bucket_prev[bucket[k]] = it->first;
            bucket[k] = it->first;
          }
        }
      }
      if (nodes_demand[node_id] > 0) {
        num_active_nodes--;
      }
      if (num_active_nodes < 0) {
        break;
      }
    }
    if (num_active_nodes < 0) {
      break;
    }
  }
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    int32_t min_rank = min(rank[node_id], bucket_index);
    if (min_rank > 0) {
      potential[node_id] -= eps * min_rank;
    }
  }
}

void MinCostFlow::priceRefinement(vector<int32_t>& potential, int32_t eps) {
  uint32_t num_nodes = graph_.get_num_nodes();
  uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  vector<uint32_t> ordered_nodes;
  vector<int32_t> distance(num_nodes + 1, 0);
  uint32_t bucket_end = num_nodes + 1;
  vector<uint32_t> bucket(max_rank + 1, 0);
  vector<uint32_t> bucket_prev(num_nodes + 1, 0);
  vector<uint32_t> bucket_next(num_nodes + 1, 0);
  if (!graph_.orderTopologically(potential, ordered_nodes)) {
    // Graph contains a cycle. Cannot update potential
    return;
  }
  for (vector<uint32_t>::iterator node_it = ordered_nodes.begin();
       node_it != ordered_nodes.end(); ++node_it) {
    map<uint32_t, Arc*>::const_iterator it = arcs[*node_it].begin();
    map<uint32_t, Arc*>::const_iterator end_it = arcs[*node_it].end();
    for (; it != end_it; ++it) {
      int32_t reduced_cost = ceil((it->second->cost + potential[*node_it] -
                                   potential[it->first]) / eps);
      if (distance[*node_it] + reduced_cost < distance[it->first]) {
        distance[it->first] = distance[*node_it] + reduced_cost;
      }
    }
  }
  // Insert node_id at -distance[node_id].
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    uint32_t bucket_index = -distance[node_id];
    bucket_next[node_id] = bucket[bucket_index];
    bucket_prev[bucket[bucket_index]] = node_id;
    bucket[bucket_index] = node_id;
  }
  for (int32_t bucket_index = max_rank; bucket_index >= 0; --bucket_index) {
    while (bucket[bucket_index] != bucket_end) {
      uint32_t node_id = bucket[bucket_index];
      bucket[bucket_index] = bucket_next[node_id];
      map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
      map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
      }
    }
  }
}

// NOTE: if threshold is set to a smaller value than 2*n*eps then the
// problem may become infeasable. Check the paper.
void MinCostFlow::arcsFixing(vector<int32_t>& potential,
                             int32_t fix_threshold) {
  uint32_t num_nodes = graph_.get_num_nodes();
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  list<Arc*>& fixed_arcs = graph_.get_fixed_arcs();
  for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
    map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
    while (it != end_it) {
      if (it->second->cost + potential[node_id] - potential[it->first] >
          fix_threshold) {
        // Fix node.
        fixed_arcs.push_front(it->second);
        map<uint32_t, Arc*>::iterator to_erase_it = it;
        arcs[node_id].erase(it->first);
        ++it;
        arcs[node_id].erase(to_erase_it);
      } else {
        ++it;
      }
    }
  }
}

// NOTE: if threshold is set to a smaller value than 2*n*eps then the
// problem may become infeasable. Check the paper.
void MinCostFlow::arcsUnfixing(vector<int32_t>& potential,
                               int32_t fix_threshold) {
  uint32_t num_nodes = graph_.get_num_nodes();
  vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
  list<Arc*>& fixed_arcs = graph_.get_fixed_arcs();
  for (list<Arc*>::iterator it = fixed_arcs.begin(); it != fixed_arcs.end(); ) {
    if ((*it)->cost + potential[(*it)->src_node_id] -
        potential[(*it)->dst_node_id] < fix_threshold) {
      // Unfix node.
      list<Arc*>::iterator to_erase_it = it;
      arcs[(*it)->src_node_id][(*it)->dst_node_id] = *it;
      ++it;
      fixed_arcs.erase(to_erase_it);
    } else {
      ++it;
    }
  }
}

void MinCostFlow::pushLookahead(uint32_t dst_node_id) {
}
