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

void MinCostFlow::printCosts(const vector<int32_t>& distance,
                             const vector<uint32_t>& predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    cout << node_id << " " << distance[node_id] << " "
         << predecessor[node_id] << endl;
  }
}

void MinCostFlow::BellmanFord(const vector<uint32_t>& source_nodes) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
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
              distance[node_id] + it->second->cost <= distance[it->first]) {
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
  printCosts(distance, predecessor);
}

void MinCostFlow::augmentFlow(vector<int32_t>& distance,
                              vector<uint32_t>& predecessor,
                              uint32_t src_node, uint32_t dst_node) {
  LOG(INFO) << "Augment: " << src_node << " " << dst_node;
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
    cur_node = predecessor[cur_node];
  } while (cur_node != dst_node);
  LOG(INFO) << "FLW: " << min_flow;
  do {
    // TODO(ionel): Update flow.
    cur_node = predecessor[cur_node];
  } while (cur_node != dst_node);
}

bool MinCostFlow::removeNegativeCycles(vector<int32_t>& distance,
                                       vector<uint32_t>& predecessor) {
  bool found_cycle = false;
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cap - it->second->flow > 0 &&
          distance[node_id] + it->second->cost < distance[it->first]) {
        // Found negative cycle.
        found_cycle = true;
        augmentFlow(distance, predecessor, node_id, it->first);
        return true;
      }
    }
  }
  return found_cycle;
}

void MinCostFlow::DijkstraSimple(const vector<uint32_t>& source_nodes) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
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
  printCosts(distance, predecessor);
}


void MinCostFlow::DijkstraOptimized(const vector<uint32_t>& source_nodes) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  vector<map<uint32_t, Arc*> > arcs = graph_.get_arcs();
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > > dist_heap;
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > >::handle_type
                handles[num_nodes];
  for (vector<uint32_t>::const_iterator it = source_nodes.begin();
       it != source_nodes.end(); ++it) {
    distance[*it] = 0;
    handles[*it] = dist_heap.push(make_pair(0, *it));
  }
  while (!dist_heap.empty()) {
    pair<int32_t, uint32_t> min_dist = dist_heap.top();
    int32_t min_cost = min_dist.first;
    uint32_t min_node_id = min_dist.second;
    dist_heap.pop();
    map<uint32_t, Arc*>::iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc*>::iterator end_it = arcs[min_node_id].end();
    for (; it != end_it; ++it) {
      if (it->second->cap - it->second->flow > 0 &&
          distance[min_node_id] + it->second->cost <= distance[it->first]) {
        distance[it->first] = distance[min_node_id] + it->second->cost;
        if (!predecessor[it->first]) {
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
  printCosts(distance, predecessor);
}

void MinCostFlow::maxFlow() {
  if (!graph_.hasSinkAndSource()) {
    graph_.addSinkAndSource();
  }
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
      LOG(INFO) << "Cur node: " << cur_node;
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
              LOG(INFO) << predecessor[cur_node] << " " << cur_node << " " << min_aux_flow;
            }
            break;
          }
        }
      }
    }
    graph_.printGraph();
  }
  graph_.removeSinkAndSource();
}

void MinCostFlow::cycleCancelling() {
  //    Establish a feasible flow x in the network
  //    while ( Gx contains a negative cycle ) do
  //        identify a negative cycle W
  //        mr = min(r(i,j)) where (i,j) is part of W
  //        augment mr units of flow along the cycle W
  //        update Gx
  maxFlow();
  graph_.printGraph();
  BellmanFord(graph_.get_supply_nodes());
  //  removeNegativeCycles(dist_pred.get<0>(), dist_pred.get<1>());
  //  graph_.printGraph();
}

void MinCostFlow::successiveShortestPath() {
  //    Transform network G by adding source and sink
  //    Initial flow x is zero
  //    Use Bellman-Ford's algorithm to establish potentials PI
  //    Reduce Cost ( PI )
  //    while ( Gx contains a path from s to t ) do
  //        Find any shortest path P from s to t
  //        Reduce Cost ( PI )
  //        Augment current flow x along P
  //        update Gx
}
