#include "min_cost_flow.h"

#include <boost/heap/binomial_heap.hpp>
#include <limits>
#include <queue>
#include <stdint.h>

using namespace boost::heap;
using namespace std;

void MinCostFlow::printCosts(int32_t* distance, uint32_t* predecessor) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    cout << node_id << " " << distance[node_id] << " "
         << predecessor[node_id] << endl;
  }
}

void MinCostFlow::BellmanFord(uint32_t source_node) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  int32_t distance[num_nodes];
  uint32_t predecessor[num_nodes];
  map<uint32_t, Arc>* arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    if (node_id == source_node) {
      distance[node_id] = 0;
    } else {
      distance[node_id] = numeric_limits<int32_t>::max();
    }
  }

  for (uint32_t iter = 1; iter < num_nodes - 1; ++iter) {
    bool relaxed = false;
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      map<uint32_t, Arc>::iterator it = arcs[node_id].begin();
      map<uint32_t, Arc>::iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
        if (it->second.cap - it->second.flow > 0 &&
            distance[node_id] + it->second.cost <= distance[it->first]) {
          distance[it->first] = distance[node_id] + it->second.cost;
          predecessor[it->first] = node_id;
          relaxed = true;
        }
      }
    }
    if (!relaxed) {
      break;
    }
  }
  printCosts(distance, predecessor);
}

void MinCostFlow::augmentFlow(int32_t* distance, uint32_t* predecessor,
                              uint32_t src_node, uint32_t dst_node) {
  map<uint32_t, Arc>* arcs = graph_.get_arcs();
  int32_t min_flow = numeric_limits<int32_t>::max();
  for (uint32_t cur_node = src_node; cur_node != dst_node;
       cur_node = predecessor[cur_node]) {
    Arc arc = arcs[predecessor[cur_node]][cur_node];
    if (abs(arc.flow) < min_flow) {
      min_flow = abs(arc.flow);
    }
  }
  for (uint32_t cur_node = src_node; cur_node != dst_node;
       cur_node = predecessor[cur_node]) {
    Arc* arc = &arcs[predecessor[cur_node]][cur_node];
    if (arc->reverse) {
      arc->flow += min_flow;
    } else {
      arc->flow -= min_flow;
    }
  }
}

bool MinCostFlow::removeNegativeCycles(int32_t* distance,
                                       uint32_t* predecessor) {
  bool found_cycle = false;
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  map<uint32_t, Arc>* arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    map<uint32_t, Arc>::iterator it = arcs[node_id].begin();
    map<uint32_t, Arc>::iterator end_it = arcs[node_id].end();
    for (; it != end_it; ++it) {
      if (it->second.cap - it->second.flow > 0 &&
          distance[node_id] + it->second.cost < distance[it->first]) {
        // Found negative cycle.
        found_cycle = true;
        augmentFlow(distance, predecessor, node_id, it->first);
      }
    }
  }
  return found_cycle;
}

void MinCostFlow::DijkstraSimple(uint32_t source_node) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  int32_t distance[num_nodes];
  uint32_t predecessor[num_nodes];
  bool node_used[num_nodes];
  map<uint32_t, Arc>* arcs = graph_.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    node_used[node_id] = false;
    if (node_id == source_node) {
      distance[node_id] = 0;
    } else {
      distance[node_id] = numeric_limits<int32_t>::max();
    }
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
    map<uint32_t, Arc>::iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc>::iterator end_it = arcs[min_node_id].end();
    for (; it != end_it; ++it) {
      if (it->second.cap - it->second.flow > 0 &&
          distance[min_node_id] + it->second.cost < distance[it->first]) {
        distance[it->first] = distance[min_node_id] + it->second.cost;
        predecessor[it->first] = min_node_id;
      }
    }
  }
  printCosts(distance, predecessor);
}


void MinCostFlow::DijkstraOptimized(uint32_t source_node) {
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  int32_t distance[num_nodes];
  uint32_t predecessor[num_nodes];
  map<uint32_t, Arc>* arcs = graph_.get_arcs();
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > > dist_heap;
  binomial_heap<pair<int32_t, uint32_t>,
                compare<greater<pair<int32_t, uint32_t> > > >::handle_type
                handles[num_nodes];
  handles[source_node] = dist_heap.push(make_pair(0, source_node));
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    predecessor[node_id] = 0;
    if (node_id == source_node) {
      distance[node_id] = 0;
    } else {
      distance[node_id] = numeric_limits<int32_t>::max();
    }
  }
  while (!dist_heap.empty()) {
    pair<int32_t, uint32_t> min_dist = dist_heap.top();
    int32_t min_cost = min_dist.first;
    uint32_t min_node_id = min_dist.second;
    dist_heap.pop();
    map<uint32_t, Arc>::iterator it = arcs[min_node_id].begin();
    map<uint32_t, Arc>::iterator end_it = arcs[min_node_id].end();
    for (; it != end_it; ++it) {
      if (it->second.cap - it->second.flow > 0 &&
          distance[min_node_id] + it->second.cost <= distance[it->first]) {
        distance[it->first] = distance[min_node_id] + it->second.cost;
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
  uint32_t num_nodes = graph_.get_num_nodes() + 1;
  map<uint32_t, Arc>* arcs = graph_.get_arcs();
  vector<uint32_t> supply_nodes = graph_.get_supply_nodes();
  vector<uint32_t> demand_nodes = graph_.get_demand_nodes();
  int32_t* node_supply = graph_.get_node_supply();
  bool has_path = true;
  while (has_path) {
    has_path = false;
    queue<uint32_t> to_visit;
    int32_t visited[num_nodes];
    uint32_t predecessor[num_nodes];
    for (vector<uint32_t>::iterator it = supply_nodes.begin();
         it != supply_nodes.end(); ++it) {
      to_visit.push(*it);
      visited[*it] = node_supply[*it];
    }
    while (!to_visit.empty() && !has_path) {
      uint32_t cur_node = to_visit.front();
      to_visit.pop();
      map<uint32_t, Arc>::iterator it = arcs[cur_node].begin();
      map<uint32_t, Arc>::iterator end_it = arcs[cur_node].end();
      for (; it != end_it; ++it) {
        if (!visited[it->first] && it->second.cap - it->second.flow > 0) {
          visited[it->first] = min(it->second.cap - it->second.flow,
                                   visited[cur_node]);
          to_visit.push(it->first);
          predecessor[it->first] = cur_node;
          if (node_supply[it->first] < 0) {
            has_path = true;
            int32_t min_aux_flow = visited[it->first];
            for (uint32_t cur_node = it->first; node_supply[cur_node] <= 0;
                 cur_node = predecessor[cur_node]) {
              arcs[predecessor[cur_node]][cur_node].flow += min_aux_flow;
              arcs[cur_node][predecessor[cur_node]].flow -= min_aux_flow;
            }
            break;
          }
        }
      }
    }
  }
}

void MinCostFlow::cycleCancelling() {
  //    Establish a feasible flow x in the network
  //    while ( Gx contains a negative cycle ) do
  //        identify a negative cycle W
  //        mr = min(r(i,j)) where (i,j) is part of W
  //        augment mr units of flow along the cycle W
  //        update Gx
  maxFlow();
  //  for (; removeNegativeCycles(distance, predecessor); ) {
  //  }
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
