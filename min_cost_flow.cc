#include "min_cost_flow.h"

#include <limits>
#include <stdint.h>

using namespace std;

void MinCostFlow::BellmanFord(Graph& graph, uint32_t source_node) {
  uint32_t num_nodes = graph.get_num_nodes() + 1;
  uint32_t distance[num_nodes];
  uint32_t predecessor[num_nodes];
  map<uint32_t, Arc>* arcs = graph.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    if (node_id == source_node) {
      distance[node_id] = 0;
    } else {
      distance[node_id] = numeric_limits<uint32_t>::max();
    }
  }

  for (uint32_t iter = 1; iter < num_nodes - 1; ++iter) {
    bool relaxed = false;
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      map<uint32_t, Arc>::iterator it = arcs[node_id].begin();
      map<uint32_t, Arc>::iterator end_it = arcs[node_id].end();
      for (; it != end_it; ++it) {
        if (it->second.cap - it->second.flow > 0 &&
            distance[node_id] + it->second.cost < distance[it->first]) {
          distance[it->first] = distance[node_id] + it->second.cost;
          predecessor[it->first] = node_id;
          relaxed = true;
        }
      }
    }
    if (!relaxed) {
      return;
    }
  }

}

void MinCostFlow::DijkstraSimple(Graph& graph, uint32_t source_node) {
  uint32_t num_nodes = graph.get_num_nodes() + 1;
  uint32_t distance[num_nodes];
  uint32_t predecessor[num_nodes];
  bool node_used[num_nodes];
  map<uint32_t, Arc>* arcs = graph.get_arcs();
  for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
    node_used[node_id] = false;
    if (node_id == source_node) {
      distance[node_id] = 0;
    } else {
      distance[node_id] = numeric_limits<uint32_t>::max();
    }
  }
  for (uint32_t iter = 1; iter < num_nodes - 1; ++iter) {
    uint32_t min_node_distance = numeric_limits<uint32_t>::max();
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
}


void MinCostFlow::DijkstraOptimized(Graph& graph, uint32_t source_node) {
}
