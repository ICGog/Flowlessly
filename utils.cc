#include "utils.h"

#include <boost/heap/binomial_heap.hpp>
#include <limits>
#include <queue>

namespace flowlessly {

  using namespace boost::heap;
  using namespace std;

  double getTime() {
    struct rusage r;
    getrusage(0, &r);
    return r.ru_utime.tv_sec + (double)r.ru_utime.tv_usec / 1000000.0;
  }

  void logCosts(const vector<int64_t>& distance,
                const vector<uint32_t>& predecessor) {
    LOG(INFO) << "Logging graph costs";
    for (uint32_t node_id = 1; node_id < distance.size(); ++node_id) {
      LOG(INFO) << node_id << " " << distance[node_id] << " "
                << predecessor[node_id] << endl;
    }
  }

  // Computes max flow over the graph using the Ford-Fulkerson algorithm.
  // The Complexity of the algorithm is O(E * F). Where F is the max flow value.
  // NOTE: This method changes the graph.
  void maxFlow(Graph& graph) {
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& arcs = graph.get_arcs();
    vector<int32_t>& nodes_demand = graph.get_nodes_demand();
    vector<int32_t> visited(num_nodes, 0);
    vector<uint32_t> predecessor(num_nodes, 0);
    // Works with the assumption that there is only a sink and a source node.
    uint32_t source_node = graph.get_source_nodes()[0];
    uint32_t sink_node = graph.get_sink_nodes()[0];
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
          if (!visited[it->first] && it->second->cap > 0) {
            visited[it->first] = min(it->second->cap, visited[cur_node]);
            to_visit.push(it->first);
            predecessor[it->first] = cur_node;
            if (it->first == sink_node) {
              has_path = true;
              int32_t min_aux_flow = visited[it->first];
              for (uint32_t cur_node = it->first; predecessor[cur_node] > 0;
                   cur_node = predecessor[cur_node]) {
                Arc* arc = arcs[predecessor[cur_node]][cur_node];
                arc->cap -= min_aux_flow;
                arc->reverse_arc->cap += min_aux_flow;
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
      graph.logGraph();
    }
  }

  void BellmanFord(Graph& graph, const vector<uint32_t>& source_nodes,
                   vector<int64_t>& distance, vector<uint32_t>& predecessor) {
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    const vector<map<uint32_t, Arc*> >& arcs = graph.get_arcs();
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
            if (it->second->cap > 0 &&
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

  void DijkstraSimple(Graph& graph, const vector<uint32_t>& source_nodes,
                      vector<int64_t>& distance,
                      vector<uint32_t>& predecessor) {
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<bool> node_used(num_nodes, false);
    const vector<map<uint32_t, Arc*> >& arcs = graph.get_arcs();
    // Works with the assumption that all the elements of distance are
    // already set to INF.
    for (vector<uint32_t>::const_iterator it = source_nodes.begin();
         it != source_nodes.end(); ++it) {
      distance[*it] = 0;
    }
    for (uint32_t iter = 1; iter < num_nodes - 1; ++iter) {
      int64_t min_node_distance = numeric_limits<int32_t>::max();
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
        if (it->second->cap > 0 &&
            distance[min_node_id] + it->second->cost < distance[it->first]) {
          distance[it->first] = distance[min_node_id] + it->second->cost;
          predecessor[it->first] = min_node_id;
        }
      }
    }
  }

  void DijkstraOptimized(Graph& graph, const vector<uint32_t>& source_nodes,
                         vector<int64_t>& distance,
                         vector<uint32_t>& predecessor) {
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    const vector<map<uint32_t, Arc*> >& arcs = graph.get_arcs();
    vector<bool> visited(num_nodes, false);
    binomial_heap<pair<int64_t, uint32_t>,
                  compare<greater<pair<int64_t, uint32_t> > > > dist_heap;
    binomial_heap<pair<int64_t, uint32_t>,
                  compare<greater<pair<int64_t, uint32_t> > > >::handle_type
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
      pair<int64_t, uint32_t> min_dist = dist_heap.top();
      int32_t min_cost = min_dist.first;
      uint32_t min_node_id = min_dist.second;
      LOG(INFO) << min_node_id;
      dist_heap.pop();
      map<uint32_t, Arc*>::const_iterator it = arcs[min_node_id].begin();
      map<uint32_t, Arc*>::const_iterator end_it = arcs[min_node_id].end();
      for (; it != end_it; ++it) {
        if (it->second->cap > 0 &&
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

}
