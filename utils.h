#ifndef FLOWLESSLY_UTILS_H
#define FLOWLESSLY_UTILS_H

#include "graph.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

DECLARE_int64(alpha_scaling_factor);
DECLARE_bool(arc_fixing);
DECLARE_int64(arc_fixing_threshold);

namespace flowlessly {

  using namespace std;

  double getTime();
  void logCosts(const vector<int64_t>& distance,
                const vector<uint32_t>& predecessor);
  void maxFlow(Graph& graph);
  void BellmanFord(Graph& graph, const set<uint32_t>& source_nodes,
                   vector<int64_t>& distance, vector<uint32_t>& predecessor);
  void DijkstraSimple(Graph& graph, const set<uint32_t>& source_node,
                      vector<int64_t>& distance, vector<uint32_t>& predecessor);
  void DijkstraOptimized(Graph& graph, const set<uint32_t>& source_node,
                         vector<int64_t>& distance,
                         vector<uint32_t>& predecessor);

}
#endif
