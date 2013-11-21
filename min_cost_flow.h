#ifndef MIN_COST_FLOW_H
#define MIN_COST_FLOW_H

#include "graph.h"

#include <boost/tuple/tuple.hpp>

class MinCostFlow {

 public:
 MinCostFlow(Graph graph): graph_(graph) {
  }

  void cycleCancelling();
  void successiveShortestPath();
  void BellmanFord(const vector<uint32_t>& source_node);
  void DijkstraSimple(const vector<uint32_t>& source_node);
  void DijkstraOptimized(const vector<uint32_t>& source_node);

 private:
  Graph graph_;

  void printCosts(const vector<int32_t>& distance,
                  const vector<uint32_t>& predecessor);
  // Returns true if it removes a negative cycle.
  bool removeNegativeCycles(vector<int32_t>& distance,
                            vector<uint32_t>& predecessor);
  void augmentFlow(vector<int32_t>& distance, vector<uint32_t>& predecessor,
                   uint32_t src_node, uint32_t dst_node);
  void maxFlow();

};
#endif
