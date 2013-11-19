#ifndef MIN_COST_FLOW_H
#define MIN_COST_FLOW_H

#include "graph.h"

class MinCostFlow {

 public:
 MinCostFlow(Graph graph): graph_(graph) {
  }

  void cycleCancelling();
  void successiveShortestPath();
  void BellmanFord(uint32_t source_node);
  void DijkstraSimple(uint32_t source_node);
  void DijkstraOptimized(uint32_t source_node);

 private:
  Graph graph_;

  void printCosts(int32_t* costs, uint32_t* predecessors);
  // Returns true if it removes a negative cycle.
  bool removeNegativeCycles(int32_t* distance, uint32_t* predecessor);
  void augmentFlow(int32_t* distance, uint32_t* predecessor,
                   uint32_t src_node, uint32_t dst_node);
  void maxFlow();

};
#endif
