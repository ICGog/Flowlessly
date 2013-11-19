#ifndef MIN_COST_FLOW_H
#define MIN_COST_FLOW_H

#include "graph.h"

class MinCostFlow {

 public:
 MinCostFlow(Graph graph): graph_(graph) {
  }

  void CycleCancelling();
  void SuccessiveShortestPath();
  void BellmanFord(uint32_t source_node);
  void DijkstraSimple(uint32_t source_node);
  void DijkstraOptimized(uint32_t source_node);

 private:
  Graph graph_;

  void printCosts(int32_t* costs, uint32_t* predecessors);

};
#endif
