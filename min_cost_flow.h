#ifndef MIN_COST_FLOW_H
#define MIN_COST_FLOW_H

#include "graph.h"

class MinCostFlow {

 public:
 MinCostFlow(Graph graph): graph_(graph) {
  }

 private:
  void CycleCancelling();
  void SuccessiveShortestPath();
  void BellmanFord(uint32_t source_node);
  void DijkstraSimple(uint32_t source_node);
  void DijkstraOptimized(uint32_t source_node);

  Graph graph_;

};
#endif
