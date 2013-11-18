#ifndef MIN_COST_FLOW_H
#define MIN_COST_FLOW_H

#include "graph.h"

class MinCostFlow {

 public:
 MinCostFlow(Graph graph): graph_(graph) {
  }

 private:
  void BellmanFord(Graph& graph, uint32_t source_node);
  void DijkstraSimple(Graph& graph, uint32_t source_node);
  void DijkstraOptimized(Graph& graph, uint32_t source_node);

  Graph graph_;

};
#endif
