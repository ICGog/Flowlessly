#ifndef MIN_COST_FLOW_H
#define MIN_COST_FLOW_H

#include "graph.h"

class MinCostFlow {

 public:
 MinCostFlow(Graph graph): graph_(graph) {
  }

 private:
  Graph graph_;

};
#endif
