#ifndef FLOWLESSLY_SUCCESSIVE_SHORTEST_H
#define FLOWLESSLY_SUCCESSIVE_SHORTEST_H

#include "graph.h"

namespace flowlessly {

  class SuccessiveShortest {

  public:
  SuccessiveShortest(Graph graph): graph_(graph) {
    }

    void successiveShortestPath();
    void successiveShortestPathPotentials();

  private:
    Graph graph_;

    void reduceCost(vector<int64_t>& potential);

  };

}
#endif
