#ifndef FLOWLESSLY_SUCCESSIVE_SHORTEST_H
#define FLOWLESSLY_SUCCESSIVE_SHORTEST_H

#include "graph.h"

namespace flowlessly {

  /**
   * Implements the successive shortest min cost maximum flow algorithm.
   */
  class SuccessiveShortest {

  public:
  SuccessiveShortest(Graph& graph): graph_(graph) {
    }

    /**
     * Implements the successive shortest path algorithm without potentials.
     * As a result, it uses Bellman-Ford to compute the paths.
     * NOTE: It changes the graph.
     **/
    void successiveShortestPath();

    /**
     * Implements the successive shortest path algorithm with potentials.
     * It uses Dijkstra's algorithm with heaps to compute the paths.
     * NOTE: It changes the graph.
     **/
    void successiveShortestPathPotentials();

  private:
    Graph& graph_;

    void reduceCost(const vector<int64_t>& distance);

  };

}
#endif
