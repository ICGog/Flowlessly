// The Flowlessly project
// Copyright (c) 2013-2016 Ionel Gog <ionel.gog@cl.cam.ac.uk>

#ifndef FLOWLESSLY_SUCCESSIVE_SHORTEST_H
#define FLOWLESSLY_SUCCESSIVE_SHORTEST_H

#include "solvers/solver.h"

#include <vector>

#include "graphs/adjacency_map_graph.h"
#include "misc/statistics.h"

namespace flowlessly {

/**
 * Implements the successive shortest min cost maximum flow algorithm.
 */
class SuccessiveShortest : public Solver {
 public:
  SuccessiveShortest(AdjacencyMapGraph* graph, Statistics* stats);
  ~SuccessiveShortest();

  void PrepareState();
  bool Run();

 private:
  /**
   * Implements the successive shortest path algorithm without potentials.
   * As a result, it uses Bellman-Ford to compute the paths.
   * NOTE: It changes the graph.
   **/
  void SuccessiveShortestBellman();

  /**
   * Implements the successive shortest path algorithm with potentials.
   * It uses Dijkstra's algorithm with heaps to compute the paths.
   * NOTE: It changes the graph.
   **/
  void SuccessiveShortestDijkstra();

  AdjacencyMapGraph* graph_;
};

}  // namespace flowlessly
#endif  // FLOWLESSLY_SUCCESSIVE_SHORTEST_H
