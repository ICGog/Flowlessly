// The Flowlessly project
// Copyright (c) 2013-2016 Ionel Gog <ionel.gog@cl.cam.ac.uk>

#ifndef FLOWLESSLY_CYCLE_CANCELLING_H
#define FLOWLESSLY_CYCLE_CANCELLING_H

#include "solvers/solver.h"

#include <gtest/gtest_prod.h>
#include <vector>

#include "graphs/adjacency_map_graph.h"
#include "misc/statistics.h"

namespace flowlessly {

/**
 * Implements the cycle cancelling min cost maximum flow algorithm.
 */
class CycleCancelling : public Solver {
 public:
  CycleCancelling(AdjacencyMapGraph* graph, Statistics* stats);
  ~CycleCancelling();

  void PrepareState();
  /**
   * Applies the Cycle cancelling algorithm to compute the min cost flow.
   * The complexity is O(F * M + N * M^2 * C * U).
   **/
  bool Run();

 private:
  FRIEND_TEST(CycleCancellingTests, AugmentFlow);

  void AugmentFlow(const vector<uint32_t>& predecessor, uint32_t src_node,
                   uint32_t dst_node);
  // Returns true if it removes a negative cycle.
  bool RemoveNegativeCycles(const vector<uint32_t>& predecessor);

  AdjacencyMapGraph* graph_;
};

} // namespace flowlessly
#endif // FLOWLESSLY_CYCLE_CANCELLING_H
