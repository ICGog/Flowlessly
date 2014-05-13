#ifndef FLOWLESSLY_CYCLE_CANCELLING_H
#define FLOWLESSLY_CYCLE_CANCELLING_H

#include "graph.h"

namespace flowlessly {

  /**
   * Implements the cycle cancelling min cost maximum flow algorithm.
   */
  class CycleCancelling {

  public:
  CycleCancelling(Graph& graph): graph_(graph) {
    }

    /**
     * Applies the Cycle cancelling algorithm to compute the min cost flow.
     * The complexity is O(F * M + N * M^2 * C * U).
     * NOTE: It changes the graph.
     **/
    void cycleCancelling(bool has_flow);

  private:
    Graph& graph_;

    // Returns true if it removes a negative cycle.
    bool removeNegativeCycles(const vector<int64_t>& distance,
                              const vector<uint32_t>& predecessor);
    void augmentFlow(const vector<uint32_t>& predecessor,
                     uint32_t src_node, uint32_t dst_node);

  };

}
#endif
