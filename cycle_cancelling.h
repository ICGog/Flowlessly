#ifndef FLOWLESSLY_CYCLE_CANCELLING_H
#define FLOWLESSLY_CYCLE_CANCELLING_H

#include "graph.h"

namespace flowlessly {

  class CycleCancelling {

  public:
  CycleCancelling(Graph& graph): graph_(graph) {
    }

    void cycleCancelling();

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
