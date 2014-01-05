#ifndef FLOWLESSLY_COST_SCALING_H
#define FLOWLESSLY_COST_SCALING_H

#include "graph.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <queue>

DECLARE_bool(global_update);
DECLARE_bool(price_refinement);
DECLARE_bool(push_lookahead);
DECLARE_int64(price_refine_threshold);

namespace flowlessly {

  using namespace std;

  class CostScaling {

  public:
  CostScaling(Graph& graph, Statistics& stats): graph_(graph),
      statistics(stats) {
    }

    void costScaling();

  private:
    Graph& graph_;
    Statistics& statistics;
    uint32_t relabel_cnt;
    uint32_t pushes_cnt;
    uint32_t refine_cnt;

    void refine(int64_t eps);
    void discharge(queue<uint32_t>& active_nodes, vector<int32_t>& nodes_demand,
                   int64_t eps);
    void globalPotentialsUpdate(int64_t eps);
    bool priceRefinement(int64_t eps);
    void updateAdmisibleGraph(vector<uint32_t>& updated_nodes);
    void relabel(uint32_t node_id, int64_t eps);
    bool pushLookahead(Arc* arc, queue<uint32_t>& active_nodes,
                       vector<int32_t>& nodes_demand, int64_t eps);
    void push(Arc* arc, queue<uint32_t>& active_nodes,
              vector<int32_t>& nodes_demand);

  };

}
#endif
