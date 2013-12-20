#ifndef FLOWLESSLY_COST_SCALING_H
#define FLOWLESSLY_COST_SCALING_H

#include "graph.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <queue>

DECLARE_int64(alpha_scaling_factor);

namespace flowlessly {

  using namespace std;

  class CostScaling {

  public:
  CostScaling(Graph graph): graph_(graph) {
    }

    void costScaling();

  private:
    Graph graph_;
    uint32_t relabel_cnt;
    uint32_t pushes_cnt;
    uint32_t refine_cnt;

    void refine(vector<int64_t>& potential, int64_t eps);
    void discharge(queue<uint32_t>& active_nodes, vector<int64_t>& potential,
                   vector<int32_t>& nodes_demand, int64_t eps);
    int64_t scaleUpCosts();
    void globalPotentialsUpdate(vector<int64_t>& potential, int64_t eps);
    bool priceRefinement(vector<int64_t>& potential, int64_t eps);
    void arcsFixing(vector<int64_t>& potential, int64_t fix_threshold);
    void arcsUnfixing(vector<int64_t>& potential, int64_t fix_threshold);
    void pushLookahead(uint32_t src_node_id, uint32_t dst_node_id);
    void updateAdmisibleGraph(vector<int64_t>& potential);

  };

}
#endif
