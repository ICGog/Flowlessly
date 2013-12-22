#ifndef FLOWLESSLY_COST_SCALING_H
#define FLOWLESSLY_COST_SCALING_H

#include "graph.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <queue>

DECLARE_int64(alpha_scaling_factor);
DECLARE_bool(global_update);
DECLARE_bool(price_refinement);
DECLARE_bool(push_lookahead);
DECLARE_bool(arc_fixing);
DECLARE_int64(arc_fixing_threshold);
DECLARE_int64(price_refine_threshold);

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
    void updateAdmisibleGraph(vector<uint32_t>& updated_nodes,
                              vector<int64_t>& potential);
    int64_t getRefinePotential(vector<int64_t>& potential, uint64_t node_id,
                               int64_t eps);
    void relabel(vector<int64_t>& potential, uint32_t node_id, int64_t eps);
    bool pushLookahead(Arc* arc, queue<uint32_t>& active_nodes,
                       vector<int32_t>& nodes_demand,
                       vector<int64_t>& potential, int64_t eps);
    void push(Arc* arc, queue<uint32_t>& active_nodes,
              vector<int32_t>& nodes_demand);

  };

}
#endif
