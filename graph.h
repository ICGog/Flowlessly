#ifndef FLOWLESSLY_GRAPH_H
#define FLOWLESSLY_GRAPH_H

#include <limits>
#include <list>
#include <map>
#include <set>
#include <string>
#include <stdint.h>
#include <vector>

#include "arc.h"
#include "statistics.h"

namespace flowlessly {

  using namespace std;

  class Graph {

  public:
  Graph(Statistics& stats): statistics(stats) {
      added_sink_and_source = false;
      last_fixing_threshold = numeric_limits<int64_t>::max();
      cluster_agg_id = 1;
      sink_id = 2;
    }

    Graph(Graph& copy) {
      num_nodes = copy.num_nodes;
      nodes_demand = copy.nodes_demand;
      potential = copy.potential;
      arcs = copy.arcs;
      admisible_arcs = copy.admisible_arcs;
      source_nodes = copy.source_nodes;
      sink_nodes = copy.sink_nodes;
      deleted_nodes = copy.deleted_nodes;
      added_sink_and_source = copy.added_sink_and_source;
      task_nodes = copy.task_nodes;
      last_fixing_threshold = copy.last_fixing_threshold;
      statistics = copy.statistics;
    }

    void readGraph(const string& graph_file);
    void logGraph();
    void logAdmisibleGraph();
    void logResidualGraph();
    void writeGraph(const string& out_graph_file);
    uint32_t get_num_nodes();
    vector<int32_t>& get_nodes_demand();
    vector<int64_t>& get_potential();
    vector<map<uint32_t, Arc*> >& get_arcs();
    vector<map<uint32_t, Arc*> >& get_admisible_arcs();
    list<Arc*>& get_fixed_arcs();
    set<uint32_t>& get_source_nodes();
    set<uint32_t>& get_sink_nodes();
    bool hasSinkAndSource();
    void removeSinkAndSource();
    void addSinkAndSource();
    bool orderTopologically(vector<uint32_t>& ordered);
    bool checkFlow(const string& flow_file);
    bool checkEpsOptimality(int64_t eps);
    void removeNode(uint32_t node_id);
    void removeNodes(vector<uint32_t>& nodes_id);
    uint32_t addTaskNode();
    uint32_t addNode(int32_t node_demand, int64_t node_potential,
                     vector<Arc*>& arcs_from_node);
    uint32_t removeTaskNodes(uint16_t percentage);
    void nodeArcsFixing(uint32_t node_id, int64_t fix_threshold);
    void arcsFixing(int64_t fix_threshold);
    void arcsUnfixing(int64_t fix_threshold);
    int64_t scaleUpCosts(int64_t scale_up);
    void scaleDownCosts(int64_t scale_down);
    int64_t getRefinePotential(uint64_t node_id, int64_t eps);
    void resetPotentials();
    bool checkValid();

  private:
    void allocateGraphMemory(uint32_t num_nodes);
    bool visitTopologically(uint32_t node_id, vector<uint8_t>& marked,
                            list<uint32_t>& ordered);

    uint32_t num_nodes;
    // nodes_demand has a positive value if the node is a supply node and a
    // negative value if the node is a demand one.
    vector<int32_t> nodes_demand;
    vector<int64_t> potential;
    vector<map<uint32_t, Arc*> > arcs;
    vector<map<uint32_t, Arc*> > admisible_arcs;
    set<uint32_t> deleted_nodes;
    list<Arc*> fixed_arcs;
    set<uint32_t> source_nodes;
    set<uint32_t> sink_nodes;
    set<uint32_t> single_source_node;
    set<uint32_t> single_sink_node;
    set<uint32_t> task_nodes;
    bool added_sink_and_source;
    int64_t last_fixing_threshold;
    Statistics statistics;
    uint32_t cluster_agg_id;
    uint32_t sink_id;

  };

}
#endif
