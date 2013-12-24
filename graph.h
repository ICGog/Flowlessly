#ifndef FLOWLESSLY_GRAPH_H
#define FLOWLESSLY_GRAPH_H

#include <list>
#include <map>
#include <set>
#include <string>
#include <stdint.h>
#include <vector>

#include "arc.h"

namespace flowlessly {

  using namespace std;

  class Graph {

  public:
    Graph() {
      added_sink_and_source = false;
    }

    Graph(Graph& copy) {
      num_nodes = copy.num_nodes;
      num_arcs = copy.num_arcs;
      nodes_demand = copy.nodes_demand;
      potential = copy.potential;
      arcs = copy.arcs;
      admisible_arcs = copy.admisible_arcs;
      source_nodes = copy.source_nodes;
      sink_nodes = copy.sink_nodes;
      deleted_nodes = copy.deleted_nodes;
      added_sink_and_source = copy.added_sink_and_source;
    }

    void readGraph(const string& graph_file);
    void logGraph();
    void logAdmisibleGraph();
    void logResidualGraph();
    void writeGraph(const string& out_graph_file, int64_t scale_down);
    uint32_t get_num_nodes();
    uint32_t get_num_arcs();
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
    uint32_t addNode(uint32_t node_id, int32_t node_demand,
                     vector<Arc*>& arcs_from_node);

  private:
    void allocateGraphMemory(uint32_t num_nodes);
    bool visitTopologically(uint32_t node_id, vector<uint8_t>& marked,
                            list<uint32_t>& ordered);

    uint32_t num_nodes;
    uint32_t num_arcs;
    // nodes_demand has a positive value if the node is a supply node and a
    // negative value if the node is a demand one.
    vector<int32_t> nodes_demand;
    vector<int64_t> potential;
    vector<map<uint32_t, Arc*> > arcs;
    vector<map<uint32_t, Arc*> > admisible_arcs;
    list<uint32_t> deleted_nodes;
    list<Arc*> fixed_arcs;
    set<uint32_t> source_nodes;
    set<uint32_t> sink_nodes;
    set<uint32_t> single_source_node;
    set<uint32_t> single_sink_node;
    bool added_sink_and_source;

  };

}
#endif
