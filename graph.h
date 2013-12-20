#ifndef FLOWLESSLY_GRAPH_H
#define FLOWLESSLY_GRAPH_H

#include <list>
#include <map>
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
      arcs = copy.arcs;
      admisible_arcs = copy.admisible_arcs;
      source_nodes = copy.source_nodes;
      sink_nodes = copy.sink_nodes;
      added_sink_and_source = copy.added_sink_and_source;
    }

    void readGraph(const string& graph_file);
    void logGraph();
    void logAdmisibleGraph(vector<int64_t>& potential);
    void logResidualGraph();
    void writeGraph(const string& out_graph_file, int64_t scale_down);
    uint32_t get_num_nodes();
    uint32_t get_num_arcs();
    vector<int32_t>& get_nodes_demand();
    vector<map<uint32_t, Arc*> >& get_arcs();
    vector<map<uint32_t, Arc*> >& get_admisible_arcs();
    list<Arc*>& get_fixed_arcs();
    vector<uint32_t>& get_source_nodes();
    vector<uint32_t>& get_sink_nodes();
    bool hasSinkAndSource();
    void removeSinkAndSource();
    void addSinkAndSource();
    bool orderTopologically(vector<int64_t>& potential,
                            vector<uint32_t>& ordered);
    bool checkFlow(const string& flow_file);
    bool checkEpsOptimality(vector<int64_t>& potential, int64_t eps);

  private:
    void allocateGraphMemory(uint32_t num_nodes);

    uint32_t num_nodes;
    uint32_t num_arcs;
    // nodes_demand has a positive value if the node is a supply node and a
    // negative value if the node is a demand one.
    vector<int32_t> nodes_demand;
    vector<map<uint32_t, Arc*> > arcs;
    vector<map<uint32_t, Arc*> > admisible_arcs;
    list<Arc*> fixed_arcs;
    vector<uint32_t> source_nodes;
    vector<uint32_t> sink_nodes;
    vector<uint32_t> single_source_node;
    vector<uint32_t> single_sink_node;
    bool added_sink_and_source;

  };

}
#endif
