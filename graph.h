#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <string>
#include <stdint.h>
#include <vector>

#include "arc.h"

using namespace std;

class Graph {

 public:
  Graph() {
    added_sink_and_source = false;
  }

  Graph(Graph& copy) {
    num_nodes = copy.num_nodes;
    num_arcs = copy.num_arcs;
    nodes_descriptor = copy.nodes_descriptor;
    arcs = copy.arcs;
    supply_nodes = copy.supply_nodes;
    demand_nodes = copy.demand_nodes;
    added_sink_and_source = copy.added_sink_and_source;
  }

  void readGraph(const string& graph_file);
  void logGraph();
  void writeGraph(const string& out_graph_file);
  uint32_t get_num_nodes();
  uint32_t get_num_arcs();
  const vector<int32_t>& get_nodes_descriptor() const;
  const vector<map<uint32_t, Arc*> >& get_arcs() const;
  const vector<uint32_t>& get_supply_nodes() const;
  const vector<uint32_t>& get_demand_nodes() const;
  uint32_t get_source_id();
  uint32_t get_sink_id();
  bool hasSinkAndSource();
  void removeSinkAndSource();
  void addSinkAndSource();

 private:
  void allocateGraphMemory(uint32_t num_nodes, uint32_t num_arcs);

  uint32_t num_nodes;
  uint32_t num_arcs;
  vector<int32_t> nodes_descriptor;
  vector<map<uint32_t, Arc*> > arcs;
  vector<uint32_t> supply_nodes;
  vector<uint32_t> demand_nodes;
  bool added_sink_and_source;

};
#endif
