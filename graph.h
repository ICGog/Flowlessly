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
  }

  Graph(Graph& copy) {
    num_nodes = copy.get_num_nodes();
    num_arcs = copy.get_num_arcs();
    nodes_supply = new int32_t[num_nodes + 1];
    std::copy(&copy.nodes_supply[0], &copy.nodes_supply[num_nodes + 1],
              nodes_supply);
    arcs = new map<uint32_t, Arc>[num_nodes + 1];
    std::copy(&copy.arcs[0], &copy.arcs[num_nodes + 1], arcs);
    // TODO(ionel): Copy supply/demand nodes.
  }

  ~Graph() {
    delete[] nodes_supply;
    delete[] arcs;
  }

  void readGraph(const string& graph_file);
  void writeGraph(const string& out_graph_file);
  uint32_t get_num_nodes();
  uint32_t get_num_arcs();
  int32_t* get_nodes_supply();
  map<uint32_t, Arc>* get_arcs();
  const vector<uint32_t>& get_supply_nodes();
  const vector<uint32_t>& get_demand_nodes();

 private:
  void allocateGraphMemory(uint32_t num_nodes, uint32_t num_arcs);

  uint32_t num_nodes;
  uint32_t num_arcs;
  int32_t* nodes_supply;
  map<uint32_t, Arc>* arcs;
  vector<uint32_t> supply_nodes;
  vector<uint32_t> demand_nodes;

};
#endif
