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
    node_supply = new uint32_t[num_nodes + 1];
    std::copy(&copy.node_supply[0], &copy.node_supply[num_nodes + 1],
              node_supply);
    arcs = new map<uint32_t, Arc>[num_nodes + 1];
    std::copy(&copy.arcs[0], &copy.arcs[num_nodes + 1], arcs);
  }

  ~Graph() {
    delete[] node_supply;
    delete [] arcs;
  }

  void readGraph(const string& graph_file);
  void writeGraph(const string& out_graph_file);
  uint32_t get_num_nodes();
  uint32_t get_num_arcs();
  map<uint32_t, Arc>* get_arcs();

 private:
  void allocateGraphMemory(uint32_t num_nodes, uint32_t num_arcs);

  uint32_t num_nodes;
  uint32_t num_arcs;
  uint32_t* node_supply;
  map<uint32_t, Arc>* arcs;

};
#endif
