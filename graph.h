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
    /**
     * Constructs a graph. It assumes that the cluster aggregator node has id 1
     * and that the sink node has id 2.
     * @param stats the object into which to store statistics
     **/
  Graph(Statistics& stats): statistics(stats) {
      added_sink_and_source = false;
      last_fixing_threshold = numeric_limits<int64_t>::max();
      // Assummes that Cluster Aggregate vertex has id 1 and sink has id 2.
      cluster_agg_id = 1;
      sink_id = 2;
    }

    /**
     * Read a graph in the DIMACS format.
     * @param graph_file the file to read the graph from
     **/
    void readGraph(const string& graph_file);

    void logGraph();

    void logAdmisibleGraph();

    void logResidualGraph();
    /**
     * Writes the flow graph to a file. It writes it in a format similar to
     * DIMACS. Every flow arc is represented as
     * f src_vertex_id dst_vertex_id flow.
     * @param out_graph_file the file to write the flow graph to
     **/

    void writeFlowGraph(const string& out_graph_file);
    /**
     * Writes the graph to a file. It writes the graph in the DIMACS format.
     * @param out_graph_file the file to write the graph to
     **/
    void writeGraph(const string& out_graph_file);

    uint32_t get_num_nodes() const;
    vector<int32_t>& get_nodes_demand();
    vector<int64_t>& get_potential();
    vector<map<uint32_t, Arc*> >& get_arcs();
    vector<map<uint32_t, Arc*> >& get_admisible_arcs();
    list<Arc*>& get_fixed_arcs();
    set<uint32_t>& get_source_nodes();
    set<uint32_t>& get_sink_nodes();

    /**
     * Checks if the graph flow is valid. It checks if all the demand is
     * satisfied.
     * NOTE: It does not check if the flow is the min cost flow.
     * @param flow_file the file storing the flow graph
     **/
    bool checkFlow(const string& flow_file);

    /**
     * Check epsilon optimality of the flow graph. A flow graph is
     * Eps-optimal if there is no arc with capacity > 0 and
     * cost + potential[src_vertex_id] - potential[dst_vertex_id] < -eps.
     * @param eps Epsilon
     **/
    bool checkEpsOptimality(int64_t eps);

    /**
     * Checks if the graph is valid. It performes the following checks:
     * 1) capacity <= initial capacity for every arc
     * 2) flow = reverse_flow for every arc
     * 3) total demand is 0
     **/
    bool checkValid();

    /**
     * Checks if two graphs are equal.
     **/
    bool checkEqual(Graph& other);

    /**
     * Check if the graph has single source and single sink.
     **/
    bool hasSinkAndSource();

    /**
     * Remove added source and sink nodes.
     * @return true if the source and sink have succesfully been removed.
     **/
    bool removeSinkAndSource();

    /**
     * Add single source and sink nodes. The source will have id num_nodes and
     * the sink will have id num_nodes + 1.
     **/
    bool addSinkAndSource();

    void removeNode(uint32_t node_id);
    uint32_t addTaskNode();
    uint32_t addNode(int32_t node_demand, int64_t node_potential,
                     const vector<Arc*>& arcs_from_node);
    uint32_t removeTaskNode();
    bool removeTaskNode(uint32_t node_id);
    uint32_t removeTaskNodes(uint16_t percentage);
    uint32_t changeArcCosts(uint16_t percentage);
    void removeNodeFromFixedArcs(uint32_t node_id);
    void removeArcs(uint32_t node_id);
    int64_t generateArcCost();
    uint32_t generateResourceNodeId();

    void nodeArcsFixing(uint32_t node_id, int64_t fix_threshold);
    void arcsFixing(int64_t fix_threshold);
    void arcsUnfixing(int64_t fix_threshold);
    bool orderTopologically(vector<uint32_t>& ordered);
    int64_t scaleUpCosts(int64_t scale_up);
    void scaleDownCosts(int64_t scale_down);
    int64_t get_refine_potential(uint64_t node_id, int64_t eps);
    void resetPotentials();
    void resetGraph();

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
    Statistics& statistics;
    uint32_t cluster_agg_id;
    uint32_t sink_id;

  };

}
#endif
