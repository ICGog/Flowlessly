#ifndef FLOWLESSLY_UTILS_H
#define FLOWLESSLY_UTILS_H

#include "graph.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <unistd.h>
#include <vector>

DECLARE_int64(alpha_scaling_factor);
DECLARE_bool(arc_fixing);
DECLARE_int64(arc_fixing_threshold);
DECLARE_int64(num_preference_arcs);

namespace flowlessly {

  using namespace std;

  /**
   * Logs the cost and predecessor vectors.
   * distance vector storing the costs of getting to each node
   * predecessor vector storing the predessor node id
   **/
  void logCosts(const vector<int64_t>& distance,
                const vector<uint32_t>& predecessor);

  /**
   * Computes the maximum flow of a graph using the Ford-Fulkerson algorithm.
   * The Complexity of the algorithm is O(E * F). Where F is the max flow value.
   * NOTE: This method changes the graph.
   * @param graph the graph over which to compute the maximum flow
   **/
  void maxFlow(Graph& graph);

  /**
   * Compute the shortest distance over a graph using Bellman-Ford.
   * @param graph the graph for which to compute the shortest distance
   * @param source_nodes set containing source nodes
   * @param distance vector into which the distances will be populated
   * @param predecessor vector which will contain predecessor nodes
   **/
  void BellmanFord(Graph& graph, const set<uint32_t>& source_nodes,
                   vector<int64_t>& distance, vector<uint32_t>& predecessor);

  /**
   * Compute the shortest distance over a graph using simple Disjkstra's
   * algorithm.
   * @param graph the graph for which to compute the shortest distance
   * @param source_node set containing the source node
   * @param distance vector into which the distances will be populated
   * @param predecessor vector which will contain predecessor nodes
   **/
  void DijkstraSimple(Graph& graph, const set<uint32_t>& source_node,
                      vector<int64_t>& distance, vector<uint32_t>& predecessor);

  /**
   * Compute the shortest distance over a graph using Disjkstra's algorithm
   * with heaps.
   * @param graph the graph for which to compute the shortest distance
   * @param source_node set containing the source node
   * @param distance vector into which the distances will be populated
   * @param predecessor vector which will contain predecessor nodes
   **/
  void DijkstraOptimized(Graph& graph, const set<uint32_t>& source_node,
                         vector<int64_t>& distance,
                         vector<uint32_t>& predecessor);

}
#endif
