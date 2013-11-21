#include "graph.h"
#include "min_cost_flow.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <limits>

DEFINE_string(graph_file, "graph.in", "File containing the input graph.");
DEFINE_string(out_graph_file, "graph.out",
              "File the output graph will be written");
DEFINE_string(algorithm, "cycle_cancelling",
              "Algorithms to run: cycle_cancelling, bellman_ford, dijkstra, dijkstra_heap, successive_shortest_path");

inline void init(int argc, char *argv[]) {
  // Set up usage message.
  string usage("Runs an RA++ job.  Sample usage:\nmusketeer -i test.rap");
  google::SetUsageMessage(usage);

  // Use gflags to parse command line flags
  // The final (boolean) argument determines whether gflags-parsed flags should
  // be removed from the array (if true), otherwise they will re-ordered such
  // that all gflags-parsed flags are at the beginning.
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Set up glog for logging output
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char *argv[]) {
  init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;
  Graph graph;
  graph.readGraph(FLAGS_graph_file);
  graph.logGraph();
  MinCostFlow min_cost_flow(graph);
  if (!FLAGS_algorithm.compare("bellman_ford")) {
    LOG(INFO) << "------------ BellmanFord ------------";
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    min_cost_flow.BellmanFord(graph.get_supply_nodes(), distance, predecessor);
    min_cost_flow.logCosts(distance, predecessor);
  } else if (!FLAGS_algorithm.compare("dijkstra")) {
    LOG(INFO) << "------------ Dijkstra ------------";
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    min_cost_flow.DijkstraSimple(graph.get_supply_nodes(), distance,
                                 predecessor);
    min_cost_flow.logCosts(distance, predecessor);
  } else if (!FLAGS_algorithm.compare("dijkstra_heap")) {
    LOG(INFO) << "------------ Dijkstra with heaps ------------";
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<int32_t> distance(num_nodes, numeric_limits<int32_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    min_cost_flow.DijkstraOptimized(graph.get_supply_nodes(), distance,
                                    predecessor);
    min_cost_flow.logCosts(distance, predecessor);
  } else if (!FLAGS_algorithm.compare("cycle_cancelling")) {
    LOG(INFO) << "------------ Cycle cancelling min cost flow ------------";
    min_cost_flow.cycleCancelling();
  } else if (!FLAGS_algorithm.compare("successive_shortest_path")) {
    LOG(INFO) << "------------ Successive shortest path min cost flow "
              << "------------";
    min_cost_flow.successiveShortestPath();
  } else if (!FLAGS_algorithm.compare("successive_shortest_path_potentials")) {
    LOG(INFO) << "------------ Successive shortest path with potential min"
              << " cost flow ------------";
    min_cost_flow.successiveShortestPathPotentials();
  } else {
    LOG(ERROR) << "Unknown algorithm: " << FLAGS_algorithm;
  }
  LOG(INFO) << "------------ Writing flow graph ------------";
  graph.writeGraph(FLAGS_out_graph_file);
  return 0;
}
