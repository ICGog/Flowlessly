#include "graph.h"
#include "min_cost_flow.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(graph_file, "graph.in", "File containing the input graph.");
DEFINE_string(out_graph_file, "graph.out",
              "File the output graph will be written");
DEFINE_string(algorithm, "cycle_cancelling",
              "Algorithms to run: cycle_cancelling, bellman_ford, dijkstra, dijkstra_heap");

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
  graph.printGraph();
  MinCostFlow min_cost_flow(graph);
  if (!FLAGS_algorithm.compare("bellman_ford")) {
    LOG(INFO) << "------------ BellmanFord ------------";
    min_cost_flow.BellmanFord(graph.get_supply_nodes());
  } else if (!FLAGS_algorithm.compare("dijkstra")) {
    LOG(INFO) << "------------ Dijkstra ------------";
    min_cost_flow.DijkstraSimple(graph.get_supply_nodes());
  } else if (!FLAGS_algorithm.compare("dijkstra_heap")) {
    LOG(INFO) << "------------ Dijkstra with heaps ------------";
    min_cost_flow.DijkstraOptimized(graph.get_supply_nodes());
  } else if (!FLAGS_algorithm.compare("cycle_cancelling")) {
    LOG(INFO) << "------------ Cycle cancelling min cost flow ------------";
    min_cost_flow.cycleCancelling();
  } else {
    LOG(ERROR) << "Unknown algorithm: " << FLAGS_algorithm;
  }
  LOG(INFO) << "------------ Writing flow graph ------------";
  graph.writeGraph(FLAGS_out_graph_file);
  return 0;
}
