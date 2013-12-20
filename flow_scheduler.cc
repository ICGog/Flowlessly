#include "cost_scaling.h"
#include "cycle_cancelling.h"
#include "graph.h"
#include "successive_shortest.h"
#include "utils.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <limits>

using namespace flowlessly;

DEFINE_string(graph_file, "graph.in", "File containing the input graph.");
DEFINE_string(out_graph_file, "graph.out",
              "File the output graph will be written");
DEFINE_string(algorithm, "cycle_cancelling",
              "Algorithms to run: cycle_cancelling, bellman_ford, dijkstra, dijkstra_heap, successive_shortest_path, cost_scaling, check_flow");
DEFINE_int64(alpha_scaling_factor, 2,
             "Value by which Eps is divided in the cost scaling algorithm");
DEFINE_string(flow_file, "", "File containing the min-cost flow graph");
DEFINE_bool(global_update, true, "Activate global update heuristic");
DEFINE_bool(price_refinment, false, "Activate price refinement heuristic");
DEFINE_bool(push_lookahead, true, "Activate push lookahead heuristic");
DEFINE_bool(arc_fixing, true, "Activate arc fixing heuristic");
DEFINE_int64(arc_fixing_threshold, 3, "After how many refines to start fixing arcs");
DEFINE_int64(price_refine_threshold, 3, "After how many iterations to start price refinement");

inline void init(int argc, char *argv[]) {
  // Set up usage message.
  string usage("Sample usage:\nflow_scheduler -algorithm cost_scaling");
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
  double read_start_time = getTime();
  graph.readGraph(FLAGS_graph_file);
  int64_t scale_down = 1;
  double algo_start_time = getTime();
  if (!FLAGS_algorithm.compare("bellman_ford")) {
    LOG(INFO) << "------------ BellmanFord ------------";
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    BellmanFord(graph, graph.get_source_nodes(), distance, predecessor);
    logCosts(distance, predecessor);
  } else if (!FLAGS_algorithm.compare("dijkstra")) {
    LOG(INFO) << "------------ Dijkstra ------------";
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    DijkstraSimple(graph, graph.get_source_nodes(), distance, predecessor);
    logCosts(distance, predecessor);
  } else if (!FLAGS_algorithm.compare("dijkstra_heap")) {
    LOG(INFO) << "------------ Dijkstra with heaps ------------";
    uint32_t num_nodes = graph.get_num_nodes() + 1;
    vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
    vector<uint32_t> predecessor(num_nodes, 0);
    DijkstraOptimized(graph, graph.get_source_nodes(), distance, predecessor);
    logCosts(distance, predecessor);
  } else if (!FLAGS_algorithm.compare("cycle_cancelling")) {
    LOG(INFO) << "------------ Cycle cancelling min cost flow ------------";
    CycleCancelling cycle_cancelling(graph);
    cycle_cancelling.cycleCancelling();
  } else if (!FLAGS_algorithm.compare("successive_shortest_path")) {
    LOG(INFO) << "------------ Successive shortest path min cost flow "
              << "------------";
    SuccessiveShortest successive_shortest(graph);
    successive_shortest.successiveShortestPath();
  } else if (!FLAGS_algorithm.compare("successive_shortest_path_potentials")) {
    LOG(INFO) << "------------ Successive shortest path with potential min"
              << " cost flow ------------";
    SuccessiveShortest successive_shortest(graph);
    successive_shortest.successiveShortestPathPotentials();
  } else if (!FLAGS_algorithm.compare("cost_scaling")) {
    LOG(INFO) << "------------ Cost scaling min cost flow ------------";
    CostScaling min_cost_flow(graph);
    min_cost_flow.costScaling();
    scale_down = FLAGS_alpha_scaling_factor * graph.get_num_nodes();
  } else if (!FLAGS_algorithm.compare("check_flow")) {
    if (!FLAGS_flow_file.compare("")) {
      LOG(ERROR) << "Please set the flow_file argument";
      return -1;
    }
    if (graph.checkFlow(FLAGS_flow_file)) {
      LOG(INFO) << "Flow is valid";
    } else {
      LOG(ERROR) << "Flow is not valid";
    }
  } else {
    LOG(ERROR) << "Unknown algorithm: " << FLAGS_algorithm;
  }
  LOG(INFO) << "------------ Writing flow graph ------------";
  double algo_end_time = getTime();
  graph.writeGraph(FLAGS_out_graph_file, scale_down);
  double write_end_time = getTime();
  LOG(INFO) << "Read time: " << algo_start_time - read_start_time;
  LOG(INFO) << "Algorithm run time: " << algo_end_time - algo_start_time;
  LOG(INFO) << "Write time: " << write_end_time - algo_end_time;
  LOG(INFO) << "Total time: " << write_end_time - read_start_time;
  return 0;
}
