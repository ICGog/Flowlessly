#include "cost_scaling.h"
#include "cycle_cancelling.h"
#include "graph.h"
#include "successive_shortest.h"
#include "utils.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <limits>
#include <cstdio>

using namespace flowlessly;

DEFINE_string(graph_file, "", "Input graph file (stdin if empty, default).");
DEFINE_string(out_graph_file, "", "Output graph file (stdout if empty, default.");
DEFINE_string(algorithm, "cost_scaling",
              "Algorithms to run: cycle_cancelling, bellman_ford, dijkstra, dijkstra_heap, successive_shortest_path, cost_scaling, check_flow");
DEFINE_int64(alpha_scaling_factor, 2,
             "Value by which Eps is divided in the cost scaling algorithm");
DEFINE_string(flow_file, "", "File containing the min-cost flow graph");
DEFINE_bool(global_update, true, "Activate global update heuristic");
DEFINE_bool(price_refinement, true, "Activate price refinement heuristic");
DEFINE_bool(push_lookahead, true, "Activate push lookahead heuristic");
DEFINE_bool(arc_fixing, true, "Activate arc fixing heuristic");
DEFINE_int64(arc_fixing_threshold, 3,
             "After how many refines to start fixing arcs");
DEFINE_int64(price_refine_threshold, 3,
             "After how many iterations to start price refinement");
DEFINE_bool(incremental, false, "Activate incremental tests");
DEFINE_int64(num_iterations, 2, "Number of scheduling iterations");
DEFINE_int64(task_completion_percentage, 0,
             "Percentage of completed tasks in each scheduling iteration");
DEFINE_bool(log_statistics, false, "Log statistics flag");
DEFINE_int64(num_preference_arcs, 20, "Number of preference arcs");

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

/*! \mainpage Flowlessly Documentation
 * Welcome to the documentation page of Flowlessly.
 */

//! Flowlessly namespace.
namespace flowlessly {
}

void runBellmanFord(Graph& graph) {
  LOG(INFO) << "------------ BellmanFord ------------";
  uint32_t num_nodes = graph.get_num_nodes() + 1;
  vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  BellmanFord(graph, graph.get_source_nodes(), distance, predecessor);
  logCosts(distance, predecessor);
}

void runDijkstra(Graph& graph, bool with_heaps) {
  LOG(INFO) << "------------ Dijkstra ------------";
  uint32_t num_nodes = graph.get_num_nodes() + 1;
  vector<int64_t> distance(num_nodes, numeric_limits<int64_t>::max());
  vector<uint32_t> predecessor(num_nodes, 0);
  if (with_heaps) {
    DijkstraOptimized(graph, graph.get_source_nodes(), distance, predecessor);
  } else {
    DijkstraSimple(graph, graph.get_source_nodes(), distance, predecessor);
  }
  logCosts(distance, predecessor);
}

int main(int argc, char *argv[]) {
  init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;
  Statistics stats;
  Graph graph(stats);
  double read_start_time = stats.getTime();
  FILE* graph_file = NULL;
  FILE* out_graph_file = NULL;
  if (FLAGS_graph_file.empty()) {
    graph_file = stdin;
  } else {
    if ((graph_file = fopen(FLAGS_graph_file.c_str(), "r")) == NULL) {
      LOG(ERROR) << "Failed to open graph file: " << FLAGS_graph_file;
      return 1;
    }
  }
  graph.readGraph(graph_file);
  double read_end_time = stats.getTime();
  int32_t num_iterations = 1;
  if (FLAGS_incremental) {
    num_iterations = FLAGS_num_iterations;
  }
  for (; num_iterations > 0; --num_iterations) {
    double algo_start_time = stats.getTime();
    if (!FLAGS_algorithm.compare("bellman_ford")) {
      runBellmanFord(graph);
    } else if (!FLAGS_algorithm.compare("dijkstra")) {
      runDijkstra(graph, false);
    } else if (!FLAGS_algorithm.compare("dijkstra_heap")) {
      runDijkstra(graph, true);
    } else if (!FLAGS_algorithm.compare("cycle_cancelling")) {
      LOG(INFO) << "------------ Cycle cancelling min cost flow ------------";
      CycleCancelling cycle_cancelling(graph);
      cycle_cancelling.cycleCancelling(false);
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
      CostScaling min_cost_flow(graph, stats);
      min_cost_flow.costScaling();
    } else if (!FLAGS_algorithm.compare("check_flow")) {
      if (!FLAGS_flow_file.compare("")) {
        LOG(ERROR) << "Please set the flow_file argument";
        return -1;
      }
      if (graph.checkFlow(FLAGS_flow_file)) {
        LOG(INFO) << "Flow is valid";
        // TODO(ionel): Check eps optimality as well.
      } else {
        LOG(ERROR) << "Flow is not valid";
      }
    } else {
      LOG(ERROR) << "Unknown algorithm: " << FLAGS_algorithm;
    }
    double algo_end_time = stats.getTime();
    if (FLAGS_log_statistics) {
      stats.logTimeStatistics();
      stats.clearStatistics();
    }
    LOG(INFO) << "Algorithm run time: " << algo_end_time - algo_start_time;
    uint32_t num_removed =
      graph.removeTaskNodes(FLAGS_task_completion_percentage);
    for (; num_removed > 0; --num_removed) {
      graph.addTaskNode();
    }
    graph.resetPotentials();
  }
  LOG(INFO) << "------------ Writing flow graph ------------";
  double write_start_time = stats.getTime();
  if (FLAGS_out_graph_file.empty()) {
    out_graph_file = stdout;
  } else {
     if ((out_graph_file = fopen(FLAGS_out_graph_file.c_str(), "w")) == NULL) {
      LOG(ERROR) << "Could no open graph file for writing: " << FLAGS_out_graph_file;
    }
  }
  graph.writeFlowGraph(out_graph_file);
  double write_end_time = stats.getTime();
  LOG(INFO) << "Read time: " << read_end_time - read_start_time;
  LOG(INFO) << "Write time: " << write_end_time - write_start_time;
  LOG(INFO) << "Total time: " << write_end_time - read_start_time;
  fclose(graph_file);
  fclose(out_graph_file);
  return 0;
}
