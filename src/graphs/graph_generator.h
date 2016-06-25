// The Flowlessly project
// Copyright (c) 2013-2016 Ionel Gog <ionel.gog@cl.cam.ac.uk>

#ifndef FLOWLESSLY_GRAPH_GENERATOR_H
#define FLOWLESSLY_GRAPH_GENERATOR_H

#include <glog/logging.h>
#include <vector>

#include "graphs/adjacency_map_graph.h"

namespace flowlessly {

class GraphGenerator {
 public:
  GraphGenerator();
  explicit GraphGenerator(uint32_t seed);
  uint32_t Generate(uint32_t num_tasks, uint32_t num_machines,
                    vector<uint32_t>* machine_node_ids,
                    Graph* graph, FILE* out_graph_file);

  void GenerateChanges(uint32_t cluster_agg_node_id, uint32_t* num_machines,
                       vector<uint32_t>* machine_node_ids, uint32_t* num_tasks,
                       Graph* graph, FILE* out_changes_file);
  uint32_t GenerateMachine(uint32_t sink_node_id,
                           uint32_t cluster_agg_node_id,
                           bool first_scheduling_iteration,
                           Graph* graph,
                           uint32_t* cur_rank_node_id,
                           uint32_t* machines_per_current_rack,
                           FILE* out_changes_file);
  void GenerateTask(uint32_t cluster_agg_node_id,
                    uint32_t num_machines,
                    const vector<uint32_t>& machine_node_ids,
                    bool first_scheduling_iteration,
                    Graph* graph,
                    FILE* out_changes_file);

 private:
  void GenerateArcChanges(uint32_t num_arc_changes,
                          vector<uint32_t>* machine_node_ids,
                          Graph* graph,
                          FILE* out_changes_file);

  uint32_t seed_;
};

} // namespace flowlessly
#endif // FLOWLESSLY_GRAPH_GENERATOR_H
