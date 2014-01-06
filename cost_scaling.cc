#include "cost_scaling.h"

#include "statistics.h"
#include "utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdint.h>

namespace flowlessly {

  using namespace std;

  void CostScaling::discharge(queue<uint32_t>& active_nodes,
                              vector<int32_t>& nodes_demand, int64_t eps) {
    statistics.update_discharge_start_time();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    while (!active_nodes.empty()) {
      uint32_t node_id = active_nodes.front();
      active_nodes.pop();
      while (nodes_demand[node_id] > 0) {
        bool has_neg_cost_arc = false;
        map<uint32_t, Arc*>::iterator end_it = admisible_arcs[node_id].end();
        for (map<uint32_t, Arc*>::iterator it = admisible_arcs[node_id].begin();
             it != end_it; ) {
          map<uint32_t, Arc*>::iterator to_push_it = it;
          ++it;
          if (FLAGS_push_lookahead) {
            bool pushed = pushLookahead(to_push_it->second, active_nodes,
                                        nodes_demand, eps);
            if (pushed) {
              has_neg_cost_arc = true;
            }
            if (pushed && to_push_it->second->cap == 0) {
              admisible_arcs[node_id].erase(to_push_it);
            }
          } else {
            has_neg_cost_arc = true;
            push(to_push_it->second, active_nodes, nodes_demand);
            if (to_push_it->second->cap == 0) {
              admisible_arcs[node_id].erase(to_push_it);
            }
          }
        }
        if (!has_neg_cost_arc) {
          relabel(node_id, eps);
        }
      }
    }
    statistics.update_discharge_end_time();
  }

  void CostScaling::refine(int64_t eps) {
    statistics.update_refine_start_time();
    // Saturate arcs with negative reduced cost.
    ++refine_cnt;
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    // Saturate all the arcs with negative cost.
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      map<uint32_t, Arc*>::iterator end_it = admisible_arcs[node_id].end();
      for (map<uint32_t, Arc*>::iterator it = admisible_arcs[node_id].begin();
           it != end_it; ) {
        nodes_demand[node_id] -= it->second->cap;
        nodes_demand[it->first] += it->second->cap;
        it->second->reverse_arc->cap += it->second->cap;
        it->second->cap = 0;
        // Remove arc from the admisible graph.
        map<uint32_t, Arc*>::iterator to_erase_it = it;
        ++it;
        admisible_arcs[node_id].erase(to_erase_it);
      }
    }
    if (FLAGS_global_update) {
      globalPotentialsUpdate(eps);
    }
    queue<uint32_t> active_nodes;
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      if (nodes_demand[node_id] > 0) {
        active_nodes.push(node_id);
      }
    }
    discharge(active_nodes, nodes_demand, eps);
    statistics.update_refine_end_time();
  }

  void CostScaling::costScaling() {
    const uint32_t num_nodes = graph_.get_num_nodes() + 1;
    const int64_t scaling_factor = FLAGS_alpha_scaling_factor * num_nodes;
    uint32_t eps_iteration_cnt = 0;
    relabel_cnt = 0;
    pushes_cnt = 0;
    refine_cnt = 0;
    for (int64_t eps = graph_.scaleUpCosts(scaling_factor) /
           FLAGS_alpha_scaling_factor; eps >= 1;
         eps = eps < FLAGS_alpha_scaling_factor && eps > 1 ?
           1 : eps / FLAGS_alpha_scaling_factor, ++eps_iteration_cnt) {
      if (FLAGS_price_refinement &&
          eps_iteration_cnt >= FLAGS_price_refine_threshold) {
        if (priceRefinement(eps)) {
          continue;
        }
      }
      refine(eps);
      if (FLAGS_arc_fixing &&
          eps_iteration_cnt >= FLAGS_arc_fixing_threshold) {
        graph_.arcsFixing(2 * (num_nodes - 1) * eps);
      }
    }
    if (FLAGS_arc_fixing) {
      graph_.arcsUnfixing(numeric_limits<int64_t>::max());
    }
    graph_.scaleDownCosts(scaling_factor);
    LOG(INFO) << "Num relables: " << relabel_cnt;
    LOG(INFO) << "Num pushes: " << pushes_cnt;
    LOG(INFO) << "Num refines: " << refine_cnt;
  }

  void CostScaling::globalPotentialsUpdate(int64_t eps) {
    statistics.update_global_update_start_time();
    const uint32_t num_nodes = graph_.get_num_nodes();
    vector<int64_t>& potential = graph_.get_potential();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    // Variable used to denote an empty bucket.
    // TODO(ionel): Goldberg says that max_rank should be set to eps * n, while
    // in Lemon is set to alpha * n. I think it should be eps * n, however, the
    // smaller value works as well. I'm not sure why.
    uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
    uint32_t bucket_end = num_nodes + 1;
    vector<uint32_t> rank(num_nodes + 1, 0);
    vector<uint32_t> bucket(max_rank + 1, bucket_end);
    vector<uint32_t> bucket_prev(num_nodes + 1, 0);
    vector<uint32_t> bucket_next(num_nodes + 1, 0);
    uint32_t num_active_nodes = 0;
    // Put nodes with negative excess in bucket[0].
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      if (nodes_demand[node_id] < 0) {
        rank[node_id] = 0;
        bucket_next[node_id] = bucket[0];
        bucket_prev[bucket[0]] = node_id;
        bucket[0] = node_id;
      } else {
        rank[node_id] = max_rank + 1;
        if (nodes_demand[node_id] > 0) {
          num_active_nodes++;
        }
      }
    }
    // TODO(ionel): Explore if returning while a small number of nodes are
    // still active improves runtime.
    // Return if there are no active nodes.
    if (num_active_nodes == 0) {
      statistics.update_global_update_end_time();
      return;
    }
    uint32_t bucket_index = 0;
    for ( ; num_active_nodes > 0 && bucket_index <= max_rank; ++bucket_index) {
      for (; num_active_nodes > 0 && bucket[bucket_index] != bucket_end; ) {
        uint32_t node_id = bucket[bucket_index];
        bucket[bucket_index] = bucket_next[node_id];
        for (map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
             it != arcs[node_id].end(); ++it) {
          Arc* rev_arc = it->second->reverse_arc;
          int64_t old_rank = rank[it->first];
          if (rev_arc->cap > 0 && bucket_index < old_rank) {
            int64_t k = floor((rev_arc->cost + potential[it->first] -
                               potential[node_id]) / eps) + 1 + bucket_index;
            if (k < old_rank) {
              rank[it->first] = k;
              if (k <= max_rank) {
                // Remove node from the old bucket.
                if (old_rank <= max_rank) {
                  // Check if node is first element.
                  if (bucket[old_rank] == it->first) {
                    bucket[old_rank] = bucket_next[it->first];
                  } else {
                    uint32_t prev = bucket_prev[it->first];
                    uint32_t next = bucket_next[it->first];
                    bucket_next[prev] = next;
                    bucket_prev[next] = prev;
                  }
                }
                // Insert into the new bucket.
                bucket_next[it->first] = bucket[k];
                bucket_prev[bucket[k]] = it->first;
                bucket[k] = it->first;
              }
            }
          }
        }
        if (nodes_demand[node_id] > 0) {
          num_active_nodes--;
        }
      }
    }
    vector<uint32_t> updated_nodes;
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      uint32_t min_rank = min(rank[node_id], bucket_index);
      if (min_rank > 0) {
        potential[node_id] -= eps * min_rank;
        updated_nodes.push_back(node_id);
      }
    }
    updateAdmisibleGraph(updated_nodes);
    statistics.update_global_update_end_time();
  }

  bool CostScaling::priceRefinement(int64_t eps) {
    statistics.update_price_refine_start_time();
    const uint32_t num_nodes = graph_.get_num_nodes();
    vector<int64_t>& potential = graph_.get_potential();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
    vector<uint32_t> ordered_nodes;
    vector<int64_t> distance(num_nodes + 1, 0);
    uint32_t bucket_end = num_nodes + 1;
    vector<uint32_t> bucket(max_rank + 1, bucket_end);
    vector<uint32_t> bucket_prev(num_nodes + 1, 0);
    vector<uint32_t> bucket_next(num_nodes + 1, 0);
    for (; graph_.orderTopologically(ordered_nodes);
         ordered_nodes.clear()) {
      int64_t top_rank = 0;
      fill(distance.begin(), distance.end(), 0);
      for (vector<uint32_t>::iterator node_it = ordered_nodes.begin();
           node_it != ordered_nodes.end(); ++node_it) {
        map<uint32_t, Arc*>::const_iterator it =
          admisible_arcs[*node_it].begin();
        map<uint32_t, Arc*>::const_iterator end_it =
          admisible_arcs[*node_it].end();
        int64_t d_node_it = distance[*node_it];
        for (; it != end_it; ++it) {
          int64_t reduced_cost = (it->second->cost + potential[*node_it] -
                                  potential[it->first] + 0.5) / eps;
          if (d_node_it + reduced_cost < distance[it->first]) {
            distance[it->first] = d_node_it + reduced_cost;
          }
        }
        // Insert *node_it at -distance[*node_it]
        uint32_t bucket_index = -d_node_it;
        if (bucket_index > 0) {
          bucket_next[*node_it] = bucket[bucket_index];
          bucket_prev[bucket[bucket_index]] = *node_it;
          bucket[bucket_index] = *node_it;
          if (bucket_index > top_rank) {
            top_rank = bucket_index;
          }
        }
      }
      // Current flow is eps optimal.
      if (top_rank == 0) {
        statistics.update_price_refine_end_time();
        return true;
      }
      for (int32_t bucket_index = top_rank; bucket_index > 0; --bucket_index) {
        while (bucket[bucket_index] != bucket_end) {
          uint32_t node_id = bucket[bucket_index];
          bucket[bucket_index] = bucket_next[node_id];
          for (map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
               it != arcs[node_id].end(); ++it) {
            Arc* arc = it->second;
            if (arc->cap > 0) {
              int64_t reduced_cost = arc->cost + potential[node_id] -
                potential[it->first];
              int64_t old_distance = distance[it->first];
              int64_t new_distance = old_distance;
              if (bucket_index < old_distance) {
                if (reduced_cost < 0) {
                  new_distance = bucket_index;
                } else {
                  new_distance = bucket_index + (reduced_cost + 0.5) / eps;
                }
                if (new_distance < old_distance) {
                  distance[it->first] = new_distance;
                  if (old_distance < 0) {
                    if (bucket[old_distance] == it->first) {
                      bucket[old_distance] = bucket_next[it->first];
                    } else {
                      uint32_t prev = bucket_prev[it->first];
                      uint32_t next = bucket_next[it->first];
                      bucket_next[prev] = next;
                      bucket_prev[next] = prev;
                    }
                  }
                  // Insert into new bucket.
                  bucket_next[it->first] = bucket[-new_distance];
                  bucket_prev[bucket[-new_distance]] = it->first;
                  bucket[-new_distance] = it->first;
                }
              }
            }
          }
        }
      }
      vector<uint32_t> updated_nodes;
      for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
        potential[node_id] += distance[node_id] * eps;
        if (distance[node_id] != 0) {
          updated_nodes.push_back(node_id);
        }
      }
      updateAdmisibleGraph(updated_nodes);
    }
    statistics.update_price_refine_end_time();
    return false;
  }

  void CostScaling::push(Arc* arc, queue<uint32_t>& active_nodes,
                         vector<int32_t>& nodes_demand) {
    statistics.update_push_start_time();
    pushes_cnt++;
    int32_t min_flow = min(nodes_demand[arc->src_node_id], arc->cap);
    arc->cap -= min_flow;
    arc->reverse_arc->cap += min_flow;
    if (nodes_demand[arc->dst_node_id] <= 0 &&
        nodes_demand[arc->dst_node_id] > -min_flow) {
      active_nodes.push(arc->dst_node_id);
    }
    nodes_demand[arc->dst_node_id] += min_flow;
    nodes_demand[arc->src_node_id] -= min_flow;
    statistics.update_push_end_time();
  }

  bool CostScaling::pushLookahead(Arc* arc, queue<uint32_t>& active_nodes,
                                  vector<int32_t>& nodes_demand, int64_t eps) {
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    if (nodes_demand[arc->dst_node_id] < 0 ||
        admisible_arcs[arc->dst_node_id].size() > 0) {
      statistics.update_push_start_time();
      pushes_cnt++;
      int32_t cur_node_demand = nodes_demand[arc->src_node_id];
      int32_t min_flow = min(cur_node_demand, arc->cap);
      arc->cap -= min_flow;
      arc->reverse_arc->cap += min_flow;
      cur_node_demand -= min_flow;
      // If node doesn't have any excess then it will be activated.
      if (nodes_demand[arc->dst_node_id] <= 0 &&
          nodes_demand[arc->dst_node_id] > -min_flow) {
        active_nodes.push(arc->dst_node_id);
      }
      nodes_demand[arc->dst_node_id] += min_flow;
      nodes_demand[arc->src_node_id] = cur_node_demand;
      statistics.update_push_end_time();
      return true;
    } else {
      relabel(arc->dst_node_id, eps);
      return false;
    }
  }

  void CostScaling::updateAdmisibleGraph(vector<uint32_t>& updated_nodes) {
    statistics.update_admisible_start_time();
    vector<int64_t>& potential = graph_.get_potential();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    for (vector<uint32_t>::iterator node_it = updated_nodes.begin();
         node_it != updated_nodes.end(); ++node_it) {
      for (map<uint32_t, Arc*>::iterator it = arcs[*node_it].begin();
           it != arcs[*node_it].end(); ++it) {
        if (it->second->cost + potential[*node_it] - potential[it->first] < 0 &&
            it->second->cap > 0) {
          admisible_arcs[*node_it][it->first] = it->second;
        } else {
          admisible_arcs[*node_it].erase(it->first);
        }
        Arc* rev_arc = it->second->reverse_arc;
        if (rev_arc->cost + potential[it->first] - potential[*node_it] < 0 &&
            rev_arc->cap > 0) {
          admisible_arcs[it->first][*node_it] = rev_arc;
        } else {
          admisible_arcs[it->first].erase(*node_it);
        }
      }
    }
    statistics.update_admisible_end_time();
  }

  void CostScaling::relabel(uint32_t node_id, int64_t eps) {
    statistics.update_relabel_start_time();
    relabel_cnt++;
    vector<int64_t>& potential = graph_.get_potential();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    int64_t refine_pot = graph_.getRefinePotential(node_id, eps);
    //    int64_t refine_pot = eps;
    for (map<uint32_t, Arc*>::iterator n_it = arcs[node_id].begin();
         n_it != arcs[node_id].end(); ++n_it) {
      if (n_it->second->cap > 0) {
        int64_t reduced_cost = n_it->second->cost + potential[node_id] -
          potential[n_it->first];
        if (reduced_cost >= 0 && reduced_cost < refine_pot) {
          admisible_arcs[node_id][n_it->first] = n_it->second;
        }
      }
      // Check if the reverse arc is not saturated.
      if (n_it->second->reverse_arc->cap > 0) {
        int64_t reduced_cost = -n_it->second->cost +
          potential[n_it->first] - potential[node_id];
        if (reduced_cost < 0 && reduced_cost >= -refine_pot) {
          admisible_arcs[n_it->first].erase(node_id);
        }
      }
    }
    potential[node_id] -= refine_pot;
    statistics.update_relabel_end_time();
  }

}
