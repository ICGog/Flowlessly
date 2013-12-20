#include "cost_scaling.h"

#include "utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdint.h>

namespace flowlessly {

  using namespace std;

  void CostScaling::discharge(queue<uint32_t>& active_nodes,
                              vector<int64_t>& potential,
                              vector<int32_t>& nodes_demand, int64_t eps) {
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    while (!active_nodes.empty()) {
      uint32_t node_id = active_nodes.front();
      active_nodes.pop();
      int32_t cur_node_demand = nodes_demand[node_id];
      while (cur_node_demand > 0) {
        bool has_neg_cost_arc = false;
        for (map<uint32_t, Arc*>::iterator it = admisible_arcs[node_id].begin();
             it != admisible_arcs[node_id].end(); ) {
          has_neg_cost_arc = true;
          // Push flow.
          pushes_cnt++;
          int32_t min_flow = min(cur_node_demand, it->second->cap);
          it->second->cap -= min_flow;
          it->second->reverse_arc->cap += min_flow;
          cur_node_demand -= min_flow;
          // If node doesn't have any excess then it will be activated.
          if (nodes_demand[it->first] <= 0 &&
              nodes_demand[it->first] > -min_flow) {
            active_nodes.push(it->first);
          }
          nodes_demand[it->first] += min_flow;
          if (it->second->cap == 0) {
            map<uint32_t, Arc*>::iterator to_erase_it = it;
            ++it;
            admisible_arcs[node_id].erase(to_erase_it);
          } else {
            ++it;
          }
        }
        if (!has_neg_cost_arc) {
          // Relabel vertex.
          relabel_cnt++;
          /*
          int64_t new_pot = numeric_limits<int64_t>::min();
          for (map<uint32_t, Arc*>::iterator n_it = arcs[node_id].begin();
               n_it != arcs[node_id].end(); ++n_it) {
            if (n_it->second->cap > 0) {
              new_pot = max(new_pot,
                            potential[n_it->first] - n_it->second->cost - eps);
            }
          }
          int64_t refine_pot = potential[node_id] - new_pot;
          */
          int64_t refine_pot = eps;
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
        }
      }
      nodes_demand[node_id] = cur_node_demand;
    }
  }

  void CostScaling::refine(vector<int64_t>& potential, int64_t eps) {
    // Saturate arcs with negative reduced cost.
    ++refine_cnt;
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    // Saturate all the arcs with negative cost.
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      for (map<uint32_t, Arc*>::iterator it = admisible_arcs[node_id].begin();
           it != admisible_arcs[node_id].end(); ) {
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
    globalPotentialsUpdate(potential, eps);
    queue<uint32_t> active_nodes;
    for (uint32_t node_id = 1; node_id < num_nodes; ++node_id) {
      if (nodes_demand[node_id] > 0) {
        active_nodes.push(node_id);
      }
    }
    discharge(active_nodes, potential, nodes_demand, eps);
  }

  // Scales up costs by alpha * num_nodes
  // It returns the value from where eps should start.
  int64_t CostScaling::scaleUpCosts() {
    uint32_t num_nodes = graph_.get_num_nodes();
    const vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    int64_t max_cost_arc = numeric_limits<int64_t>::min();
    int64_t scale_up = FLAGS_alpha_scaling_factor * num_nodes;
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      for (map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
           it != arcs[node_id].end(); ++it) {
        it->second->cost *= scale_up;
        max_cost_arc = max(max_cost_arc, it->second->cost);
      }
    }
    return pow(FLAGS_alpha_scaling_factor,
               ceil(log(max_cost_arc) / log(FLAGS_alpha_scaling_factor)));
  }

  void CostScaling::costScaling() {
    //    eps = max arc cost
    //    potential(v) = 0
    //    Establish a feasible flow x in the network
    //    while eps >= 1/n do
    //      (e, f, p) = refine(e, f p)
    uint32_t eps_fixing_threshold = pow(FLAGS_alpha_scaling_factor, 5);
    uint32_t price_refine_threshold = 3;
    uint32_t eps_iteration_cnt = 0;
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<int64_t> potential(num_nodes, 0);
    relabel_cnt = 0;
    pushes_cnt = 0;
    refine_cnt = 0;
    for (int64_t eps = scaleUpCosts() / FLAGS_alpha_scaling_factor; eps >= 1;
         eps = eps < FLAGS_alpha_scaling_factor && eps > 1 ?
           1 : eps / FLAGS_alpha_scaling_factor) {
      ++eps_iteration_cnt;
      if (eps_iteration_cnt >= price_refine_threshold) {
        //        if (priceRefinement(potential, eps)) {
        //          continue;
        //        }
      }
      refine(potential, eps);
      if (eps <= eps_fixing_threshold) {
        //        arcsFixing(potential, 2 * (num_nodes - 1) * eps);
      }
    }
    //    arcsUnfixing(potential, numeric_limits<int64_t>::max());
    LOG(INFO) << "Num relables: " << relabel_cnt;
    LOG(INFO) << "Num pushes: " << pushes_cnt;
    LOG(INFO) << "Num refines: " << refine_cnt;
  }

  void CostScaling::globalPotentialsUpdate(vector<int64_t>& potential,
                                           int64_t eps) {
    uint32_t num_nodes = graph_.get_num_nodes();
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
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
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

    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      uint32_t min_rank = min(rank[node_id], bucket_index);
      if (min_rank > 0) {
        potential[node_id] -= eps * min_rank;
      }
    }
    updateAdmisibleGraph(potential);
  }

  // TODO(ionel): I suspect that this is not working because the input graph
  // is not eps optimal.
  bool CostScaling::priceRefinement(vector<int64_t>& potential, int64_t eps) {
    uint32_t num_nodes = graph_.get_num_nodes();
    uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    vector<uint32_t> ordered_nodes;
    vector<int64_t> distance(num_nodes + 1, 0);
    uint32_t bucket_end = num_nodes + 1;
    vector<uint32_t> bucket(max_rank + 1, bucket_end);
    vector<uint32_t> bucket_prev(num_nodes + 1, 0);
    vector<uint32_t> bucket_next(num_nodes + 1, 0);
    for (; graph_.orderTopologically(potential, ordered_nodes);
         ordered_nodes.clear()) {
      int64_t top_rank = 0;
      for (vector<uint32_t>::iterator node_it = ordered_nodes.begin();
           node_it != ordered_nodes.end(); ++node_it) {
        map<uint32_t, Arc*>::const_iterator it = admisible_arcs[*node_it].begin();
        map<uint32_t, Arc*>::const_iterator end_it =
          admisible_arcs[*node_it].end();
        int64_t d_node_it = distance[*node_it];
        for (; it != end_it; ++it) {
          int64_t reduced_cost = ceil((it->second->cost + potential[*node_it] -
                                       potential[it->first]) / eps);
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
              if (distance[node_id] < old_distance) {
                if (reduced_cost < 0) {
                  new_distance = distance[node_id];
                } else {
                  new_distance = distance[node_id] + ceil(reduced_cost / eps);
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
      for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
        potential[node_id] += distance[node_id] * eps;
      }
      updateAdmisibleGraph(potential);
    }
    return false;
  }

  // NOTE: if threshold is set to a smaller value than 2*n*eps then the
  // problem may become infeasable. Check the paper.
  void CostScaling::arcsFixing(vector<int64_t>& potential,
                               int64_t fix_threshold) {
    uint32_t num_nodes = graph_.get_num_nodes();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    list<Arc*>& fixed_arcs = graph_.get_fixed_arcs();
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
      map<uint32_t, Arc*>::iterator end_it = arcs[node_id].end();
      while (it != end_it) {
        if (it->second->cost + potential[node_id] - potential[it->first] >
            fix_threshold) {
          fixed_arcs.push_front(it->second);
          fixed_arcs.push_front(it->second->reverse_arc);
          map<uint32_t, Arc*>::iterator to_erase_it = it;
          uint32_t dst_node_id = it->first;
          ++it;
          arcs[node_id].erase(to_erase_it);
          arcs[dst_node_id].erase(node_id);
          admisible_arcs[dst_node_id].erase(node_id);
        } else {
          ++it;
        }
      }
    }
  }

  // NOTE: if threshold is set to a smaller value than 2*n*eps then the
  // problem may become infeasable. Check the paper.
  void CostScaling::arcsUnfixing(vector<int64_t>& potential,
                                 int64_t fix_threshold) {
    uint32_t num_nodes = graph_.get_num_nodes();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    list<Arc*>& fixed_arcs = graph_.get_fixed_arcs();
    for (list<Arc*>::iterator it = fixed_arcs.begin(); it != fixed_arcs.end(); ) {
      int64_t reduced_cost = (*it)->cost + potential[(*it)->src_node_id] -
        potential[(*it)->dst_node_id];
      if (reduced_cost < fix_threshold) {
        // Unfix node.
        list<Arc*>::iterator to_erase_it = it;
        arcs[(*it)->src_node_id][(*it)->dst_node_id] = *it;
        if (reduced_cost < 0 && (*it)->cap > 0) {
          admisible_arcs[(*it)->src_node_id][(*it)->dst_node_id] = *it;
        }
        ++it;
        fixed_arcs.erase(to_erase_it);
      } else {
        ++it;
      }
    }
  }

  void CostScaling::pushLookahead(uint32_t src_node_id, uint32_t dst_node_id) {
    /*
    uint32_t dst_node_id = it->first;
    has_neg_cost_arc = true;
    // Check if it can be pushed any flow.
    if (nodes_demand[dst_node_id] < 0 ||
        admisible_arcs[dst_node_id].size() > 0) {
      // Push flow.
      pushes_cnt++;
      int32_t min_flow = min(cur_node_demand, it->second->cap);
      it->second->cap -= min_flow;
      it->second->reverse_arc->cap += min_flow;
      cur_node_demand -= min_flow;
      // If node doesn't have any excess then it will be activated.
      if (nodes_demand[dst_node_id] <= 0 &&
          nodes_demand[dst_node_id] > -min_flow) {
        active_nodes.push(dst_node_id);
      }
      nodes_demand[dst_node_id] += min_flow;
      if (it->second->cap == 0) {
        map<uint32_t, Arc*>::iterator to_erase_it = it;
        ++it;
        admisible_arcs[node_id].erase(to_erase_it);
      } else {
        ++it;
      }
    } else {
      // Relabel dst_node.
      relabel_cnt++;
      int64_t refine_pot = eps;
      for (map<uint32_t, Arc*>::iterator n_it = arcs[dst_node_id].begin();
           n_it != arcs[dst_node_id].end(); ++n_it) {
        if (n_it->second->cap > 0) {
          int64_t reduced_cost = n_it->second->cost +
            potential[dst_node_id] - potential[n_it->first];
          if (reduced_cost >= 0 && reduced_cost < refine_pot) {
            admisible_arcs[dst_node_id][n_it->first] = n_it->second;
          }
        }
        // Check if the reverse arc is not saturated.
        if (n_it->second->reverse_arc->cap > 0) {
          int64_t reduced_cost = -n_it->second->cost +
            potential[n_it->first] - potential[dst_node_id];
          if (reduced_cost < 0 && reduced_cost >= -refine_pot) {
            admisible_arcs[n_it->first].erase(dst_node_id);
          }
        }
      }
      potential[dst_node_id] -= refine_pot;
      ++it;
    }
    */
  }

  void CostScaling::updateAdmisibleGraph(vector<int64_t>& potential) {
    uint32_t num_nodes = graph_.get_num_nodes();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      for (map<uint32_t, Arc*>::iterator it = arcs[node_id].begin();
           it != arcs[node_id].end(); ++it) {
        if (it->second->cost + potential[node_id] -
            potential[it->first] < 0 && it->second->cap > 0) {
          admisible_arcs[node_id][it->first] = it->second;
        } else {
          admisible_arcs[node_id].erase(it->first);
        }
      }
    }
  }

}
