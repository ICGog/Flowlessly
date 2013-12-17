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
    vector<map<uint32_t, Arc*> >& admisible_arcs = graph_.get_admisible_arcs();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    while (!active_nodes.empty()) {
      uint32_t node_id = active_nodes.front();
      active_nodes.pop();
      while (nodes_demand[node_id] > 0) {
        bool has_neg_cost_arc = false;
        for (map<uint32_t, Arc*>::iterator it = admisible_arcs[node_id].begin();
             it != admisible_arcs[node_id].end(); ) {
          pushes_cnt++;
          has_neg_cost_arc = true;
          // Push flow.
          int32_t min_flow = min(nodes_demand[node_id], it->second->cap);
          LOG(INFO) << "Pushing flow " << min_flow << " on (" << node_id
                    << ", " << it->first << ")";
          it->second->cap -= min_flow;
          it->second->reverse_arc->cap += min_flow;
          nodes_demand[node_id] -= min_flow;
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
          LOG(INFO) << "Potential of " << node_id << ": " << potential[node_id];
        }
      }
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
    graph_.logGraph();
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
    uint32_t num_nodes = graph_.get_num_nodes() + 1;
    vector<int64_t> potential(num_nodes, 0);
    relabel_cnt = 0;
    pushes_cnt = 0;
    refine_cnt = 0;
    for (int64_t eps = scaleUpCosts() / FLAGS_alpha_scaling_factor; eps >= 1;
         eps = eps < FLAGS_alpha_scaling_factor && eps > 1 ?
           1 : eps / FLAGS_alpha_scaling_factor) {
      graph_.logGraph();
      refine(potential, eps);
      //      arcsFixing(potential, 2 * (num_nodes - 1) * eps);
    }
    //    arcsUnfixing(potential, numeric_limits<int64_t>::max());
    LOG(ERROR) << "Num relables: " << relabel_cnt;
    LOG(ERROR) << "Num pushes: " << pushes_cnt;
    LOG(ERROR) << "Num refines: " << refine_cnt;
  }

  void CostScaling::globalPotentialsUpdate(vector<int64_t>& potential,
                                           int64_t eps) {
    uint32_t num_nodes = graph_.get_num_nodes();
    // Variable used to denote an empty bucket.
    uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
    uint32_t bucket_end = num_nodes + 1;
    vector<int64_t> rank(num_nodes + 1, 0);
    vector<uint32_t> bucket(max_rank + 1, 0);
    vector<uint32_t> bucket_prev(num_nodes + 1, 0);
    vector<uint32_t> bucket_next(num_nodes + 1, 0);
    vector<int32_t>& nodes_demand = graph_.get_nodes_demand();
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    uint32_t num_active_nodes = 0;
    // Initialize buckets.
    for (uint32_t cur_rank = 0; cur_rank <= max_rank; ++cur_rank) {
      bucket[cur_rank] = bucket_end;
    }
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
    int32_t bucket_index = 0;
    for ( ; num_active_nodes > 0 && bucket_index <= max_rank; ++bucket_index) {
      while (bucket[bucket_index] != bucket_end) {
        uint32_t node_id = bucket[bucket_index];
        bucket[bucket_index] = bucket_next[node_id];
        map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
        map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
        for (; it != end_it; ++it) {
          Arc* rev_arc = it->second->reverse_arc;
          if (rev_arc->cap > 0 && bucket_index < rank[it->first]) {
            int64_t k = floor((rev_arc->cost + potential[it->first] -
                               potential[node_id]) / eps) + 1 + bucket_index;
            int64_t old_rank = rank[it->first];
            if (k < rank[it->first]) {
              rank[it->first] = k;
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
        if (nodes_demand[node_id] > 0) {
          num_active_nodes--;
        }
        if (num_active_nodes == 0) {
          break;
        }
      }
      if (num_active_nodes == 0) {
        break;
      }
    }
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      int64_t min_rank = min(rank[node_id], (int64_t)bucket_index);
      if (min_rank > 0) {
        potential[node_id] -= eps * min_rank;
      }
    }
  }

  bool CostScaling::priceRefinement(vector<int64_t>& potential, int64_t eps) {
    uint32_t num_nodes = graph_.get_num_nodes();
    uint32_t max_rank = FLAGS_alpha_scaling_factor * num_nodes;
    vector<map<uint32_t, Arc*> >& arcs = graph_.get_arcs();
    vector<uint32_t> ordered_nodes;
    vector<int64_t> distance(num_nodes + 1, 0);
    uint32_t bucket_end = num_nodes + 1;
    vector<uint32_t> bucket(max_rank + 1, 0);
    vector<uint32_t> bucket_prev(num_nodes + 1, 0);
    vector<uint32_t> bucket_next(num_nodes + 1, 0);
    if (!graph_.orderTopologically(potential, ordered_nodes)) {
      // Graph contains a cycle. Cannot update potential
      return false;
    }
    for (vector<uint32_t>::iterator node_it = ordered_nodes.begin();
         node_it != ordered_nodes.end(); ++node_it) {
      map<uint32_t, Arc*>::const_iterator it = arcs[*node_it].begin();
      map<uint32_t, Arc*>::const_iterator end_it = arcs[*node_it].end();
      for (; it != end_it; ++it) {
        int64_t reduced_cost = ceil((it->second->cost + potential[*node_it] -
                                     potential[it->first]) / eps);
        if (distance[*node_it] + reduced_cost < distance[it->first]) {
          distance[it->first] = distance[*node_it] + reduced_cost;
        }
      }
    }
    // Insert node_id at -distance[node_id].
    for (uint32_t node_id = 1; node_id <= num_nodes; ++node_id) {
      uint32_t bucket_index = -distance[node_id];
      bucket_next[node_id] = bucket[bucket_index];
      bucket_prev[bucket[bucket_index]] = node_id;
      bucket[bucket_index] = node_id;
    }
    for (int32_t bucket_index = max_rank; bucket_index >= 0; --bucket_index) {
      while (bucket[bucket_index] != bucket_end) {
        uint32_t node_id = bucket[bucket_index];
        bucket[bucket_index] = bucket_next[node_id];
        map<uint32_t, Arc*>::const_iterator it = arcs[node_id].begin();
        map<uint32_t, Arc*>::const_iterator end_it = arcs[node_id].end();
        for (; it != end_it; ++it) {
        }
      }
    }
    return true;
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
          // Fix node.
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
  }

}
