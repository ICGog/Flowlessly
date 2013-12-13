#include "arc.h"

int32_t Arc::get_cap() {
  return cap;
}

int32_t Arc::get_flow() {
  return flow;
}

int64_t Arc::get_cost() {
  return cost;
}

uint32_t Arc::get_src_node_id() {
  return src_node_id;
}

uint32_t Arc::get_dst_node_id() {
  return dst_node_id;
}

Arc* Arc::get_reverse_arc() {
  return reverse_arc;
}

void Arc::set_reverse_arc(Arc* arc) {
  reverse_arc = arc;
}
