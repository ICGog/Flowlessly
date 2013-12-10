#include "arc.h"

int32_t Arc::get_cap() {
  return cap;
}

int32_t Arc::get_flow() {
  return flow;
}

int32_t Arc::get_cost() {
  return cost;
}

Arc* Arc::get_reverse_arc() {
  return reverse_arc;
}

void Arc::set_reverse_arc(Arc* arc) {
  reverse_arc = arc;
}
