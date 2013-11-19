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

bool Arc::get_reverse() {
  return reverse;
}
