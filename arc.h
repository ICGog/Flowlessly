#ifndef ARC_H
#define ARC_H

#include <stdint.h>

class Arc {

 public:
 Arc(): cap(0), flow(0), cost(0) {
 }

 Arc(int32_t capacity, uint32_t flw, int32_t cst): cap(capacity), flow(flw),
    cost(cst) {
 }

  int32_t get_cap();
  uint32_t get_flow();
  int32_t get_cost();

  int32_t cap;
  uint32_t flow;
  int32_t cost;

};
#endif
