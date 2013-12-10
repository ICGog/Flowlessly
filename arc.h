#ifndef ARC_H
#define ARC_H

#include <stdint.h>

class Arc {

 public:
 Arc(): cap(0), flow(0), cost(0) {
 }

 Arc(int32_t capacity, int32_t flw, int32_t cst, Arc* rvrd_arc):
  cap(capacity), flow(flw), cost(cst), reverse_arc(rvrd_arc) {
 }

  int32_t get_cap();
  int32_t get_flow();
  int32_t get_cost();
  Arc* get_reverse_arc();
  void set_reverse_arc(Arc* arc);

  int32_t cap;
  int32_t flow;
  int32_t cost;
  Arc* reverse_arc;

};
#endif
