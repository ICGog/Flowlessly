#ifndef ARC_H
#define ARC_H

#include <stdint.h>

class Arc {

 public:
 Arc(): cap(0), flow(0), cost(0) {
 }

 Arc(uint32_t src_id, uint32_t dst_id, int32_t capacity, int32_t flw,
     int64_t cst, Arc* rvrd_arc): src_node_id(src_id), dst_node_id(dst_id),
    cap(capacity), flow(flw), cost(cst), reverse_arc(rvrd_arc) {
 }

  int32_t get_cap();
  int32_t get_flow();
  int64_t get_cost();
  uint32_t get_src_node_id();
  uint32_t get_dst_node_id();
  Arc* get_reverse_arc();
  void set_reverse_arc(Arc* arc);

  uint32_t src_node_id;
  uint32_t dst_node_id;
  int32_t cap;
  int32_t flow;
  int64_t cost;
  Arc* reverse_arc;

};
#endif
