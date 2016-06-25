// The Flowlessly project
// Copyright (c) 2013-2016 Ionel Gog <ionel.gog@cl.cam.ac.uk>

#ifndef FLOWLESSLY_NODE_H
#define FLOWLESSLY_NODE_H

namespace flowlessly {

// Node type is used to construct the mapping of tasks to pus.
// NOTE: Do not reorder types because it will affect the communication with
// the scheduler.
// OTHER is the first in the enum so that an unset type (i.e. 0) defaults
// to OTHER.
enum NodeType {
  OTHER = 0,
  TASK = 1,
  PU = 2,
  SINK = 3,
  MACHINE = 4,
  INTERMEDIATE_RES = 5
};

enum NodeStatus {
  NOT_VISITED = 0,
  VISITING = 1,
  VISITED = 2
};

class Node {
 public:
  Node() : supply(0), potential(0), distance(0), type(NodeType::OTHER),
    status(NodeStatus::NOT_VISITED) {
  }
  Node(const Node& node)
    : supply(node.supply), potential(node.potential),
    distance(node.distance), type(node.type), status(node.status) {
  }

  int32_t supply;
  int64_t potential;
  int64_t distance; // node's distance from the source (in multiples of eps)
  NodeType type;
  NodeStatus status; // node's status during DFS
};

} // namespace flowlessly
#endif // FLOWLESSLY_NODE_H
