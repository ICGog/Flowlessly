// The Flowlessly project
// Copyright (c) 2013-2016 Ionel Gog <ionel.gog@cl.cam.ac.uk>

#ifndef FLOWLESSLY_SOLVER_H
#define FLOWLESSLY_SOLVER_H

#include <boost/thread/latch.hpp>

#include "misc/statistics.h"
#include "misc/utils.h"

namespace flowlessly {

class Solver {
 public:
  explicit Solver(Statistics* stats) : stats_(stats), latch_(NULL) {}
  virtual ~Solver() {
    // The solver is not the owner of the stats object.
  }

  inline void LogStatistics() {
    stats_->LogStatistics();
  }

  inline void ResetStatistics() {
    stats_->ResetStatistics();
  }

  /**
   * Must return false if it terminates earlier (i.e. if the thread
   * is interrupted).
   */
  virtual bool Run() = 0;

  virtual void PrepareState() = 0;

  void SetLatch(boost::latch* latch) {
    latch_ = latch;
  }

 protected:
  Statistics* stats_;
  boost::latch* latch_;
};

} // namespace flowlessly
#endif // FLOWLESSLY_SOLVER_H
