// The Flowlessly project
// Copyright (c) 2013-2016 Ionel Gog <ionel.gog@cl.cam.ac.uk>

#include "misc/statistics.h"

#include <glog/logging.h>

namespace flowlessly {

  void Statistics::LogStatistics() {
    LOG(INFO) << "Refine time: " << refine_time_;
    LOG(INFO) << "Discharge time: " << discharge_time_;
    LOG(INFO) << "Global update time: " << global_update_time_;
    LOG(INFO) << "Price refine time: " << price_refine_time_;
    LOG(INFO) << "Arcs fixing time: " << arcs_fixing_time_;
    LOG(INFO) << "Arcs unfixing time: " << arcs_unfixing_time_;
    LOG(INFO) << "Relabel time: " << relabel_time_;
    LOG(INFO) << "Push time: " << push_time_;
    LOG(INFO) << "Update admissible time: " << update_admissible_time_;
    LOG(INFO) << "Number of pushes: " << num_pushes_;
    LOG(INFO) << "Number of refines: " << num_refines_;
    LOG(INFO) << "Number of relabels: " << num_relabels_;
  }

  void Statistics::ResetStatistics() {
    refine_time_ = 0;
    discharge_time_ = 0;
    global_update_time_ = 0;
    price_refine_time_ = 0;
    arcs_fixing_time_ = 0;
    arcs_unfixing_time_ = 0;
    relabel_time_ = 0;
    push_time_ = 0;
    update_admissible_time_ = 0;
    num_pushes_ = 0;
    num_refines_ = 0;
    num_relabels_ = 0;
  }

} // namespace flowlessly
