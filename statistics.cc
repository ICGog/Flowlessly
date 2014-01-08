#include "statistics.h"

#include <glog/logging.h>
#include <sys/time.h>
#include <sys/resource.h>

namespace flowlessly {

  double Statistics::getTime() {
    struct rusage r;
    getrusage(0, &r);
    return r.ru_utime.tv_sec + (double)r.ru_utime.tv_usec / 1000000.0;
  }

  void Statistics::logTimeStatistics() {
    LOG(INFO) << "Refine time: " << refine_time;
    LOG(INFO) << "Discharge time: " << discharge_time;
    LOG(INFO) << "Global update time: " << global_update_time;
    LOG(INFO) << "Price refine time: " << price_refine_time;
    LOG(INFO) << "Arcs fixing time: " << arcs_fixing_time;
    LOG(INFO) << "Arcs unfixing time: " << arcs_unfixing_time;
    LOG(INFO) << "Relabel time: " << relabel_time;
    LOG(INFO) << "Push time: " << push_time;
    LOG(INFO) << "Update admisible time: " << update_admisible_time;
  }

  void Statistics::clearStatistics() {
    refine_time = 0;
    discharge_time = 0;
    global_update_time = 0;
    price_refine_time = 0;
    arcs_fixing_time = 0;
    arcs_unfixing_time = 0;
    relabel_time = 0;
    push_time = 0;
    update_admisible_time = 0;
  }

  double Statistics::get_arcs_fixing_time() {
    return arcs_fixing_time;
  }

  void Statistics::update_arcs_fixing_start_time() {
    if (FLAGS_log_statistics) {
      arcs_fixing_time -= getTime();
    }
  }

  void Statistics::update_arcs_fixing_end_time() {
    if (FLAGS_log_statistics) {
      arcs_fixing_time += getTime();
    }
  }

  double Statistics::get_arcs_unfixing_time() {
    return arcs_unfixing_time;
  }

  void Statistics::update_arcs_unfixing_start_time() {
    if (FLAGS_log_statistics) {
      arcs_unfixing_time -= getTime();
    }
  }

  void Statistics::update_arcs_unfixing_end_time() {
    if (FLAGS_log_statistics) {
      arcs_unfixing_time += getTime();
    }
  }

  double Statistics::get_refine_time() {
    return refine_time;
  }

  void Statistics::update_refine_start_time() {
    if (FLAGS_log_statistics) {
      refine_time -= getTime();
    }
  }

  void Statistics::update_refine_end_time() {
    if (FLAGS_log_statistics) {
      refine_time += getTime();
    }
  }

  double Statistics::get_discharge_time()  {
    return discharge_time;
  }

  void Statistics::update_discharge_start_time()  {
    if (FLAGS_log_statistics) {
      discharge_time -= getTime();
    }
  }

  void Statistics::update_discharge_end_time()  {
    if (FLAGS_log_statistics) {
      discharge_time += getTime();
    }
  }

  double Statistics::get_global_update_time() {
    return global_update_time;
  }

  void Statistics::update_global_update_start_time() {
    if (FLAGS_log_statistics) {
      global_update_time -= getTime();
    }
  }

  void Statistics::update_global_update_end_time() {
    if (FLAGS_log_statistics) {
      global_update_time += getTime();
    }
  }

  double Statistics::get_price_refine_time()  {
    return price_refine_time;
  }

  void Statistics::update_price_refine_start_time()  {
    if (FLAGS_log_statistics) {
      price_refine_time -= getTime();
    }
  }

  void Statistics::update_price_refine_end_time()  {
    if (FLAGS_log_statistics) {
      price_refine_time += getTime();
    }
  }

  double Statistics::get_relabel_time() {
    return relabel_time;
  }

  void Statistics::update_relabel_start_time() {
    if (FLAGS_log_statistics) {
      relabel_time -= getTime();
    }
  }

  void Statistics::update_relabel_end_time() {
    if (FLAGS_log_statistics) {
      relabel_time += getTime();
    }
  }

  double Statistics::get_push_time()  {
    return push_time;
  }

  void Statistics::update_push_start_time()  {
    if (FLAGS_log_statistics) {
      push_time -= getTime();
    }
  }

  void Statistics::update_push_end_time()  {
    if (FLAGS_log_statistics) {
      push_time += getTime();
    }
  }

  double Statistics::get_update_admisible_time() {
    return update_admisible_time;
  }

  void Statistics::update_admisible_start_time() {
    if (FLAGS_log_statistics) {
      update_admisible_time -= getTime();
    }
  }

  void Statistics::update_admisible_end_time() {
    if (FLAGS_log_statistics) {
      update_admisible_time += getTime();
    }
  }

}
