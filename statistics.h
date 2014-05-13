#ifndef STATISTICS_H
#define STATISTICS_H

#include <gflags/gflags.h>

DECLARE_bool(log_statistics);

namespace flowlessly {

  /**
   * Stores statistics related to various operations such as:
   * total time spent relabeling, total time spent arc fixing etc.
   **/
  class Statistics {

  public:
    double getTime();
    void logTimeStatistics();
    void clearStatistics();
    double get_arcs_fixing_time();
    void update_arcs_fixing_start_time();
    void update_arcs_fixing_end_time();
    double get_arcs_unfixing_time();
    void update_arcs_unfixing_start_time();
    void update_arcs_unfixing_end_time();
    double get_refine_time();
    void update_refine_start_time();
    void update_refine_end_time();
    double get_discharge_time();
    void update_discharge_start_time();
    void update_discharge_end_time();
    double get_global_update_time();
    void update_global_update_start_time();
    void update_global_update_end_time();
    double get_price_refine_time();
    void update_price_refine_start_time();
    void update_price_refine_end_time();
    double get_relabel_time();
    void update_relabel_start_time();
    void update_relabel_end_time();
    double get_push_time();
    void update_push_start_time();
    void update_push_end_time();
    double get_update_admisible_time();
    void update_admisible_start_time();
    void update_admisible_end_time();

    double arcs_fixing_time;
    double arcs_unfixing_time;
    double refine_time;
    double discharge_time;
    double global_update_time;
    double price_refine_time;
    double relabel_time;
    double push_time;
    double update_admisible_time;

  };

}
#endif
