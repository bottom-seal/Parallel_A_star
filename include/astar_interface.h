// include/astar_iface.h
#pragma once
#include <vector>
#include "helper.h"

#if defined(USE_AOS)
    #include "layout_aos.h"
#elif defined(USE_SOA)
    #include "layout_soa.h"
#endif

std::vector<float> precompute_heuristic_manhattan(const GridMap &map);

struct AStarTimings {
    double overhead_ms = 0.0; // ctx_init + build_map + alloc + seed
    double work_ms     = 0.0; // threads running (create+join included)
    double merge_ms    = 0.0; // export parent chain
    double total_ms    = 0.0; // whole run_astar()
};

bool run_astar(const GridMap& map,
               AStarState& state,
               const std::vector<float>& hvals,
               int num_threads,
            AStarTimings *out_tm = nullptr);


