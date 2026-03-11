// src/astar.cpp
#include "helper.h"
#include "astar_interface.h"
#if defined(USE_AOS)
    #include "layout_aos.h"
#elif defined(USE_SOA)
    #include "layout_soa.h"
#else
    #error "Must define either USE_AOS or USE_SOA"
#endif

#include <queue>
#include <vector>

//shared min heap to find the next node to expand
struct OpenEntry {
    int   idx;
    float f;
};

struct CompareF {//priorit-queue is max heap, change it to min heap
    bool operator()(const OpenEntry &a, const OpenEntry &b) const {
        return a.f > b.f; // min-heap
    }
};

using OpenSet = std::priority_queue<
    OpenEntry,
    std::vector<OpenEntry>,
    CompareF
>;

static inline float heuristic_manhattan(const GridMap &map, int idx) {
    int W = map.width;
    int x = idx % W;//1-D layout back to x, y
    int y = idx / W;
    int dx = x - map.goal_x;
    int dy = y - map.goal_y;
    if (dx < 0) dx = -dx;//abs value
    if (dy < 0) dy = -dy;
    return float(dx + dy);
}

std::vector<float> precompute_heuristic_manhattan(const GridMap &map) {
    int N = map.width * map.height;
    std::vector<float> h(N);
    for (int idx = 0; idx < N; ++idx) {
        h[idx] = heuristic_manhattan(map, idx);
    }
    return h;
}

bool run_astar(const GridMap &map,
               AStarState &state,
               const std::vector<float> &h,
               int num_threads,
                AStarTimings *out_tm)   // <- new param
{
    (void)num_threads;            // <- serial version ignores threads

    const int start = map.start_idx;
    const int goal  = map.goal_idx;

    set_g(state, start, 0.0f);
    float h0 = h[start];
    set_f(state, start, h0);
    set_state(state, start, 1);

    OpenSet open;
    open.push(OpenEntry{start, h0});

    while (!open.empty()) {
        OpenEntry cur = open.top();
        open.pop();
        int n = cur.idx;

        if (get_state(state, n) == 2) continue;
        if (n == goal) return true;

        set_state(state, n, 2);

        int W = map.width;
        int x = n % W;
        int y = n / W;

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        float g_n = get_g(state, n);

        for (int k = 0; k < 4; ++k) {
            int nx = x + dx[k];
            int ny = y + dy[k];

            if (!map.in_bounds(nx, ny) || map.is_wall(nx, ny)) continue;

            int m = ny * W + nx;
            if (get_state(state, m) == 2) continue;

            float tentative_g = g_n + 1.0f;
            if (tentative_g < get_g(state, m)) {
                set_g(state, m, tentative_g);

                float h_m = h[m];
                float f   = tentative_g + h_m;
                set_f(state, m, f);
                set_parent(state, m, n);

                open.push(OpenEntry{m, f});
                set_state(state, m, 1);
            }
        }
    }

    return false;
}