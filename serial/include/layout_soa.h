// include/layout_soa.h
#pragma once
#include <vector>
#include <limits>
#include <cstdint>

struct AStarStateSoA {
    std::vector<float>   g;
    std::vector<float>   f;
    std::vector<int>     parent;
    std::vector<uint8_t> state; // 0=unvisited,1=open,2=closed

    explicit AStarStateSoA(int N)
        : g(N, std::numeric_limits<float>::infinity()),
          f(N, std::numeric_limits<float>::infinity()),
          parent(N, -1),
          state(N, 0) {}
};

using AStarState = AStarStateSoA;

// accessors
inline float get_g(const AStarState &s, int idx)       { return s.g[idx]; }
inline void  set_g(AStarState &s, int idx, float v)    { s.g[idx] = v; }

inline float get_f(const AStarState &s, int idx)       { return s.f[idx]; }
inline void  set_f(AStarState &s, int idx, float v)    { s.f[idx] = v; }

inline int   get_parent(const AStarState &s, int idx)  { return s.parent[idx]; }
inline void  set_parent(AStarState &s, int idx, int p) { s.parent[idx] = p; }

inline uint8_t get_state(const AStarState &s, int idx) { return s.state[idx]; }
inline void    set_state(AStarState &s, int idx, uint8_t v) { s.state[idx] = v; }
